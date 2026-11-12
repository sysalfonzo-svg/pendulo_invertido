#include <Arduino.h>
#include <HardwareTimer.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "state_math.h"
#include "motor_safety.h"
#include "motor_hardware.h"
#include "lqr_force_model.h"
#include "auto_lqr.h"
#include "boot_autostart.h"
#include "command_parse.h"

// Integracion opcional: transmisor unilateral STM32 -> ESP32-WROOM-32.
// 1: UART6_TX PA11 + DMA2 Stream6 Channel5 (sin USB nativo).
// 0: comportamiento USB CDC original, sin telemetria ESP.
#ifndef ENABLE_ESP32_TX
#define ENABLE_ESP32_TX 1
#endif

#if ENABLE_ESP32_TX && (defined(USBD_USE_CDC) || defined(USBCON) || defined(PIO_FRAMEWORK_ARDUINO_ENABLE_CDC) || defined(PIO_FRAMEWORK_ARDUINO_ENABLE_CDC_WITHOUT_SERIAL))
#error "PA11 USART6_TX es incompatible con USB CDC. Retirar flags USB de platformio.ini; flashear por ST-Link."
#endif

#ifndef ENABLE_MANUAL_MOTORS
#define ENABLE_MANUAL_MOTORS 1
#endif
#ifndef ENABLE_AUTO_LQR
#define ENABLE_AUTO_LQR 0
#endif
#ifndef ENABLE_AUTOSTART_LQR
#define ENABLE_AUTOSTART_LQR 0
#endif

// Se conserva adquisicion de 4 estados y mando manual protegido.
// LQR autonomo desde el encendido de STM32: sin botones, sin cliente MATLAB.
// Base exacta: backups/adquisicion_validada_2026-10-31/ (SHA256 verificado).
// La ley LQR y el inversor de velocidad->PWM son EXPERIMENTALES y protegidos.
constexpr uint32_t EN_L=PA9,EN_R=PA8,L1=PB14,L2=PB15,R1=PB12,R2=PB13;
constexpr uint32_t LA=PA6,LB=PA7,RA=PA0,RB=PA1;
constexpr int DEFAULT_SIGN_L=1,DEFAULT_SIGN_R=-1,DEFAULT_SIGN_THETA=-1;
constexpr uint16_t DEFAULT_ZERO_RAW=2650; // registro fisico de 2026-10-31
// Sentidos originales reproducidos en motor_hardware.h.
constexpr bool INVERTIR_L=false,INVERTIR_R=true;
constexpr int PWM_MAX=motor_safety::Controller::PWM_LIMIT;
constexpr uint32_t TS_US=5000,ACQ_TIMEOUT_US=1500;
constexpr float FILTER_TAU_S=0.020f;
// Ajuste fino opcional: valor que lee theta cuando el pendulo esta VERTICAL.
// Ej.: si en la vertical real theta=+0.5 grados, poner +0.5 aqui.
// Mantener 0.0 hasta medir: NO usar trim para ocultar fallos mecanicos.
constexpr float ANGLE_ZERO_TRIM_DEG=0.0f;
constexpr uint16_t AS5600_ADDR=0x36<<1;
constexpr uint8_t SENSOR_BAD=1,ZERO_PENDING=2,SIGNS_PENDING=4,ESTOP=8;

struct CriticalGuard {
  uint32_t saved;
  CriticalGuard():saved(__get_PRIMASK()) {__disable_irq();__DMB();}
  ~CriticalGuard() {__DMB();__set_PRIMASK(saved);}
};
// HAL tick is 1 kHz, SysTick priority 0. Call ONLY inside CriticalGuard.
// Core micros() does not compensate a SysTick pending while PRIMASK=1.
// All critical sections here are bounded and far shorter than one ms.
uint32_t criticalMicros() {
  uint32_t ms=HAL_GetTick();
  uint32_t ticks=SysTick->VAL;
  const uint32_t reload=SysTick->LOAD+1u;
  if(SCB->ICSR&SCB_ICSR_PENDSTSET_Msk) {++ms;ticks=SysTick->VAL;}
  return ms*1000u+((reload-ticks)*1000u)/reload;
}
motor_safety::Controller motorPolicy;
boot_autostart::Startup bootController;
auto_lqr::Session autoSession;
motor_safety::Health motorHealth;
bool physicalSignsVerified=false;
bool watchdogReady=false;
IWDG_HandleTypeDef controlWatchdog={};
volatile uint32_t lastMainProgressUs=0,maxLoopUs=0,maxCycleUs=0;
float latestX=0,latestVelocity=0,armX=0,forceDiagnostic=NAN;
constexpr uint32_t TRAVEL_FAULT=1u<<9;
constexpr float TRAVEL_LIMIT_M=0.10f;
uint32_t previousInvalidL=0,previousInvalidR=0;

bool motorSessionActive() {
  return motorPolicy.state()==motor_safety::State::Armed ||
         motorPolicy.state()==motor_safety::State::Pulse || autoSession.active();
}
// Called either from TIM2 ISR or under CriticalGuard in foreground.
// En PA11_TX no hay canal de comandos ni DTR. El modo manual queda desarmado.
// Solo el autoarranque independiente, si ya fue habilitado conscientemente,
// conserva el permiso de enlace original (las otras protecciones siguen intactas).
bool controlLinkPresent() {
#if ENABLE_ESP32_TX
  return ENABLE_AUTOSTART_LQR != 0;
#else
  return ENABLE_AUTOSTART_LQR ? true : Serial.dtr();
#endif
}

motor_safety::Health currentHealthLocked(uint32_t now,bool schedulerOK=true) {
  auto health=motorHealth;
  health.link=controlLinkPresent();
  health.signsVerified=health.signsVerified&&physicalSignsVerified;
  health.timingOK=health.timingOK&&schedulerOK&&watchdogReady&&
                 (uint32_t)(now-lastMainProgressUs)<=10000u;
  return health;
}
void safetyStepLocked(uint32_t now,bool schedulerOK=true) {
  auto health=currentHealthLocked(now,schedulerOK);
  if((motorPolicy.state()==motor_safety::State::Armed ||
      motorPolicy.state()==motor_safety::State::Pulse) &&
     fabsf(latestX-armX)>TRAVEL_LIMIT_M)
    motorPolicy.trip(now,TRAVEL_FAULT);
  motorPolicy.serviceTick(now,health);
  autoSession.service(now,health,latestX,latestVelocity);
  if(!motorPolicy.enabled() && !autoSession.running()) motor_hardware::off();
}
void tripMotors(uint32_t cause) {
  if(ENABLE_AUTOSTART_LQR) bootController.disable();
  CriticalGuard guard;
  motorPolicy.trip(criticalMicros(),cause);
  autoSession.trip(cause);motor_hardware::off();
}

volatile uint32_t countL=0,countR=0,invalidL=0,invalidR=0;
volatile uint8_t prevL=0,prevR=0;
const int8_t QUAD[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
uint8_t ab(uint32_t a,uint32_t b) {return (digitalRead(a)?2:0)|(digitalRead(b)?1:0);}
void irqL() {
  uint8_t v=ab(LA,LB),old=prevL;
  countL+=QUAD[(old<<2)|v]; // unsigned modular arithmetic avoids signed overflow
  if((old^v)==3) ++invalidL;
  prevL=v;
}
void irqR() {
  uint8_t v=ab(RA,RB),old=prevR;
  countR+=QUAD[(old<<2)|v];
  if((old^v)==3) ++invalidR;
  prevR=v;
}
void stopMotors() {
  CriticalGuard guard;
  motorPolicy.stop(criticalMicros());autoSession.stop();
  motor_hardware::off(); // giro libre, EN LOW
}

struct Capture {uint32_t seq,t,l,r,el,er,late;int pwmL,pwmR;};
Capture capture,work;
volatile bool slotBusy=false,slotReady=false;
volatile uint32_t missed=0;
uint32_t sequence=0,expectedTick=0;
HardwareTimer sampleTimer(TIM2); // independiente del PWM TIM1
void tick() {
  uint32_t now=micros();
  int32_t lag=(int32_t)(now-expectedTick);
  uint32_t late=lag>0?(uint32_t)lag:0,slots=1+late/TS_US;
  expectedTick+=slots*TS_US;sequence+=slots;missed+=slots-1;
  safetyStepLocked(now,late<=100u&&!slotBusy&&slots==1);
  if(slotBusy) {++missed;return;}
  slotBusy=true;
  capture={sequence,now,countL,countR,invalidL,invalidR,late,
    autoSession.running()?autoSession.left():motorPolicy.left(),
    autoSession.running()?autoSession.right():motorPolicy.right()};
  __DMB();slotReady=true;
}

// No Wire: handlers propios I2C1. HAL IT + precheck evita espera BUSY.
// Bus con un solo maestro. No se escribe EEPROM/configuracion del AS5600.
I2C_HandleTypeDef sensorBus={};
uint8_t sensorBytes[3]={}; // STATUS 0x0B, RAW_ANGLE 0x0C..0x0D
volatile bool sensorDone=false,sensorError=false;
volatile uint32_t sensorDoneTime=0;
bool acquiring=false;
uint32_t i2cErrors=0;
extern "C" void I2C1_EV_IRQHandler() {HAL_I2C_EV_IRQHandler(&sensorBus);}
extern "C" void I2C1_ER_IRQHandler() {HAL_I2C_ER_IRQHandler(&sensorBus);}
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) {
  if(h==&sensorBus) {sensorDoneTime=micros();__DMB();sensorDone=true;}
}
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) {
  if(h==&sensorBus) sensorError=true;
}
void initSensorBus() {
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  __HAL_RCC_I2C1_CLK_ENABLE();
  I2C1->CR2&=~(I2C_CR2_ITEVTEN|I2C_CR2_ITBUFEN|I2C_CR2_ITERREN);
  __HAL_RCC_I2C1_FORCE_RESET();__HAL_RCC_I2C1_RELEASE_RESET();
  HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
  memset(&sensorBus,0,sizeof(sensorBus));sensorBus.Instance=I2C1;
  sensorBus.Init.ClockSpeed=400000;sensorBus.Init.DutyCycle=I2C_DUTYCYCLE_2;
  sensorBus.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  sensorBus.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
  sensorBus.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
  sensorBus.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&sensorBus); // register setup only, no transaction/polling
  sensorDone=false;sensorError=false;
  HAL_NVIC_SetPriority(I2C1_EV_IRQn,2,0);HAL_NVIC_SetPriority(I2C1_ER_IRQn,2,0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

// Cola finita. La adquisicion nunca depende de la capacidad USB.
constexpr size_t TX_SLOTS=8,TX_SIZE=480;
struct TxLine {char data[TX_SIZE];size_t size;};
TxLine tx[TX_SLOTS];
size_t txHead=0,txTail=0,txCount=0,txOffset=0;
uint32_t txDrop=0;
bool streaming=false,greeted=false;
bool enqueue(const char *s) {
#if ENABLE_ESP32_TX
  (void)s;return true; // mensajes de consola descartados sin costo de USB
#else
  size_t n=strlen(s);
  if(txCount==TX_SLOTS||n>=TX_SIZE) return false;
  memcpy(tx[txHead].data,s,n);tx[txHead].size=n;
  txHead=(txHead+1)%TX_SLOTS;++txCount;return true;
#endif
}
void clearTx() {txHead=txTail=txCount=txOffset=0;}
void serviceTx() {
#if ENABLE_ESP32_TX
  return; // no existe puerto USB en el perfil PA11_TX
#else
  // USBSerial::operator bool() in STM32duino 2.11 calls delay(10)!
  // dtr() reads the flag directly, without blocking.
  if(!Serial.dtr()||!txCount) return;
  int freeBytes=Serial.availableForWrite();
  if(freeBytes<=0) return;
  size_t n=tx[txTail].size-txOffset;
  if(n>(size_t)freeBytes) n=freeBytes;
  if(n>64) n=64;
  txOffset+=Serial.write((const uint8_t*)tx[txTail].data+txOffset,n);
  if(txOffset==tx[txTail].size) {txOffset=0;txTail=(txTail+1)%TX_SLOTS;--txCount;}
#endif
}
#if ENABLE_AUTO_LQR
const char *identity=ENABLE_AUTOSTART_LQR ?
  "# PENDULO_STATES_V1; LQR_AUTO_BOOT_ANTIOSC_V2; Ts_us=5000; NO_BUTTON; PA0_encoder; poweron=auto_if_healthy; P=estado; S=emergencia\n" :
  "# PENDULO_STATES_V1; MOTOR_MANUAL_V1; LQR_EXPERIMENTAL_V1; Ts_us=5000; boot=DISARMED; ? G H S N Z D V E A K C P T\n";
#else
const char *identity="# PENDULO_STATES_V1; MOTOR_MANUAL_V1; Ts_us=5000; LQR_DISABLED; boot=DISARMED; ? G H S N Z D V E A K C P\n";
#endif
const char *header="# CSV:seq,t_us,dt_us,nL,nR,x_m,v_m_s,theta_rad,omega_rad_s,raw,status,flags,pwmL,pwmR,invalidL,invalidR,missed,tx_drop,i2c_errors,acq_us,late_us,zero_raw,signL,signR,signTheta,v_raw_m_s,omega_raw_rad_s\n";

int signL=1,signR=1,signTheta=1;
bool signsConfirmed=false,zeroSet=false,zeroRequest=false,originRequest=false;
bool hadEncoder=false,hadAngle=false,lastSensorGood=false;
uint16_t zeroRaw=0,previousRaw=0;
uint32_t previousTime=0,previousL=0,previousR=0,lastGoodTime=0,previousAngleTime=0;
int64_t relativeL=0,relativeR=0;
float velocity=0,omega=0, wheelVelocityL=0,wheelVelocityR=0;


#if ENABLE_ESP32_TX
// Paquete binario v1, 50 bytes; little endian; IEEE754 float32.
// [0..1] sync A5 5A, [2] version=1, [3] payloadBytes=44
// [4..7] seq, [8..11] t_us, [12..15] encL, [16..19] encR
// [20..21] pwmL, [22..23] pwmR, [24..25] AS5600 raw (FFFF si invalido)
// [26] status AS5600, [27] flags, [28..31] x_m, [32..35] v_m_s
// [36..39] theta_rad, [40..43] omega_rad_s, [44..47] missed
// [48..49] CRC16 CCITT-FALSE sobre bytes [0..47], LSB primero.
// No se espera ACK; se descarta una trama si el DMA no esta disponible.
constexpr uint32_t ESP_BAUD=115200u;
constexpr uint32_t ESP_DIVIDER=10u; // 200Hz control / 10 = 20Hz telemetria
constexpr size_t ESP_PACKET_BYTES=50u;
static_assert(sizeof(float)==4, "Se requiere IEEE754 float32");
struct EspSnapshot {
  uint32_t seq,t_us,encL,encR;
  int16_t pwmL,pwmR;
  uint16_t raw;
  uint8_t status,flags;
  float x,v,theta,omega;
  uint32_t missed;
};
EspSnapshot espSnapshot={};
bool espPending=false,espReady=false;
uint8_t espFrame[ESP_PACKET_BYTES] __attribute__((aligned(4)))={};
uint32_t espDropped=0,espDmaErrors=0;

void espSnapshotSample(int raw,int status,uint8_t flags,
                       float x,float v,float theta,float omega) {
  if((work.seq%ESP_DIVIDER)!=0u) return;
  if(espPending) {++espDropped;return;}
  espSnapshot.seq=work.seq;
  espSnapshot.t_us=work.t;
  espSnapshot.encL=work.l;
  espSnapshot.encR=work.r;
  espSnapshot.pwmL=(int16_t)work.pwmL;
  espSnapshot.pwmR=(int16_t)work.pwmR;
  espSnapshot.raw=raw>=0?(uint16_t)raw:0xffffu;
  espSnapshot.status=status>=0?(uint8_t)status:0xffu;
  espSnapshot.flags=flags;
  espSnapshot.x=x;
  espSnapshot.v=v;
  espSnapshot.theta=theta;
  espSnapshot.omega=omega;
  espSnapshot.missed=missed;
  espPending=true;
}

void espPutU16(size_t &p,uint16_t u) {
  espFrame[p++]=(uint8_t)u;espFrame[p++]=(uint8_t)(u>>8);
}
void espPutU32(size_t &p,uint32_t u) {
  for(unsigned i=0;i<4;++i) espFrame[p++]=(uint8_t)(u>>(8*i));
}
void espPutFloat(size_t &p,float f) {
  uint32_t bits;
  memcpy(&bits,&f,sizeof(bits));
  espPutU32(p,bits);
}
uint16_t espCrc16(const uint8_t *data,size_t length) {
  uint16_t crc=0xffffu;
  for(size_t i=0;i<length;++i) {
    crc^=(uint16_t)data[i]<<8;
    for(unsigned j=0;j<8;++j)
      crc=(crc&0x8000u)?(uint16_t)((crc<<1)^0x1021u):(uint16_t)(crc<<1);
  }
  return crc;
}

// Arranque directo de USART6 y DMA, sin HAL_UART, callbacks ni ISR de telemetria.
void initEspTelemetry() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio={};
  gpio.Pin=GPIO_PIN_11;gpio.Mode=GPIO_MODE_AF_PP;
  gpio.Pull=GPIO_PULLUP;gpio.Speed=GPIO_SPEED_FREQ_LOW;
  gpio.Alternate=GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOA,&gpio);

  __HAL_RCC_USART6_CLK_ENABLE();
  USART6->CR1=0u;USART6->CR2=0u;USART6->CR3=0u;
  const uint32_t pclk=HAL_RCC_GetPCLK2Freq();
  USART6->BRR=(pclk+(ESP_BAUD/2u))/ESP_BAUD; // 16x oversampling, 8N1
  USART6->CR3=USART_CR3_DMAT;
  USART6->CR1=USART_CR1_TE|USART_CR1_UE;

  __HAL_RCC_DMA2_CLK_ENABLE();
  DMA2_Stream6->CR&=~DMA_SxCR_EN;
  if(DMA2_Stream6->CR&DMA_SxCR_EN) {espReady=false;return;}
  DMA2_Stream6->CR=0u;
  DMA2_Stream6->FCR=0u; // modo directo, FIFO deshabilitado
  DMA2_Stream6->PAR=(uint32_t)&USART6->DR;
  DMA2_Stream6->M0AR=(uint32_t)espFrame;
  DMA2->HIFCR=DMA_HIFCR_CFEIF6|DMA_HIFCR_CDMEIF6|
              DMA_HIFCR_CTEIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTCIF6;
  // Stream6/Channel5: memoria->periferico, incrementa memoria, prioridad baja.
  DMA2_Stream6->CR=DMA_CHANNEL_5|DMA_SxCR_DIR_0|DMA_SxCR_MINC;
  espReady=true;
}

// Solo en foreground, fuera de la ISR de control y de las secciones criticas.
void serviceEspTelemetry() {
  if(!espReady||!espPending) return;
  if(DMA2_Stream6->CR&DMA_SxCR_EN) return;
  if(!(USART6->SR&USART_SR_TC)) return;
  if(DMA2->HISR&(DMA_HISR_TEIF6|DMA_HISR_DMEIF6|DMA_HISR_FEIF6))
    ++espDmaErrors;
  DMA2->HIFCR=DMA_HIFCR_CFEIF6|DMA_HIFCR_CDMEIF6|
              DMA_HIFCR_CTEIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTCIF6;

  const EspSnapshot &s=espSnapshot;
  espFrame[0]=0xa5;espFrame[1]=0x5a;espFrame[2]=1;espFrame[3]=44;
  size_t p=4;
  espPutU32(p,s.seq);espPutU32(p,s.t_us);
  espPutU32(p,s.encL);espPutU32(p,s.encR);
  espPutU16(p,(uint16_t)s.pwmL);espPutU16(p,(uint16_t)s.pwmR);
  espPutU16(p,s.raw);espFrame[p++]=s.status;espFrame[p++]=s.flags;
  espPutFloat(p,s.x);espPutFloat(p,s.v);
  espPutFloat(p,s.theta);espPutFloat(p,s.omega);
  espPutU32(p,s.missed);
  if(p!=ESP_PACKET_BYTES-2u) {++espDmaErrors;espPending=false;return;}
  espPutU16(p,espCrc16(espFrame,p));
  __DMB();
  DMA2_Stream6->NDTR=(uint16_t)p;
  DMA2_Stream6->M0AR=(uint32_t)espFrame;
  DMA2_Stream6->CR|=DMA_SxCR_EN;
  espPending=false;
}
#endif // ENABLE_ESP32_TX

void finishSample(bool busOk,uint32_t acquisitionUs) {
  uint32_t dtUs=hadEncoder?work.t-previousTime:TS_US;
  float dt=dtUs*1.0e-6f;
  int32_t dl=hadEncoder?state_math::countDelta(work.l,previousL):0;
  int32_t dr=hadEncoder?state_math::countDelta(work.r,previousR):0;
  if(originRequest||!hadEncoder) {
    relativeL=relativeR=0;dl=dr=0;velocity=0;
    wheelVelocityL=wheelVelocityR=0;originRequest=false;
  }
  relativeL+=dl;relativeR+=dr;
  float vRaw=dtUs?(float)state_math::position(dl,dr,signL,signR)/dt:0;
  velocity=state_math::lowpass(velocity,vRaw,dt,FILTER_TAU_S);
  if(dtUs) {
    wheelVelocityL=state_math::lowpass(wheelVelocityL,
      signL*(float)(dl*state_math::METERS_L)/dt,dt,FILTER_TAU_S);
    wheelVelocityR=state_math::lowpass(wheelVelocityR,
      signR*(float)(dr*state_math::METERS_R)/dt,dt,FILTER_TAU_S);
  }
  float x=(float)state_math::position(relativeL,relativeR,signL,signR);
  previousL=work.l;previousR=work.r;previousTime=work.t;hadEncoder=true;
  int raw=-1,status=-1;
  float theta=NAN,wRaw=NAN,wFiltered=NAN;
  bool sensorGood=false;
  if(busOk) {
    status=sensorBytes[0];raw=((sensorBytes[1]&0x0f)<<8)|sensorBytes[2];
    sensorGood=(status&0x38)==0x20; // MD=1, ML=MH=0
  }
  if(sensorGood) {
    uint32_t angleTime=work.t+acquisitionUs;
    float angleDt=(angleTime-previousAngleTime)*1.0e-6f;
    wRaw=hadAngle&&angleDt>0?signTheta*state_math::angleDelta(raw,previousRaw)*state_math::RAD_PER_COUNT/angleDt:0;
    omega=hadAngle?state_math::lowpass(omega,wRaw,angleDt,FILTER_TAU_S):0;
    previousRaw=raw;previousAngleTime=angleTime;hadAngle=true;lastGoodTime=work.t;
    if(zeroRequest) {
      {CriticalGuard guard;physicalSignsVerified=false;}
      zeroRaw=raw;zeroSet=true;zeroRequest=false;omega=0;wRaw=0;
      enqueue("# ZERO_OK: referencia ARRIBA asumida por orden manual; RAM solamente\n");
    }
    theta=zeroSet?(state_math::angle(raw,zeroRaw,signTheta)-ANGLE_ZERO_TRIM_DEG*(state_math::PI_F/180.0f)):NAN;wFiltered=omega;
  } else {
    hadAngle=false;omega=0;zeroRequest=false;
    // Una lectura invalida corta inmediatamente el control. En modo boton,
    // conservar los SIGNOS cargados de la calibracion previa para poder
    // reconocer la falla al recuperarse el sensor (NO rearmar automaticamente).
    if(!ENABLE_AUTOSTART_LQR) physicalSignsVerified=false;
  }
  lastSensorGood=sensorGood;
  forceDiagnostic=sensorGood&&zeroSet ?
    lqr_force_model::diagnosticForceN(x,velocity,theta,wFiltered):NAN;
  bool faultLatched=false;
  {
    CriticalGuard guard;
    const bool timingOK=dtUs>=4900u&&dtUs<=5100u&&work.late<=100u&&
                        acquisitionUs<=ACQ_TIMEOUT_US;
    const bool encoderOK=work.el==previousInvalidL&&work.er==previousInvalidR;
    previousInvalidL=work.el;previousInvalidR=work.er;
    bool good=sensorGood&&zeroSet&&signsConfirmed&&timingOK&&encoderOK&&
              isfinite(theta)&&fabsf(theta)<=motor_safety::Controller::ANGLE_LIMIT_RAD&&
              isfinite(wFiltered)&&fabsf(wFiltered)<=motor_safety::Controller::OMEGA_LIMIT_RAD_S;
    motorHealth.goodSamples=good ? (motorHealth.goodSamples<1000?motorHealth.goodSamples+1:1000) : 0;
    motorHealth.sensorValid=sensorGood;motorHealth.zeroSet=zeroSet;
    motorHealth.signsVerified=signsConfirmed&&physicalSignsVerified;
    motorHealth.theta=theta;motorHealth.omega=wFiltered;
    motorHealth.lastSampleUs=work.t;motorHealth.link=controlLinkPresent();
    motorHealth.timingOK=timingOK;motorHealth.encoderOK=encoderOK;
    latestX=x;latestVelocity=velocity;
    uint32_t now=criticalMicros();
    safetyStepLocked(now);
    if(ENABLE_AUTO_LQR && autoSession.running()) {
      auto_lqr::Drive d=autoSession.update(now,currentHealthLocked(now),
                                          x,velocity,wheelVelocityL,wheelVelocityR);
      if(autoSession.running() && (d.left!=0 || d.right!=0))
        motor_hardware::pulse(d.left,d.right);
      else motor_hardware::off();
    }
    faultLatched=motorPolicy.faults()!=0 || autoSession.faults()!=0;
  }
  uint8_t flags=(sensorGood?0:SENSOR_BAD)|(zeroSet?0:ZERO_PENDING)|
    (signsConfirmed?0:SIGNS_PENDING)|(faultLatched?ESTOP:0);
#if ENABLE_ESP32_TX
  espSnapshotSample(raw,status,flags,x,velocity,theta,wFiltered);
#endif
  if(streaming) {
    if(txCount==TX_SLOTS) ++txDrop;
    else {
      char row[TX_SIZE];
      int n=snprintf(row,sizeof(row),
        "%lu,%lu,%lu,%ld,%ld,%.6f,%.6f,%.6f,%.6f,%d,%d,%u,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%.6f,%.6f\n",
        (unsigned long)work.seq,(unsigned long)work.t,(unsigned long)dtUs,
        (long)state_math::countDelta(work.l,0),(long)state_math::countDelta(work.r,0),
        (double)x,(double)velocity,(double)theta,(double)wFiltered,raw,status,flags,
        work.pwmL,work.pwmR,
        (unsigned long)work.el,(unsigned long)work.er,(unsigned long)missed,
        (unsigned long)txDrop,(unsigned long)i2cErrors,(unsigned long)acquisitionUs,
        (unsigned long)work.late,zeroSet?(int)zeroRaw:-1,signL,signR,signTheta,
        (double)vRaw,(double)wRaw);
      if(n<=0||n>=(int)sizeof(row)||!enqueue(row)) ++txDrop;
    }
  }
  acquiring=false;__DMB();slotBusy=false;
  uint32_t cycleUs=micros()-work.t;
  if(cycleUs>maxCycleUs) maxCycleUs=cycleUs;
  {CriticalGuard guard;safetyStepLocked(criticalMicros(),cycleUs<=2500u);}
  // Watchdog fed ONLY after foreground processed a complete acquisition.
  if(watchdogReady) HAL_IWDG_Refresh(&controlWatchdog);
}
void serviceSample() {
  if(acquiring) {
    if(sensorDone) {
      __DMB();uint32_t age=sensorDoneTime-work.t;
      bool good=!sensorError&&age<=ACQ_TIMEOUT_US;
      if(!good) {++i2cErrors;initSensorBus();}
      finishSample(good,age);
    } else if(sensorError||(uint32_t)(micros()-work.t)>=ACQ_TIMEOUT_US) {
      uint32_t age=micros()-work.t;
      ++i2cErrors;initSensorBus();finishSample(false,age);
    }
    return;
  }
  if(!slotReady) return;
  // ISR cannot overwrite this capture until finishSample clears slotBusy.
  __DMB();work=capture;slotReady=false;sensorDone=false;sensorError=false;
  // Check BEFORE HAL API: HAL otherwise spins when bus remains BUSY.
  if((uint32_t)(micros()-work.t)>=ACQ_TIMEOUT_US||
     (I2C1->SR2&I2C_SR2_BUSY)||sensorBus.State!=HAL_I2C_STATE_READY) {
    uint32_t age=micros()-work.t;
    ++i2cErrors;initSensorBus();finishSample(false,age);return;
  }
  acquiring=true;
  if(HAL_I2C_Mem_Read_IT(&sensorBus,AS5600_ADDR,0x0b,I2C_MEMADD_SIZE_8BIT,sensorBytes,3)!=HAL_OK) {
    ++i2cErrors;initSensorBus();finishSample(false,micros()-work.t);
  }
}

bool reportMotorStatus(bool force=false) {
  static motor_safety::State reportedState=motor_safety::State::Fault;
  static uint32_t reportedFaults=0xffffffffu;
  motor_safety::State state;uint32_t faults,loopMax,cycleMax;
  int l,r;unsigned verified,good;
  {
    CriticalGuard guard;
    state=motorPolicy.state();faults=motorPolicy.faults();
    l=motorPolicy.left();r=motorPolicy.right();verified=physicalSignsVerified;
    good=motorHealth.goodSamples;loopMax=maxLoopUs;cycleMax=maxCycleUs;
  }
  if(!force&&state==reportedState&&faults==reportedFaults) return true;
  const char* names[]={"DISARMED","ARMED","PULSE","FAULT"};
  char msg[320];
  snprintf(msg,sizeof(msg),
    "# MOTOR state=%s faults=%lu verify=%u good=%u pwmL=%d pwmR=%d max_loop_us=%lu max_cycle_us=%lu F_N=%.5f manual_build=%u t_us=%lu\n",
    names[(unsigned)state],(unsigned long)faults,verified,good,l,r,
    (unsigned long)loopMax,(unsigned long)cycleMax,(double)forceDiagnostic,
    (unsigned)ENABLE_MANUAL_MOTORS,(unsigned long)micros());
  if(!enqueue(msg)) return false;
  reportedState=state;reportedFaults=faults;return true;
}
bool reportAutoStatus(bool force=false) {
  static auto_lqr::State lastState=auto_lqr::State::Fault;
  static uint32_t lastFaults=0xffffffffu;
  auto_lqr::State state;uint32_t faults;int l,r;float f;bool sat;
  uint32_t elapsed;
  {
    CriticalGuard guard;
    state=autoSession.state();faults=autoSession.faults();
    l=autoSession.left();r=autoSession.right();f=autoSession.force();
    sat=autoSession.saturated();elapsed=autoSession.elapsedUs(criticalMicros());
  }
  if(!force && state==lastState && faults==lastFaults) return true;
  const char *names[]={"DISARMED","ARMED","RUNNING","FAULT"};
  char msg[280];
  snprintf(msg,sizeof(msg),
    "# AUTO state=%s faults=%lu pwmL=%d pwmR=%d F_N=%.5f saturado=%u elapsed_us=%lu enabled=%u\n",
    names[(unsigned)state],(unsigned long)faults,l,r,(double)f,(unsigned)sat,
    (unsigned long)elapsed,(unsigned)ENABLE_AUTO_LQR);
  if(!enqueue(msg)) return false;
  lastState=state;lastFaults=faults;return true;
}
bool calibrationAllowed() {
  CriticalGuard guard;
  return !motorSessionActive();
}
// Arranque por alimentacion de la STM32, SIN boton ni comandos ni cableado extra.
// No intenta mover hasta tener 100 muestras consecutivas validas y angulo cercano
// al cero calibrado. Una sola activacion por encendido, sin rearme despues de fallo.
void serviceAutoStart() {
  if(!ENABLE_AUTOSTART_LQR || !ENABLE_AUTO_LQR) return;
  bool started=false,expired=false;
  {
    CriticalGuard guard;
    const uint32_t now=criticalMicros();
    safetyStepLocked(now);
    if(motorPolicy.state()==motor_safety::State::Fault ||
       autoSession.state()==auto_lqr::State::Fault) bootController.disable();
    const auto previous=bootController.state();
    if(bootController.canTry(now) &&
       motorPolicy.state()==motor_safety::State::Disarmed &&
       autoSession.state()==auto_lqr::State::Disarmed) {
      const auto health=currentHealthLocked(now);
      if(auto_lqr::Session::healthFaults(now,health,latestX,latestVelocity)==0 &&
         fabsf(health.theta)<=auto_lqr::START_ANGLE_RAD &&
         fabsf(latestVelocity)<=auto_lqr::START_SPEED_LIMIT_M_S) {
        if(autoSession.arm(now,health,latestX,latestVelocity)) {
          if(autoSession.start(now,health,latestX,latestVelocity)) {
            bootController.markStarted();started=true;
          } else autoSession.stop();
        }
      }
    }
    expired=previous==boot_autostart::State::Waiting &&
            bootController.state()==boot_autostart::State::Expired;
    if(!autoSession.running()) motor_hardware::off();
  }
  if(started) enqueue("# AUTO_BOOT_START: LQR activo; PWM protegido, sin limite fijo de tiempo\n");
  if(expired) enqueue("# AUTO_BOOT_EXPIRED: sin arranque; revisar vertical/sensores y reiniciar energia\n");
}

void command(char *s) {
  while(*s==' '||*s=='\t') ++s;
  if(!*s) return;
  size_t length=strlen(s);
  while(length&&isspace((unsigned char)s[length-1])) s[--length]=0;
  char c=toupper((unsigned char)*s++);
  while(*s==' '||*s=='\t') ++s;
  // Stops always take effect, even if trailing characters were received.
  if(c=='S') {tripMotors(motor_safety::ESTOP);enqueue("# ESTOP enclavado\n");return;}
  if(c=='H') {if(ENABLE_AUTOSTART_LQR) bootController.disable();streaming=false;stopMotors();enqueue("# FIN DISARMED\n");return;}
  if((c=='D'||c=='Z'||c=='N'||c=='V')&&!calibrationAllowed()) {
    enqueue("# ERROR: desarmar con H antes de calibrar/verificar\n");return;
  }
  if(c=='D') {
    int v[3];
    if(!command_parse::integers(s,v,3)||
       (v[0]!=1&&v[0]!=-1)||(v[1]!=1&&v[1]!=-1)||(v[2]!=1&&v[2]!=-1)) {
      enqueue("# ERROR: D signL signR signTheta; cada signo +1 o -1\n");return;
    }
    {CriticalGuard guard;physicalSignsVerified=false;motorHealth.goodSamples=0;}
    signL=v[0];signR=v[1];signTheta=v[2];signsConfirmed=true;
    originRequest=true;hadAngle=false;velocity=omega=0;
    enqueue("# SIGNS_OK: origen reiniciado; falta verificacion fisica V 1\n");return;
  }
  if(c=='V') {
    int v;
    if(!command_parse::integers(s,&v,1)||(v!=0&&v!=1)) {
      enqueue("# ERROR: V 1=signos comprobados fisicamente; V 0=revocar\n");return;
    }
    if(v&&(zeroRequest||originRequest||!zeroSet||!signsConfirmed||!lastSensorGood)) {
      enqueue("# VERIFY_REJECT: falta calibracion estable; esperar Z/D/N completados\n");return;
    }
    {CriticalGuard guard;physicalSignsVerified=v!=0;motorHealth.signsVerified=signsConfirmed&&physicalSignsVerified;}
    enqueue(v?"# VERIFY_OK: frente, avance/retroceso y theta declarados comprobados\n":"# VERIFY_REVOKED\n");return;
  }
  if(c=='A') {
    if(strcmp(s,"AUTO")==0) {
      bool ok=false;
      {
        CriticalGuard guard;uint32_t now=criticalMicros();safetyStepLocked(now);
        if(ENABLE_AUTO_LQR && !ENABLE_AUTOSTART_LQR &&
           motorPolicy.state()==motor_safety::State::Disarmed)
          ok=autoSession.arm(now,currentHealthLocked(now),latestX,latestVelocity);
        if(!autoSession.running()) motor_hardware::off();
      }
      enqueue(ok?"# AUTO_ARM_OK: enviar T antes de 5s; heartbeat K cada 50ms\n":
                 "# AUTO_ARM_REJECT: comprobar P, signos, cero y angulo <4 grados\n");
      return;
    }
    if(strcmp(s,"MANUAL")!=0) {enqueue("# ERROR: A MANUAL o A AUTO\n");return;}
    bool ok=false;
    {
      CriticalGuard guard;uint32_t now=criticalMicros();safetyStepLocked(now);
      if(ENABLE_MANUAL_MOTORS && autoSession.state()==auto_lqr::State::Disarmed)
        ok=motorPolicy.arm(now,currentHealthLocked(now));
      if(ok) armX=latestX;
      if(!motorPolicy.enabled() && !autoSession.running()) motor_hardware::off();
    }
    enqueue(ok?"# ARM_OK: un pulso; vigencia 5s; K cada <250ms\n":"# ARM_REJECT: consultar P; no se habilitaron motores\n");return;
  }
  if(c=='E') {
    if(strcmp(s,"CLEAR")!=0) {enqueue("# ERROR: E CLEAR\n");return;}
    bool ok=false;
    {
      CriticalGuard guard;uint32_t now=criticalMicros();
      auto h=currentHealthLocked(now);
      bool needManual=motorPolicy.state()==motor_safety::State::Fault;
      bool needAuto=autoSession.state()==auto_lqr::State::Fault;
      if(needManual||needAuto) {
        bool goodManual=!needManual || (motor_safety::Controller::healthFaults(now,h)==0 &&
                        h.goodSamples>=motor_safety::Controller::REQUIRED_GOOD_SAMPLES);
        bool goodAuto=!needAuto || (auto_lqr::Session::healthFaults(now,h,latestX,latestVelocity)==0);
        if(goodManual&&goodAuto) {
          bool a=!needManual||motorPolicy.clearFault(now,h);
          bool b=!needAuto||autoSession.clear(now,h,latestX,latestVelocity);
          ok=a&&b;
        }
      }
      if(!motorPolicy.enabled() && !autoSession.running()) motor_hardware::off();
    }
    enqueue(ok?"# CLEAR_OK: DISARMED, requiere nueva orden A\n":
               "# CLEAR_REJECT: comprobar P, recuperacion y 100 muestras\n");return;
  }
  if(c=='T') {
    if(*s) {enqueue("# ERROR: T sin argumentos\n");return;}
    bool ok=false;
    {
      CriticalGuard guard;uint32_t now=criticalMicros();safetyStepLocked(now);
      if(ENABLE_AUTO_LQR && !ENABLE_AUTOSTART_LQR)
        ok=autoSession.start(now,currentHealthLocked(now),latestX,latestVelocity);
      if(!ok) motor_hardware::off();
    }
    enqueue(ok?"# AUTO_RUN: control de equilibrio experimental\n":
               "# AUTO_RUN_REJECT: P; enviar A AUTO y mantener K\n");return;
  }
  if(c=='C') {
    int v[3];
    if(!command_parse::integers(s,v,3)||v[2]<0) {
      enqueue("# PULSE_REJECT: C pwmL pwmR ms; enteros, ms 50..150\n");return;
    }
    bool ok=false;
    {
      CriticalGuard guard;uint32_t now=criticalMicros();safetyStepLocked(now);
      if(ENABLE_MANUAL_MOTORS && !autoSession.active())
        ok=motorPolicy.pulse(now,v[0],v[1],(uint32_t)v[2]);
      if(ok) motor_hardware::pulse(motorPolicy.left(),motorPolicy.right());
      else if(!motorPolicy.enabled() && !autoSession.running()) motor_hardware::off();
    }
    enqueue(ok?"# PULSE_OK: corte automatico y desarmado al terminar\n":"# PULSE_REJECT: P; requiere armado, limites y salud validos\n");return;
  }
  if(c=='F'||c=='B'||c=='L'||c=='R') {
    stopMotors();enqueue("# ERROR: F/B/L/R no implementados; use A AUTO + T con cliente K\n");return;
  }
  if(*s) {enqueue("# ERROR: argumentos inesperados\n");return;}
  switch(c) {
    case '?':enqueue(identity);break;
    case 'G':if(enqueue(header)) streaming=true;break;
    case 'P':reportMotorStatus(true);reportAutoStatus(true);break;
    case 'K':{
      CriticalGuard guard;uint32_t now=criticalMicros();
      motorPolicy.heartbeat(now);autoSession.heartbeat(now);
      if(!motorPolicy.enabled()&&!autoSession.running()) motor_hardware::off();
      break;
    }
    case 'N':
      originRequest=true;{CriticalGuard guard;motorHealth.goodSamples=0;}
      enqueue("# ORIGEN carro solicitado; contadores raw conservados\n");break;
    case 'Z':
      if(!lastSensorGood||(uint32_t)(micros()-lastGoodTime)>2*TS_US)
        enqueue("# ZERO_ERROR: sensor no valido/reciente\n");
      else if(fabsf(omega)>0.3f||fabsf(velocity)>0.03f)
        enqueue("# ZERO_ERROR: inmovilice carro y pendulo ARRIBA\n");
      else {
        zeroRequest=true;
        CriticalGuard guard;physicalSignsVerified=false;motorHealth.goodSamples=0;
      }
      break;
    default:enqueue(ENABLE_AUTO_LQR ?
      "# ERROR: ? G H S N Z D V E A K C P T; AUTO experimental\n":
      "# ERROR: ? G H S N Z D V E A K C P; LQR bloqueado\n");break;
  }
}
char line[64];size_t lineN=0;bool overflow=false;
#if !ENABLE_ESP32_TX
void receive() {
  for(int k=0;k<24&&Serial.available()>0;++k) {
    char c=Serial.read();
    if(c=='!') {tripMotors(motor_safety::ESTOP);lineN=0;overflow=false;continue;}
    if(!command_parse::allowedByte((unsigned char)c)) {
      tripMotors(motor_safety::ESTOP);lineN=0;overflow=true;continue;
    }
    if(c=='\r'||c=='\n') {
      if(!overflow&&lineN) {line[lineN]=0;command(line);}
      if(overflow) {tripMotors(motor_safety::ESTOP);enqueue("# ERROR: linea demasiado larga; ESTOP\n");}
      lineN=0;overflow=false;
    } else if(!overflow) {
      if(lineN<sizeof(line)-1) line[lineN++]=c;
      else {overflow=true;tripMotors(motor_safety::ESTOP);}
    }
  }
}
#endif // USB receive

void setup() {
  const uint32_t outputPins[]={EN_L,EN_R,L1,L2,R1,R2};
  for(uint32_t p:outputPins) {digitalWrite(p,LOW);pinMode(p,OUTPUT);}
  motor_hardware::begin();stopMotors();
  pinMode(LA,INPUT_PULLUP);pinMode(LB,INPUT_PULLUP);
  pinMode(RA,INPUT_PULLUP);pinMode(RB,INPUT_PULLUP);
  prevL=ab(LA,LB);prevR=ab(RA,RB);
  attachInterrupt(digitalPinToInterrupt(LA),irqL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(LB),irqL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RA),irqR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RB),irqR,CHANGE);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpio={};
  gpio.Pin=GPIO_PIN_6|GPIO_PIN_7;gpio.Mode=GPIO_MODE_AF_OD;
  gpio.Pull=GPIO_PULLUP;gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;gpio.Alternate=GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB,&gpio); // SCL PB6 / SDA PB7; pullups externos a 3.3V
  initSensorBus();
#if ENABLE_ESP32_TX
  initEspTelemetry(); // USART6 PA11 hacia GPIO16 del ESP32, solo TX
#else
  Serial.begin(115200); // USB CDC sin espera (comportamiento original)
#endif
  if(ENABLE_AUTOSTART_LQR) {
    // Datos anteriores del MISMO montaje. Recalibrar/editar si cambia iman.
    signL=DEFAULT_SIGN_L;signR=DEFAULT_SIGN_R;signTheta=DEFAULT_SIGN_THETA;
    zeroRaw=DEFAULT_ZERO_RAW;zeroSet=true;signsConfirmed=true;
    physicalSignsVerified=true;
    autoSession.setStandaloneMode(true); // modo local: no requiere K ni USB
  }
  bootController.begin(micros()); // antes de detectar IWDG: el fallo debe bloquear autoarranque
  const bool watchdogReset=(RCC->CSR&RCC_CSR_IWDGRSTF)!=0;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  controlWatchdog.Instance=IWDG;
  controlWatchdog.Init.Prescaler=IWDG_PRESCALER_16;
  controlWatchdog.Init.Reload=127; // nominal 64ms at 32kHz LSI; tolerance applies
  watchdogReady=HAL_IWDG_Init(&controlWatchdog)==HAL_OK; // setup only
  if(!watchdogReady||watchdogReset) tripMotors(motor_safety::WATCHDOG);
  sampleTimer.pause();sampleTimer.setOverflow(TS_US,MICROSEC_FORMAT);
  sampleTimer.attachInterrupt(tick);sampleTimer.setInterruptPriority(1,0);
  lastMainProgressUs=micros();expectedTick=micros()+TS_US;sampleTimer.resume();
}
void finishLoop(uint32_t start) {
  uint32_t now=micros(),elapsed=now-start;
  if(elapsed>maxLoopUs) maxLoopUs=elapsed;
  CriticalGuard guard;
  uint32_t safetyNow=criticalMicros();
  safetyStepLocked(safetyNow,elapsed<=2000u);
  lastMainProgressUs=safetyNow;
}
void loop() {
  uint32_t start=micros();
  {CriticalGuard guard;safetyStepLocked(criticalMicros());}
  serviceSample();
  serviceAutoStart();
#if ENABLE_ESP32_TX
  // Esta rama solo transmite; jamas acepta comandos del ESP32.
  // Motor manual queda desarmado. LQR solo arranca si sus flags AUTOSTART
  // ya estaban habilitados y supera TODAS las protecciones originales.
  serviceEspTelemetry();
  finishLoop(start);
#else
  if(!Serial.dtr()) {
    if(!ENABLE_AUTOSTART_LQR) {
      stopMotors();
      {CriticalGuard guard;physicalSignsVerified=false;}
    }
    streaming=false;greeted=false;clearTx();lineN=0;overflow=false;
    finishLoop(start);
    return; // boton puede mantener LQR incluso sin USB
  }
  if(!greeted) greeted=enqueue(identity);
  receive();serviceSample();reportMotorStatus();reportAutoStatus();serviceTx();finishLoop(start);
#endif
}
