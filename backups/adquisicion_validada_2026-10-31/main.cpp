#include <Arduino.h>
#include <HardwareTimer.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "state_math.h"

// ETAPA 1: adquisicion solamente. No existe ruta que active los motores.
// Original: backups/antes_integracion_2026-10-31/.
constexpr uint32_t EN_L=PA9,EN_R=PA8,L1=PB14,L2=PB15,R1=PB12,R2=PB13;
constexpr uint32_t LA=PA6,LB=PA7,RA=PA0,RB=PA1;
// Sentidos originales documentados; no se usan para mover en esta etapa.
constexpr bool INVERTIR_L=false,INVERTIR_R=true;
constexpr int PWM_MAX=70; // limite historico; limite efectivo etapa 1 = 0
constexpr uint32_t TS_US=5000,ACQ_TIMEOUT_US=1500;
constexpr float FILTER_TAU_S=0.020f;
constexpr uint16_t AS5600_ADDR=0x36<<1;
constexpr uint8_t SENSOR_BAD=1,ZERO_PENDING=2,SIGNS_PENDING=4,ESTOP=8;

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
  // EN low -> giro libre; nunca frenado activo. No PWM activo.
  digitalWrite(EN_L,LOW);digitalWrite(EN_R,LOW);
  digitalWrite(L1,LOW);digitalWrite(L2,LOW);digitalWrite(R1,LOW);digitalWrite(R2,LOW);
}

struct Capture {uint32_t seq,t,l,r,el,er,late;};
Capture capture,work;
volatile bool slotBusy=false,slotReady=false;
volatile uint32_t missed=0;
uint32_t sequence=0,expectedTick=0;
HardwareTimer sampleTimer(TIM2); // no modificar TIM1/PA8/PA9
void tick() {
  uint32_t now=micros();
  int32_t lag=(int32_t)(now-expectedTick);
  uint32_t late=lag>0?(uint32_t)lag:0,slots=1+late/TS_US;
  expectedTick+=slots*TS_US;sequence+=slots;missed+=slots-1;
  if(slotBusy) {++missed;return;}
  slotBusy=true;
  capture={sequence,now,countL,countR,invalidL,invalidR,late};
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
bool streaming=false,greeted=false,emergency=false;
bool enqueue(const char *s) {
  size_t n=strlen(s);
  if(txCount==TX_SLOTS||n>=TX_SIZE) return false;
  memcpy(tx[txHead].data,s,n);tx[txHead].size=n;
  txHead=(txHead+1)%TX_SLOTS;++txCount;return true;
}
void clearTx() {txHead=txTail=txCount=txOffset=0;}
void serviceTx() {
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
}
const char *identity="# PENDULO_STATES_V1; Ts_us=5000; MOTORS_LOCKED; ? G H S N Z D\n";
const char *header="# CSV:seq,t_us,dt_us,nL,nR,x_m,v_m_s,theta_rad,omega_rad_s,raw,status,flags,pwmL,pwmR,invalidL,invalidR,missed,tx_drop,i2c_errors,acq_us,late_us,zero_raw,signL,signR,signTheta,v_raw_m_s,omega_raw_rad_s\n";

int signL=1,signR=1,signTheta=1;
bool signsConfirmed=false,zeroSet=false,zeroRequest=false,originRequest=false;
bool hadEncoder=false,hadAngle=false,lastSensorGood=false;
uint16_t zeroRaw=0,previousRaw=0;
uint32_t previousTime=0,previousL=0,previousR=0,lastGoodTime=0,previousAngleTime=0;
int64_t relativeL=0,relativeR=0;
float velocity=0,omega=0;

void finishSample(bool busOk,uint32_t acquisitionUs) {
  uint32_t dtUs=hadEncoder?work.t-previousTime:TS_US;
  float dt=dtUs*1.0e-6f;
  int32_t dl=hadEncoder?state_math::countDelta(work.l,previousL):0;
  int32_t dr=hadEncoder?state_math::countDelta(work.r,previousR):0;
  if(originRequest||!hadEncoder) {
    relativeL=relativeR=0;dl=dr=0;velocity=0;originRequest=false;
  }
  relativeL+=dl;relativeR+=dr;
  float vRaw=dtUs?(float)state_math::position(dl,dr,signL,signR)/dt:0;
  velocity=state_math::lowpass(velocity,vRaw,dt,FILTER_TAU_S);
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
      zeroRaw=raw;zeroSet=true;zeroRequest=false;omega=0;wRaw=0;
      enqueue("# ZERO_OK: referencia ARRIBA asumida por orden manual; RAM solamente\n");
    }
    theta=zeroSet?state_math::angle(raw,zeroRaw,signTheta):NAN;wFiltered=omega;
  } else {hadAngle=false;omega=0;zeroRequest=false;stopMotors();}
  lastSensorGood=sensorGood;
  uint8_t flags=(sensorGood?0:SENSOR_BAD)|(zeroSet?0:ZERO_PENDING)|
    (signsConfirmed?0:SIGNS_PENDING)|(emergency?ESTOP:0);
  if(streaming) {
    if(txCount==TX_SLOTS) ++txDrop;
    else {
      char row[TX_SIZE];
      int n=snprintf(row,sizeof(row),
        "%lu,%lu,%lu,%ld,%ld,%.6f,%.6f,%.6f,%.6f,%d,%d,%u,0,0,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d,%d,%d,%d,%.6f,%.6f\n",
        (unsigned long)work.seq,(unsigned long)work.t,(unsigned long)dtUs,
        (long)state_math::countDelta(work.l,0),(long)state_math::countDelta(work.r,0),
        (double)x,(double)velocity,(double)theta,(double)wFiltered,raw,status,flags,
        (unsigned long)work.el,(unsigned long)work.er,(unsigned long)missed,
        (unsigned long)txDrop,(unsigned long)i2cErrors,(unsigned long)acquisitionUs,
        (unsigned long)work.late,zeroSet?(int)zeroRaw:-1,signL,signR,signTheta,
        (double)vRaw,(double)wRaw);
      if(n<=0||n>=(int)sizeof(row)||!enqueue(row)) ++txDrop;
    }
  }
  acquiring=false;__DMB();slotBusy=false;
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

void command(char *s) {
  while(*s==' '||*s=='\t') ++s;
  if(!*s) return;
  char c=toupper((unsigned char)*s++);
  if(c=='D') {
    int l,r,t;char extra;
    if(sscanf(s,"%d %d %d %c",&l,&r,&t,&extra)!=3||
       (l!=1&&l!=-1)||(r!=1&&r!=-1)||(t!=1&&t!=-1)) {
      enqueue("# ERROR: D signL signR signTheta; cada signo +1 o -1\n");return;
    }
    signL=l;signR=r;signTheta=t;signsConfirmed=true;
    originRequest=true;hadAngle=false;velocity=omega=0;
    enqueue("# SIGNS_OK: sentidos declarados por usuario; origen carro reiniciado\n");return;
  }
  if(c=='C'||c=='F'||c=='B'||c=='L'||c=='R') {
    stopMotors();enqueue("# MOTORS_LOCKED: etapa 1, PWM permitido=0\n");return;
  }
  while(*s==' '||*s=='\t') ++s;
  if(*s) {enqueue("# ERROR: argumentos inesperados\n");return;}
  switch(c) {
    case '?':enqueue(identity);break;
    case 'G':if(enqueue(header)) streaming=true;break;
    case 'H':streaming=false;stopMotors();enqueue("# FIN\n");break;
    case 'S':emergency=true;stopMotors();enqueue("# ESTOP enclavado; medicion continua\n");break;
    case 'N':originRequest=true;enqueue("# ORIGEN carro solicitado; contadores raw conservados\n");break;
    case 'Z':
      if(!lastSensorGood||(uint32_t)(micros()-lastGoodTime)>2*TS_US)
        enqueue("# ZERO_ERROR: sensor no valido/reciente\n");
      else if(fabsf(omega)>0.3f||fabsf(velocity)>0.03f)
        enqueue("# ZERO_ERROR: inmovilice carro y pendulo ARRIBA\n");
      else zeroRequest=true;
      break;
    default:enqueue("# ERROR: ? G H S N Z D; motores bloqueados\n");break;
  }
}
char line[64];size_t lineN=0;bool overflow=false;
void receive() {
  for(int k=0;k<24&&Serial.available()>0;++k) {
    char c=Serial.read();
    if(c=='!') {emergency=true;stopMotors();lineN=0;overflow=false;continue;}
    if(c=='\r'||c=='\n') {
      if(!overflow&&lineN) {line[lineN]=0;command(line);}
      if(overflow) enqueue("# ERROR: linea demasiado larga\n");
      lineN=0;overflow=false;
    } else if(!overflow) {
      if(lineN<sizeof(line)-1) line[lineN++]=c;else overflow=true;
    }
  }
}
void setup() {
  const uint32_t outputPins[]={EN_L,EN_R,L1,L2,R1,R2};
  for(uint32_t p:outputPins) {digitalWrite(p,LOW);pinMode(p,OUTPUT);}
  stopMotors();
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
  initSensorBus();Serial.begin(115200); // nunca esperar por USB
  sampleTimer.pause();sampleTimer.setOverflow(TS_US,MICROSEC_FORMAT);
  sampleTimer.attachInterrupt(tick);sampleTimer.setInterruptPriority(1,0);
  expectedTick=micros()+TS_US;sampleTimer.resume();
}
void loop() {
  serviceSample();
  if(!Serial.dtr()) {
    stopMotors();streaming=false;greeted=false;clearTx();lineN=0;overflow=false;
    return; // adquisicion continua aun sin puerto USB abierto
  }
  if(!greeted) greeted=enqueue(identity);
  receive();serviceSample();serviceTx();
}
