#pragma once

// Politica portable de prueba manual. No toca HAL, GPIO, timers ni serial.
// Un unico escritor o una seccion critica debe proteger TODAS sus llamadas.
// Un cambio de salida se aplica al hardware en la misma seccion critica.
#include <stdint.h>
#include <math.h>

namespace motor_safety {

enum class State : uint8_t { Disarmed, Armed, Pulse, Fault };
enum FaultBits : uint32_t {
  NONE=0, ESTOP=1u<<0, SENSOR=1u<<1, STALE=1u<<2, ANGLE=1u<<3,
  TIMING=1u<<4, LINK=1u<<5, ENCODER=1u<<6, WATCHDOG=1u<<7,
  CALIBRATION=1u<<8
};
enum class Reject : uint8_t { None, WrongState, Unhealthy, Bounds, Cooldown };

struct Health {
  bool sensorValid=false;
  bool zeroSet=false;
  bool signsVerified=false;
  bool link=false;
  bool timingOK=true;
  bool encoderOK=true;
  float theta=0.0f; // rad, cero vertical ARRIBA
  float omega=0.0f; // rad/s
  uint32_t lastSampleUs=0; // instante de captura del ultimo ciclo completo
  // Numero consecutivo de ciclos NUEVOS y sanos. No incrementar por loop/ISR.
  uint16_t goodSamples=0;
};

class Controller {
 public:
  static constexpr int PWM_LIMIT=50;
  static constexpr uint32_t PULSE_MIN_MS=50, PULSE_MAX_MS=150;
  static constexpr uint32_t ARM_US=5000000, LEASE_US=250000;
  static constexpr uint32_t COOLDOWN_US=1000000, FRESH_US=10000;
  static constexpr uint16_t REQUIRED_GOOD_SAMPLES=100;
  static constexpr float ANGLE_LIMIT_RAD=0.3490658503988659f; // 20 grados
  static constexpr float OMEGA_LIMIT_RAD_S=8.0f;

  State state() const { return state_; }
  uint32_t faults() const { return faults_; }
  Reject lastReject() const { return reject_; }
  int left() const { return left_; }
  int right() const { return right_; }
  bool enabled() const { return state_==State::Pulse; }

  static uint32_t healthFaults(uint32_t now,const Health& h) {
    uint32_t f=NONE;
    if(!h.sensorValid || !isfinite(h.theta) || !isfinite(h.omega)) f|=SENSOR;
    if(uint32_t(now-h.lastSampleUs)>FRESH_US) f|=STALE;
    if(isfinite(h.theta) && fabsf(h.theta)>ANGLE_LIMIT_RAD) f|=ANGLE;
    if(isfinite(h.omega) && fabsf(h.omega)>OMEGA_LIMIT_RAD_S) f|=ANGLE;
    if(!h.link) f|=LINK;
    if(!h.timingOK) f|=TIMING;
    if(!h.encoderOK) f|=ENCODER;
    if(!h.zeroSet || !h.signsVerified) f|=CALIBRATION;
    return f;
  }

  bool arm(uint32_t now,const Health& h) {
    reject_=Reject::None;
    latest_=h;
    if(state_!=State::Disarmed) return reject(Reject::WrongState);
    if(!ready(now,h)) return reject(Reject::Unhealthy);
    if(cooling_ && uint32_t(now-cooldownStart_)<COOLDOWN_US)
      return reject(Reject::Cooldown);
    cooling_=false;
    state_=State::Armed;
    armStart_=leaseStart_=now;
    left_=right_=0;
    return true;
  }

  // El wrapper debe actualizar serviceTick(now,health) inmediatamente antes
  // de pulse, bajo la misma seccion critica, para usar la salud mas reciente.
  bool pulse(uint32_t now,int left,int right,uint32_t durationMs) {
    reject_=Reject::None;
    if(state_!=State::Armed) return reject(Reject::WrongState);
    serviceTick(now,latest_); // tambien verifica frescura al instante del mando
    if(state_!=State::Armed)
      return reject(state_==State::Fault?Reject::Unhealthy:Reject::WrongState);
    if(left < -PWM_LIMIT || left > PWM_LIMIT ||
       right < -PWM_LIMIT || right > PWM_LIMIT ||
       (left==0 && right==0) ||
       durationMs<PULSE_MIN_MS || durationMs>PULSE_MAX_MS)
      return reject(Reject::Bounds); // nunca saturar silenciosamente un mando
    left_=left;right_=right;
    pulseStart_=now;pulseDuration_=durationMs*1000u;
    state_=State::Pulse;
    return true;
  }

  // Solo un comando explicito de heartbeat puede renovar este plazo.
  // Heartbeat no arma y no puede resucitar un plazo ya vencido.
  bool heartbeat(uint32_t now) {
    reject_=Reject::None;
    if(state_!=State::Armed && state_!=State::Pulse)
      return reject(Reject::WrongState);
    if(uint32_t(now-leaseStart_)>=LEASE_US) {
      trip(now,LINK);
      return reject(Reject::Unhealthy);
    }
    if(uint32_t(now-armStart_)>=ARM_US) {
      stop(now);
      return reject(Reject::WrongState);
    }
    leaseStart_=now;
    return true;
  }

  void serviceTick(uint32_t now,const Health& h) {
    latest_=h;
    if(state_==State::Disarmed || state_==State::Fault) return;
    uint32_t f=healthFaults(now,h);
    if(h.goodSamples<REQUIRED_GOOD_SAMPLES) f|=SENSOR;
    if(uint32_t(now-leaseStart_)>=LEASE_US) f|=LINK;
    if(f!=NONE) {trip(now,f);return;}
    if(uint32_t(now-armStart_)>=ARM_US) {stop(now);return;}
    if(state_==State::Pulse && uint32_t(now-pulseStart_)>=pulseDuration_)
      stop(now); // un solo pulso por ARM; no repeticion automatica
  }

  // Parada normal: conserva todo enclavamiento existente.
  void stop(uint32_t now) {
    if(state_==State::Pulse) {cooling_=true;cooldownStart_=now;}
    left_=right_=0;
    if(state_!=State::Fault) state_=State::Disarmed;
  }

  void trip(uint32_t now,uint32_t cause) {
    if(cause==NONE) return;
    if(state_==State::Pulse) {cooling_=true;cooldownStart_=now;}
    left_=right_=0;
    faults_|=cause;
    state_=State::Fault;
  }

  // CLEAR solo quita el enclavamiento; ARM debe ser otra accion explicita.
  bool clearFault(uint32_t now,const Health& h) {
    reject_=Reject::None;
    latest_=h;
    if(state_!=State::Fault) return reject(Reject::WrongState);
    if(!ready(now,h)) return reject(Reject::Unhealthy);
    faults_=NONE;left_=right_=0;state_=State::Disarmed;
    return true;
  }

 private:
  static bool ready(uint32_t now,const Health& h) {
    return healthFaults(now,h)==NONE && h.goodSamples>=REQUIRED_GOOD_SAMPLES;
  }
  bool reject(Reject why) {reject_=why;return false;}
  State state_=State::Disarmed;
  Reject reject_=Reject::None;
  uint32_t faults_=NONE;
  int left_=0,right_=0;
  Health latest_;
  uint32_t armStart_=0,leaseStart_=0,pulseStart_=0,pulseDuration_=0;
  uint32_t cooldownStart_=0;
  bool cooling_=false;
};
}
