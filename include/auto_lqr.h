#pragma once
#include <stdint.h>
#include <math.h>
#include "motor_safety.h"
#include "lqr_force_model.h"

// Primer controlador de EQUILIBRIO EXPERIMENTAL: no es identificacion de fuerza.
// Se implementa el LQR en N y una conversion APROXIMADA a PWM basada SOLO en
// plantas de VELOCIDAD de ruedas al aire. No se garantiza estabilidad.
// En arranque autonomo se exige salud de sensores y casi vertical.
namespace auto_lqr {

constexpr float MASS_EFFECTIVE_KG=1.0f/lqr_force_model::B[1];
constexpr float WHEEL_RADIUS_M=0.0325f;
constexpr float PI=3.14159265358979323846f;
constexpr float WHEEL_M_PER_RPM=2.0f*PI*WHEEL_RADIUS_M/60.0f;
constexpr float LEFT_RPM_PER_PWM=1.3355f;
constexpr float RIGHT_RPM_PER_PWM=1.1062f;
constexpr float LEFT_TAU_S=0.1688f;
constexpr float RIGHT_TAU_S=0.1573f;
constexpr float LEFT_MIN_MOVING_PWM=25.0f; // umbral de banco: NO validado bajo carga
constexpr float RIGHT_MIN_MOVING_PWM=30.0f;
constexpr int PWM_MAX=60; // configurable; limite NO validado bajo carga
// Ajustes de la planta de accionamiento, independientes de K (LQR original).
// 0.85 reduce un 15% la demanda inicial de aceleracion; NO es una ganancia identificada.
constexpr float FORCE_TO_ACCEL_SCALE=0.85f;
constexpr float FORCE_DEADBAND_N=0.04f;
constexpr float SYNC_GAIN=0.25f;
constexpr uint32_t LEASE_US=250000u;
constexpr uint32_t ARM_TIMEOUT_US=5000000u;
constexpr uint32_t FRESH_US=10000u;
constexpr float START_ANGLE_RAD=0.06981317f; // 4 grados
constexpr float ANGLE_LIMIT_RAD=0.26179939f; // 15 grados
constexpr float OMEGA_LIMIT_RAD_S=6.0f;
constexpr float TRAVEL_LIMIT_M=0.20f; // +/-20 cm desde armado
constexpr float SPEED_LIMIT_M_S=0.60f;
constexpr float START_SPEED_LIMIT_M_S=0.03f;
constexpr uint16_t REQUIRED_GOOD=100;
constexpr uint32_t FAULT_TRAVEL=1u<<9, FAULT_TIMEOUT=1u<<10;
constexpr uint32_t FAULT_STATE=1u<<11;

enum class State:uint8_t {Disarmed, Armed, Running, Fault};
struct Drive {int left=0, right=0; float forceN=0; bool saturated=false;};

// Nueva estrategia: el LQR solicita un PWM continuo (virtual). NO lo elevamos
// inmediatamente a 25/30%, porque eso genera una respuesta discontinua y
// puede producir una oscilacion sostenida (ciclo limite) alrededor del cero.
// El actuador aplica pulsos breves al PWM minimo solo cuando se acumula una
// demanda subumbral, conservando aproximadamente su valor medio.
inline Drive mapForce(float fN,float cartV,float wheelVL,float wheelVR) {
  Drive d;d.forceN=fN;
  if(!isfinite(fN)||!isfinite(cartV)||!isfinite(wheelVL)||!isfinite(wheelVR)) return d;
  if(fabsf(fN)<FORCE_DEADBAND_N && fabsf(cartV)<0.012f) return d;
  const float accel=(fN/MASS_EFFECTIVE_KG)*FORCE_TO_ACCEL_SCALE;
  const float leftWanted=cartV+LEFT_TAU_S*accel-SYNC_GAIN*(wheelVL-cartV);
  const float rightWanted=cartV+RIGHT_TAU_S*accel-SYNC_GAIN*(wheelVR-cartV);
  const float pctL=leftWanted/(LEFT_RPM_PER_PWM*WHEEL_M_PER_RPM);
  const float pctR=rightWanted/(RIGHT_RPM_PER_PWM*WHEEL_M_PER_RPM);
  d.saturated=fabsf(pctL)>PWM_MAX || fabsf(pctR)>PWM_MAX;
  // Convencion original del carro: C -40 -40 = avance.
  d.left=-(int)lroundf(fmaxf(-PWM_MAX,fminf(PWM_MAX,pctL)));
  d.right=-(int)lroundf(fmaxf(-PWM_MAX,fminf(PWM_MAX,pctR)));
  return d;
}

// Modulador de mando para vencer rozamiento sin aplicar siempre el PWM minimo.
// El PWM efectivo debajo del umbral se convierte en una densidad de pulsos:
// por ejemplo, un mando 10 y un umbral 25 producen aprox. 40% de ciclos a 25.
// Es una primera aproximacion experimental, no un modelo identificado.
struct OutputSlew {
  int applied=0;
  int direction=0;
  uint8_t reverseTicks=0;
  float virtualMagnitude=0.0f;
  float pulseCredit=0.0f;

  int step(int target,float minimumMovingPwm) {
    if(target>PWM_MAX) target=PWM_MAX;
    if(target< -PWM_MAX) target=-PWM_MAX;
    if(target==0) {
      applied=0;virtualMagnitude=0; pulseCredit=0;
      return 0;
    }
    const int wantedSign=target>0?1:-1;
    if(direction!=0 && wantedSign!=direction) {
      // Salida EN=0 antes de invertir, con los mismos pines y polaridad.
      direction=0;reverseTicks=2;virtualMagnitude=0;pulseCredit=0;
      applied=0;return 0;
    }
    if(reverseTicks>0) {--reverseTicks;applied=0;return 0;}
    if(direction==0) direction=wantedSign;
    const float desired=(float)abs(target);
    const float stepUp=6.0f,stepDown=10.0f;
    if(virtualMagnitude<desired)
      virtualMagnitude=fminf(desired,virtualMagnitude+stepUp);
    else
      virtualMagnitude=fmaxf(desired,virtualMagnitude-stepDown);
    if(virtualMagnitude>=minimumMovingPwm) {
      pulseCredit=0;
      applied=direction*(int)lroundf(virtualMagnitude);
      return applied;
    }
    // Densidad de pulsos a 200 Hz sin signo alternante ni aumento abrupto
    // permanente de un mando de 2% a 25/30%.
    pulseCredit+=virtualMagnitude/minimumMovingPwm;
    if(pulseCredit>=1.0f) {
      pulseCredit-=1.0f;
      applied=direction*(int)lroundf(minimumMovingPwm);
    } else applied=0;
    return applied;
  }
  void reset() {
    applied=0;direction=0;reverseTicks=0;
    virtualMagnitude=0;pulseCredit=0;
  }
};

class Session {
 public:
  State state() const {return state_;}
  uint32_t faults() const {return faults_;}
  int left() const {return left_.applied;}
  int right() const {return right_.applied;}
  bool running() const {return state_==State::Running;}
  bool active() const {return state_==State::Armed||state_==State::Running;}
  void setStandaloneMode(bool enabled) {standaloneMode_=enabled;}
  void setLocalButtonMode(bool enabled) {setStandaloneMode(enabled);} // compatibilidad de pruebas anteriores
  float force() const {return forceN_;}
  bool saturated() const {return saturated_;}
  uint32_t elapsedUs(uint32_t now) const {return state_==State::Running ? uint32_t(now-runStart_):0;}

  static uint32_t healthFaults(uint32_t now,const motor_safety::Health& h,float x,float v) {
    uint32_t f=motor_safety::Controller::healthFaults(now,h);
    if(!isfinite(x)||!isfinite(v)||fabsf(v)>SPEED_LIMIT_M_S) f|=FAULT_STATE;
    if(isfinite(h.theta)&&fabsf(h.theta)>ANGLE_LIMIT_RAD) f|=motor_safety::ANGLE;
    if(isfinite(h.omega)&&fabsf(h.omega)>OMEGA_LIMIT_RAD_S) f|=motor_safety::ANGLE;
    if(h.goodSamples<REQUIRED_GOOD) f|=motor_safety::SENSOR;
    return f;
  }
  bool arm(uint32_t now,const motor_safety::Health& h,float x,float v) {
    if(state_!=State::Disarmed) return false;
    if(healthFaults(now,h,x,v)||fabsf(h.theta)>START_ANGLE_RAD||
       fabsf(v)>START_SPEED_LIMIT_M_S) return false;
    state_=State::Armed;armStart_=now;leaseStart_=now;referenceX_=x;
    resetOutput();return true;
  }
  bool start(uint32_t now,const motor_safety::Health& h,float x,float v) {
    if(state_!=State::Armed) return false;
    service(now,h,x,v);
    if(state_!=State::Armed) return false;
    if(fabsf(h.theta)>START_ANGLE_RAD || fabsf(v)>START_SPEED_LIMIT_M_S) return false;
    runStart_=now;state_=State::Running;resetOutput();return true;
  }
  void heartbeat(uint32_t now) {
    if(!active()) return;
    if(!standaloneMode_ && uint32_t(now-leaseStart_)>=LEASE_US) {trip(motor_safety::LINK);return;}
    leaseStart_=now;
  }
  void service(uint32_t now,const motor_safety::Health& h,float x,float v) {
    if(!active()) return;
    uint32_t f=healthFaults(now,h,x,v);
    if(fabsf(x-referenceX_)>TRAVEL_LIMIT_M) f|=FAULT_TRAVEL;
    if(!standaloneMode_ && uint32_t(now-leaseStart_)>=LEASE_US) f|=motor_safety::LINK;
    if(f) {trip(f);return;}
    if(state_==State::Armed && uint32_t(now-armStart_)>=ARM_TIMEOUT_US) {stop();return;}
    // RUNNING no tiene limite temporal; detiene el boton o una proteccion.
  }
  Drive update(uint32_t now,const motor_safety::Health& h,float x,float v,
               float wheelVL,float wheelVR) {
    Drive result;
    service(now,h,x,v);
    if(!running()) return result;
    const float f=lqr_force_model::diagnosticForceN(x,v,h.theta,h.omega);
    result=mapForce(f,v,wheelVL,wheelVR);
    if(!isfinite(f)) {trip(motor_safety::SENSOR);return Drive();}
    result.left=left_.step(result.left,LEFT_MIN_MOVING_PWM);
    result.right=right_.step(result.right,RIGHT_MIN_MOVING_PWM);
    forceN_=f;saturated_=result.saturated;
    return result;
  }
  void stop() {if(state_!=State::Fault) state_=State::Disarmed;resetOutput();}
  void trip(uint32_t f) {if(!f) return;faults_|=f;state_=State::Fault;resetOutput();}
  bool clear(uint32_t now,const motor_safety::Health& h,float x,float v) {
    if(state_!=State::Fault||healthFaults(now,h,x,v)) return false;
    faults_=0;state_=State::Disarmed;resetOutput();return true;
  }
 private:
  void resetOutput(){left_.reset();right_.reset();forceN_=0;saturated_=false;}
  State state_=State::Disarmed;
  uint32_t faults_=0, armStart_=0,runStart_=0,leaseStart_=0;
  float referenceX_=0,forceN_=0;
  bool saturated_=false;
  bool standaloneMode_=false;
  OutputSlew left_,right_;
};
}
