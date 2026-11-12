#include <cassert>
#include <cmath>
#include <iostream>
#include "auto_lqr.h"
using namespace auto_lqr;
static motor_safety::Health ready(uint32_t t) {
  motor_safety::Health h;
  h.sensorValid=true;h.zeroSet=true;h.signsVerified=true;
  h.link=true;h.timingOK=true;h.encoderOK=true;
  h.theta=0;h.omega=0;h.lastSampleUs=t;h.goodSamples=120;
  return h;
}
int main() {
  static_assert(PWM_MAX<=100,"PWM fuera de rango");
  auto d=mapForce(2.3049f,0,0,0);
  assert(d.left<0&&d.right<0);
  assert(abs(d.left)<=PWM_MAX&&abs(d.right)<=PWM_MAX);
  auto b=mapForce(-2.3049f,0,0,0);
  assert(b.left>0&&b.right>0);
  d=mapForce(0,0,0,0);assert(d.left==0&&d.right==0);
  // Los mandos por debajo de 25/30 no se convierten ya en minimos fijos.
  auto small=mapForce(0.3f,0,0,0);
  assert(small.left<0&&abs(small.left)<25);
  assert(small.right<0&&abs(small.right)<30);
  OutputSlew a;
  int sum=0,nonzero=0;
  for(int i=0;i<200;++i){int p=a.step(-10,25);sum+=abs(p);nonzero+=(p!=0);
    assert(p==0||p==-25);
  }
  float avg=sum/200.0f;
  assert(avg>8.0f && avg<11.0f);
  assert(nonzero>0 && nonzero<200);
  assert(a.step(10,25)==0); // ventana de inversion
  assert(a.step(10,25)==0);
  assert(a.step(10,25)==0);
  // Despues del intervalo el signo nuevo puede salir (no necesariamente en
  // el primer ciclo por la modulacion subumbral).
  bool reversed=false;
  for(int i=0;i<20;++i) if(a.step(30,25)>0) reversed=true;
  assert(reversed);
  assert(a.step(0,25)==0);
  Session s;s.setStandaloneMode(true);
  uint32_t now=1000000;auto h=ready(now);
  assert(s.arm(now,h,0,0));assert(s.start(now+1,h,0,0));
  for(int i=0;i<3000;++i) {
    now+=5000;h.lastSampleUs=now;
    h.theta=0.01f;
    auto p=s.update(now,h,0,0,0,0);
    assert(abs(p.left)<=PWM_MAX && abs(p.right)<=PWM_MAX);
    assert(s.running());
  }
  h.theta=0.30f;h.lastSampleUs=now+5000;
  s.update(now+5000,h,0,0,0,0);
  assert(s.state()==State::Fault && s.left()==0 && s.right()==0);
  h.theta=0;h.lastSampleUs=now+10000;
  assert(s.clear(now+10000,h,0,0));
  assert(s.state()==State::Disarmed);
  std::cout<<"PASS: polaridad, dosificacion PWM subumbral, inversion, sesion y seguridad\n";
}
