// Host-only tests; do not flash.
// g++ -std=c++11 -O2 -Wall -Wextra -pedantic -Iinclude tests/test_motor_safety.cpp -o tests/test_motor_safety.exe
#include "motor_safety.h"
#include <cassert>
#include <climits>
#include <limits>
#include <iostream>
using namespace motor_safety;

static Health healthy(uint32_t t) {
  Health h;
  h.sensorValid=h.zeroSet=h.signsVerified=h.link=true;
  h.lastSampleUs=t;h.goodSamples=100;
  return h;
}
static void tick(Controller& c,uint32_t t) {c.serviceTick(t,healthy(t));}
static void off(const Controller& c) {assert(!c.enabled() && c.left()==0 && c.right()==0);}

int main() {
  Controller c;
  assert(c.state()==State::Disarmed);off(c);
  assert(!c.pulse(0,-40,-40,100));
  assert(!c.heartbeat(0));
  Health h=healthy(0);h.goodSamples=99;
  assert(!c.arm(0,h));
  h.goodSamples=100;h.signsVerified=false;
  assert(!c.arm(0,h));h.signsVerified=true;h.zeroSet=false;
  assert(!c.arm(0,h));
  assert(c.arm(0,healthy(0)));off(c);
  assert(!c.pulse(0,INT_MIN,0,100));
  assert(!c.pulse(0,0,INT_MAX,100));
  assert(!c.pulse(0,51,0,100));
  assert(!c.pulse(0,0,-51,100));
  assert(!c.pulse(0,0,0,100));
  assert(!c.pulse(0,40,40,49));
  assert(!c.pulse(0,40,40,151));
  assert(!c.pulse(0,40,40,UINT_MAX));
  off(c);
  assert(c.pulse(0,-40,-40,150));
  assert(c.left()==-40 && c.right()==-40 && c.enabled());
  assert(!c.pulse(1000,40,40,150)); // no overlap, no inversion in pulse
  assert(c.left()==-40 && c.right()==-40);
  tick(c,149999);assert(c.enabled());
  tick(c,150000);assert(c.state()==State::Disarmed);off(c);
  assert(!c.pulse(150001,-40,-40,100));
  assert(!c.arm(1149999,healthy(1149999)));
  assert(c.lastReject()==Reject::Cooldown);
  assert(c.arm(1150000,healthy(1150000)));
  assert(c.pulse(1150000,50,-50,50));
  c.trip(1150010,ESTOP);off(c);
  tick(c,1150020);assert(c.state()==State::Fault && (c.faults()&ESTOP));
  c.stop(1150030);assert(c.state()==State::Fault);
  assert(!c.arm(2150030,healthy(2150030)));
  assert(!c.heartbeat(2150030));
  h=healthy(2150030);h.goodSamples=0;
  assert(!c.clearFault(2150030,h));
  assert(c.clearFault(2150030,healthy(2150030)));off(c);
  assert(c.state()==State::Disarmed);
  assert(c.arm(2150030,healthy(2150030)));

  // Explicit heartbeat lease: normal sampling never renews it.
  Controller lease;
  assert(lease.arm(0,healthy(0)));
  tick(lease,249999);assert(lease.state()==State::Armed);
  tick(lease,250000);assert(lease.state()==State::Fault);
  assert(lease.faults()&LINK);off(lease);
  assert(!lease.heartbeat(250001));
  Controller late;
  assert(late.arm(0,healthy(0)));
  assert(!late.heartbeat(250000));assert(late.faults()&LINK);
  Controller expiry;
  assert(expiry.arm(0,healthy(0)));
  for(uint32_t t=100000;t<5000000;t+=100000) {
    tick(expiry,t);assert(expiry.heartbeat(t));
  }
  tick(expiry,5000000);assert(expiry.state()==State::Disarmed);off(expiry);

  // Every mandatory fault kills an active pulse and stays latched.
  const uint32_t expected[]={SENSOR,STALE,ANGLE,ANGLE,TIMING,LINK,ENCODER,CALIBRATION,CALIBRATION,SENSOR,SENSOR};
  for(unsigned k=0;k<sizeof(expected)/sizeof(expected[0]);++k) {
    Controller f;assert(f.arm(100,healthy(100)));assert(f.pulse(100,-40,-40,150));
    Health bad=healthy(200);
    switch(k) {
      case 0:bad.sensorValid=false;break;
      case 1:bad.lastSampleUs=uint32_t(200-10001);break;
      case 2:bad.theta=Controller::ANGLE_LIMIT_RAD+0.001f;break;
      case 3:bad.omega=-8.001f;break;
      case 4:bad.timingOK=false;break;
      case 5:bad.link=false;break;
      case 6:bad.encoderOK=false;break;
      case 7:bad.signsVerified=false;break;
      case 8:bad.zeroSet=false;break;
      case 9:bad.theta=std::numeric_limits<float>::quiet_NaN();break;
      case 10:bad.omega=std::numeric_limits<float>::infinity();break;
    }
    f.serviceTick(200,bad);off(f);
    assert(f.state()==State::Fault && (f.faults()&expected[k]));
    tick(f,300);assert(f.state()==State::Fault);
    f.trip(400,ESTOP);assert(f.faults()&ESTOP);assert(f.faults()&expected[k]);
  }

  // Pulse rechecks stale snapshot even without an intervening timer tick.
  Controller stale;
  assert(stale.arm(10,healthy(10)));
  assert(!stale.pulse(10011,-40,-40,100));assert(stale.faults()&STALE);
  // Boundaries are inclusive where specified: fresh<=10 ms, |theta|<=20 deg.
  h=healthy(0);h.theta=-Controller::ANGLE_LIMIT_RAD;h.omega=8;
  assert(Controller::healthFaults(10000,h)==NONE);
  assert(Controller::healthFaults(10001,h)&STALE);

  // All elapsed-time checks cross the micros() rollover safely.
  const uint32_t start=0xffff0000u;
  Controller wrap;assert(wrap.arm(start,healthy(start)));
  assert(wrap.pulse(start,0,-50,100));
  tick(wrap,start+99999u);assert(wrap.enabled());
  tick(wrap,start+100000u);off(wrap);
  assert(!wrap.arm(start+1099999u,healthy(start+1099999u)));
  assert(wrap.arm(start+1100000u,healthy(start+1100000u)));
  assert(wrap.heartbeat(start+1300000u));
  tick(wrap,start+1549999u);assert(wrap.state()==State::Armed);
  tick(wrap,start+1550000u);assert(wrap.state()==State::Fault && (wrap.faults()&LINK));

  Controller boot;
  boot.trip(0,WATCHDOG);assert(boot.state()==State::Fault);off(boot);
  assert(!boot.arm(1,healthy(1)));
  assert(boot.clearFault(1,healthy(1)));assert(boot.state()==State::Disarmed);off(boot);
  std::cout << "PASS motor_safety: startup, bounds, no overlap, deadlines, cooldown, sticky faults, explicit lease, stale health, rollover.\n";
}
