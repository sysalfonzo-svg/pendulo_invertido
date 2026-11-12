// Host-only checks: do not flash this file to the STM32.
// g++ -std=c++11 -O2 -Wall -Wextra -pedantic -Iinclude tests/test_state_math.cpp -o tests/test_state_math.exe
#include "state_math.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

static void close(double got, double expected, double tolerance=1e-11) {
  assert(std::isfinite(got));
  assert(std::fabs(got-expected)<=tolerance);
}

int main() {
  const double pi=std::acos(-1.0);
  const double countRadians=2*pi/4096;
  std::uint64_t angleCases=0;
  // Every possible pair of 12-bit readings, with both sensor directions.
  // Independently establish shortest circular displacement and half-turn tie.
  for(int old=0;old<4096;++old) {
    for(int now=0;now<4096;++now) {
      int expected=now-old;
      if(expected>=2048) expected-=4096;
      if(expected< -2048) expected+=4096;
      assert(state_math::angleDelta(now,old)==expected);
      for(int sign: {-1,1}) {
        int signedCounts=sign*expected;
        if(signedCounts>=2048) signedCounts-=4096;
        double actual=state_math::angle(now,old,sign);
        close(actual,signedCounts*countRadians,4e-7);
        assert(actual>=-state_math::PI_F && actual<state_math::PI_F);
      }
      ++angleCases;
    }
  }
  assert(state_math::angleDelta(0,4095)==1);
  assert(state_math::angleDelta(4095,0)==-1);
  close(state_math::angle(5,4090,1),11*countRadians,2e-9);

  // Counter rollover in both directions, including every signed endpoint.
  const std::uint32_t origins[]={0u,1u,0x7fffffffu,0x80000000u,0xfffffffeu,0xffffffffu};
  const std::int32_t deltas[]={std::numeric_limits<std::int32_t>::min(),-1974,-1,0,1,1974,
                            std::numeric_limits<std::int32_t>::max()};
  for(auto old:origins) for(auto d:deltas)
    assert(state_math::countDelta(old+static_cast<std::uint32_t>(d),old)==d);
  std::uint32_t random=0x13579bdfu;
  for(int k=0;k<100000;++k) {
    random=1664525u*random+1013904223u;
    const std::uint32_t old=random;
    random=1664525u*random+1013904223u;
    const std::int32_t d=static_cast<std::int32_t>(random%2000001u)-1000000;
    assert(state_math::countDelta(old+static_cast<std::uint32_t>(d),old)==d);
  }

  // Registered R is physical left (1974), registered L is physical right (1969.6).
  const double circumference=2*pi*0.0325;
  close(state_math::position(0,1974,1,1),circumference/2);
  close(state_math::position(9848,0,1,1),5*circumference/2);
  close(state_math::position(9848,9870,1,1),5*circumference);
  close(state_math::position(9848,9870,-1,-1),-5*circumference);
  close(state_math::position(9848,9870,1,-1),0);
  close(state_math::position(-9848,-9870,-1,-1),5*circumference);
  close(state_math::position(1,0,1,1),circumference/(2*1969.6));
  close(state_math::position(0,1,1,1),circumference/(2*1974));

  close(state_math::lowpass(0,1,0.005f,0.020f),0.2,2e-8);
  close(state_math::lowpass(0,1,0.020f,0.020f),0.5,2e-8);
  close(state_math::lowpass(3,-1,0,0.020f),3);
  close(state_math::lowpass(3,-1,0.005f,0),-1);
  float filtered=-2;
  for(int k=0;k<1000;++k) {
    const float next=state_math::lowpass(filtered,1,(k%3+1)*0.005f,0.020f);
    assert(next>=filtered && next<=1);
    filtered=next;
  }
  close(filtered,1,2e-7);
  std::cout << "PASS: " << angleCases << " raw angle pairs, both signs; 100042 counter rollover cases; wheel mapping; variable-dt filters.\n";
}
