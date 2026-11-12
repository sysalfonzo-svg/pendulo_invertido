#pragma once
#include <stdint.h>
#include <math.h>
namespace state_math {
constexpr float PI_F=3.14159265358979323846f;
constexpr float RAD_PER_COUNT=2.0f*PI_F/4096.0f;
constexpr double METERS_L=2.0*3.14159265358979323846*0.0325/1969.6;
constexpr double METERS_R=2.0*3.14159265358979323846*0.0325/1974.0;
inline int32_t countDelta(uint32_t now,uint32_t before) {
  uint32_t d=now-before;
  return d<=INT32_MAX ? (int32_t)d : (int32_t)((int64_t)d-4294967296LL);
}
// Modular difference in [-2048,2047]. Requires motion <pi per sample.
inline int16_t angleDelta(uint16_t now,uint16_t before) {
  return (int16_t)(((int32_t)now-before+6144)%4096-2048);
}
inline float angle(uint16_t raw,uint16_t zero,int sign) {
  int32_t d=sign*angleDelta(raw,zero);
  if(d>=2048) d-=4096;
  return d*RAD_PER_COUNT;
}
inline double position(int64_t nL,int64_t nR,int signL,int signR) {
  return 0.5*(signL*nL*METERS_L+signR*nR*METERS_R);
}
inline float lowpass(float previous,float input,float dt,float tau) {
  return previous+dt/(tau+dt)*(input-previous);
}
}
