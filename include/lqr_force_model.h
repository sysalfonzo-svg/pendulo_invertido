#pragma once

// Ganancias originales de matlab/preparar_lqr_fuerza.m (sin cambios).
// F=-K_F*X entrega newtons; auto_lqr.h aproxima la actuacion PWM.
// X=[x_m, vx_m_s, theta_rad, omega_rad_s]; theta=0 arriba.
// El accionamiento todavia no esta identificado bajo carga.
namespace lqr_force_model {
constexpr char SOURCE_SHA256[] = "1d03253a3c87b435e75649c1d71c77787b2dd4c8d50c6fb36f4a48a10c46b18b";
constexpr float TS_S = 0.005f;
constexpr float JP_KG_M2 = 0.00253432f;
constexpr float A[4][4] = {
  {0.0f, 1.0f, 0.0f, 0.0f},
  {0.0f, -0.0f, -0.237437418f, 0.0f},
  {0.0f, 0.0f, 0.0f, 1.0f},
  {0.0f, 0.0f, 28.6815343f, -0.0f}
};
constexpr float B[4] = {0.0f, 1.17199177f, 0.0f, -3.34558165f};
constexpr float AD[4][4] = {
  {1.0f, 0.005f, -2.96814508e-06f, -4.94679023e-09f},
  {0.0f, 1.0f, -0.00118732897f, -2.96814508e-06f},
  {0.0f, 0.0f, 1.00035854f, 0.00500059755f},
  {0.0f, 0.0f, 0.14342481f, 1.00035854f}
};
constexpr float BD[4] = {1.46499178e-05f, 0.00585997541f, -4.18222695e-05f, -0.0167299074f};
constexpr float K_F[4] = {-9.58072413f, -8.65868827f, -44.0193959f, -8.04174124f};
constexpr float POLES_REAL[4] = {0.989573973f, 0.989573973f, 0.968035307f, 0.968035307f};
constexpr float POLES_IMAG[4] = {0.00717906939f, -0.00717906939f, 0.0143097321f, -0.0143097321f};
inline float diagnosticForceN(float x, float vx, float theta, float omega) {
  return -(K_F[0]*x + K_F[1]*vx + K_F[2]*theta + K_F[3]*omega);
}
} // namespace lqr_force_model
