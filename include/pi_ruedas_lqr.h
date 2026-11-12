#pragma once
// PI de sincronizacion para carro-pendulo invertido STM32F401.
// No sustituye el LQR: solamente aplica correcciones pequenas y opuestas
// a sus ordenes de PWM. No requiere Arduino ni ejecuta funciones bloqueantes.
// Las ganancias son INICIALES PARA ENSAYO, NO ganancias identificadas/validadas.
#include <stdint.h>
#include <math.h>

namespace pi_ruedas_lqr {

inline float limitar(float v, float mn, float mx) {
  return v < mn ? mn : (v > mx ? mx : v);
}
inline int limitarEntero(int v, int mn, int mx) {
  return v < mn ? mn : (v > mx ? mx : v);
}
inline int redondear(float x) {
  return (int)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

struct Salida {
  int izquierda;
  int derecha;
  float referencia_m_s;  // Promedio de las velocidades medidas
  float error_izq_m_s;
  float error_der_m_s;
  int correccion_pwm;    // +correccion en izquierda; -correccion en derecha
};

struct PI {
  float integral = 0.0f;  // fraccion del PWM_MAX

  void reiniciar() { integral = 0.0f; }

  float calcular(float error_m_s, float dt_s, float kp, float ki,
                 float limite_fraccion) {
    integral = limitar(integral, -limite_fraccion, limite_fraccion);
    const float nuevaIntegral = limitar(integral + ki * error_m_s * dt_s,
                                       -limite_fraccion, limite_fraccion);
    const float salidaTentativa = kp * error_m_s + nuevaIntegral;
    // Anti-windup por integracion condicional (clamping).
    if (!((salidaTentativa > limite_fraccion && error_m_s > 0.0f) ||
          (salidaTentativa < -limite_fraccion && error_m_s < 0.0f))) {
      integral = nuevaIntegral;
    }
    return limitar(kp * error_m_s + integral,
                   -limite_fraccion, limite_fraccion);
  }
};

class ParMotores {
 public:
  // kp: fraccion PWM_MAX por (m/s) de error.
  // ki: fraccion PWM_MAX por (m/s * s) de error acumulado.
  // Tuning experimental muy conservador; ajustar a partir de ensayos.
  float kp = 0.35f;
  float ki = 0.15f;
  float max_correccion_fraccion = 0.04f; // Maximo 4 % de PWM_MAX por motor
  float umbral_m_s = 0.007f;            // Reduce ruido cerca de cero
  float velocidad_max_m_s = 2.0f;       // Solo filtro de plausibilidad
  float pwm_minimo_fraccion = 0.12f;    // Inactivo a bajos duty cycles

  static constexpr uint32_t PERIODO_US = 20000u; // 50 Hz PI, LQR sigue a 200 Hz
  static constexpr uint32_t MAX_INTERVALO_US = 30000u;

  void reiniciar() {
    izq_.reiniciar(); der_.reiniciar();
    ultimaActualizacionUs_ = 0u;
    iniciado_ = false;
    ultimoSentido_ = 0;
    correccionFraccion_ = 0.0f;
  }

  Salida actualizar(uint32_t tiempo_us,
                    int pwmLqrIzq, int pwmLqrDer, int pwmMax,
                    float velocidadIzq_m_s, float velocidadDer_m_s,
                    bool controlActivo, bool sensoresValidos) {
    Salida salida = {pwmLqrIzq, pwmLqrDer, 0.0f, 0.0f, 0.0f, 0};

    if (pwmMax <= 0 || !controlActivo || !sensoresValidos ||
        !isfinite(velocidadIzq_m_s) || !isfinite(velocidadDer_m_s) ||
        fabsf(velocidadIzq_m_s) > velocidad_max_m_s ||
        fabsf(velocidadDer_m_s) > velocidad_max_m_s ||
        !isfinite(kp) || !isfinite(ki) || kp < 0 || ki < 0 ||
        pwmLqrIzq > pwmMax || pwmLqrIzq < -pwmMax ||
        pwmLqrDer > pwmMax || pwmLqrDer < -pwmMax) {
      reiniciar();
      return salida;
    }

    const int sentidoL = (pwmLqrIzq > 0) - (pwmLqrIzq < 0);
    const int sentidoR = (pwmLqrDer > 0) - (pwmLqrDer < 0);
    const int minPwm = (int)(pwmMax * pwm_minimo_fraccion);
    if (sentidoL == 0 || sentidoR == 0 || sentidoL != sentidoR ||
        absEntero(pwmLqrIzq) < minPwm || absEntero(pwmLqrDer) < minPwm) {
      reiniciar();
      return salida;
    }
    if (ultimoSentido_ != 0 && ultimoSentido_ != sentidoL) reiniciar();
    ultimoSentido_ = sentidoL;

    // Limita las correcciones para no invertir el giro ni exceder PWM_MAX.
    const int a = absEntero(pwmLqrIzq), b = absEntero(pwmLqrDer);
    int margen = menorEntero(menorEntero(a, b), pwmMax - mayorEntero(a,b));
    if (margen < 1) {
      reiniciar();
      return salida;
    }
    const float limite = limitar(max_correccion_fraccion, 0.0f, 1.0f);
    const float margenFraccion = limitar((float)margen / (float)pwmMax, 0.0f, limite);

    salida.referencia_m_s = 0.5f * (velocidadIzq_m_s + velocidadDer_m_s);
    salida.error_izq_m_s = salida.referencia_m_s - velocidadIzq_m_s;
    salida.error_der_m_s = salida.referencia_m_s - velocidadDer_m_s;
    if (fabsf(salida.error_izq_m_s) < umbral_m_s) salida.error_izq_m_s = 0.0f;
    if (fabsf(salida.error_der_m_s) < umbral_m_s) salida.error_der_m_s = 0.0f;

    if (!iniciado_) {
      iniciado_ = true;
      ultimaActualizacionUs_ = tiempo_us;
    } else {
      const uint32_t transcurridoUs = tiempo_us - ultimaActualizacionUs_;
      if (transcurridoUs > MAX_INTERVALO_US) {
        reiniciar();
        // Se rearma sin correcciones en la siguiente muestra valida.
        return salida;
      }
      if (transcurridoUs >= PERIODO_US) {
        const float dt = (float)transcurridoUs * 1.0e-6f;
        const float salidaL = izq_.calcular(salida.error_izq_m_s, dt,
                                           kp, ki, margenFraccion);
        const float salidaR = der_.calcular(salida.error_der_m_s, dt,
                                           kp, ki, margenFraccion);
        // Proyeccion diferencial: mantiene la suma de las ordenes del LQR.
        correccionFraccion_ = limitar((salidaL - salidaR) * 0.5f,
                                     -margenFraccion, margenFraccion);
        ultimaActualizacionUs_ = tiempo_us;
      }
    }

    int delta = redondear(correccionFraccion_ * (float)pwmMax);
    delta = limitarEntero(delta, -margen, margen);
    salida.correccion_pwm = delta;
    salida.izquierda = pwmLqrIzq + delta;
    salida.derecha = pwmLqrDer - delta;
    return salida;
  }

 private:
  static int absEntero(int n) { return n >= 0 ? n : -n; }
  static int menorEntero(int a, int b) { return a < b ? a : b; }
  static int mayorEntero(int a, int b) { return a > b ? a : b; }
  PI izq_, der_;
  uint32_t ultimaActualizacionUs_ = 0u;
  bool iniciado_ = false;
  int ultimoSentido_ = 0;
  float correccionFraccion_ = 0.0f;
};

} // namespace pi_ruedas_lqr
