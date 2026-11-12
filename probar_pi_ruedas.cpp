#include <cassert>
#include <iostream>
#include "../include/pi_ruedas_lqr.h"
using pi_ruedas_lqr::ParMotores;
int main() {
  ParMotores pi;
  const int maxPwm = 100;
  auto o = pi.actualizar(1000000,40,40,maxPwm,0.30f,0.20f,true,true);
  assert(o.izquierda==40 && o.derecha==40);
  // Izquierda mas rapida: bajar PWM izquierda, subir PWM derecha.
  for(uint32_t t=1020000;t<=2000000;t+=20000) {
    o=pi.actualizar(t,40,40,maxPwm,0.30f,0.20f,true,true);
    assert(o.izquierda+o.derecha==80);
    assert(o.izquierda>=0 && o.derecha>=0);
    assert(o.izquierda<=100 && o.derecha<=100);
  }
  assert(o.izquierda<40 && o.derecha>40);
  // Equilibrio de velocidades -> integrador se va deshaciendo gradualmente.
  pi.reiniciar();
  for(uint32_t t=1000000;t<=1200000;t+=20000)
    o=pi.actualizar(t,30,35,maxPwm,0.25f,0.25f,true,true);
  assert(o.izquierda==30 && o.derecha==35);
  // Sentido negativo: la compensacion no invierte la orden.
  pi.reiniciar();
  for(uint32_t t=1000000;t<=1400000;t+=20000)
    o=pi.actualizar(t,-40,-40,maxPwm,-0.30f,-0.20f,true,true);
  assert(o.izquierda+o.derecha==-80);
  assert(o.izquierda<0 && o.derecha<0);
  // Protecciones: parar, sensores invalidos, duty muy bajo, margen saturado.
  o=pi.actualizar(1420000,40,40,maxPwm,0.4f,0.1f,false,true);
  assert(o.izquierda==40 && o.derecha==40);
  o=pi.actualizar(1440000,40,40,maxPwm,0.4f,0.1f,true,false);
  assert(o.izquierda==40 && o.derecha==40);
  o=pi.actualizar(1460000,5,5,maxPwm,0.4f,0.1f,true,true);
  assert(o.izquierda==5 && o.derecha==5);
  o=pi.actualizar(1480000,100,100,maxPwm,0.4f,0.1f,true,true);
  assert(o.izquierda==100 && o.derecha==100);
  // Intervalos excesivos -> resetea PI sin pedir movimiento extra.
  pi.reiniciar();
  o=pi.actualizar(2000000,40,40,maxPwm,0.4f,0.2f,true,true);
  o=pi.actualizar(2100000,40,40,maxPwm,0.4f,0.2f,true,true);
  assert(o.izquierda==40 && o.derecha==40);
  std::cout << "OK: pruebas logicas PI sincronizacion de ruedas\n";
}
