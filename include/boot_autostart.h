#pragma once
#include <stdint.h>
namespace boot_autostart {
constexpr uint32_t BOOT_WINDOW_US=10000000u; // 10 s desde arranque para situar vertical
// No rearme tras un arranque, tras un fallo ni tras vencimiento de ventana.
enum class State: uint8_t {Waiting, Started, Expired, Disabled};
class Startup {
 public:
  void begin(uint32_t now) {start_=now; state_=State::Waiting;}
  bool canTry(uint32_t now) {
    if(state_!=State::Waiting) return false;
    if((uint32_t)(now-start_) >= BOOT_WINDOW_US) {state_=State::Expired;return false;}
    return true;
  }
  void markStarted() {if(state_==State::Waiting) state_=State::Started;}
  void disable() {state_=State::Disabled;}
  State state() const {return state_;}
 private:
  State state_=State::Disabled;
  uint32_t start_=0;
};
}
