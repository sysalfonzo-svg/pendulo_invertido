#include <cassert>
#include <iostream>
#include "boot_autostart.h"
using namespace boot_autostart;
int main() {
  Startup a; assert(!a.canTry(100));
  a.begin(100);assert(a.canTry(100));
  assert(a.canTry(100+BOOT_WINDOW_US-1));
  assert(!a.canTry(100+BOOT_WINDOW_US));assert(a.state()==State::Expired);
  assert(!a.canTry(100+BOOT_WINDOW_US+10));
  a.begin(100);a.markStarted();assert(a.state()==State::Started);
  assert(!a.canTry(200));
  a.begin(100);a.disable();assert(!a.canTry(101));
  a.begin(0xfffffff0u);assert(a.canTry(20));
  std::cout<<"PASS auto power-on startup one-shot, timeout, inhibit, rollover\n";
}
