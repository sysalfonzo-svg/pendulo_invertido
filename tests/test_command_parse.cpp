// Host-only protocol parser checks. The serial byte filter must reject an
// embedded NUL before invoking this C-string parser (strlen cannot see past it).
// g++ -std=c++11 -O2 -Wall -Wextra -pedantic -Iinclude tests/test_command_parse.cpp -o tests/test_command_parse.exe
#include "command_parse.h"
#include "motor_safety.h"
#include <cassert>
#include <climits>
#include <cstring>
#include <iostream>

static bool three(const char* s) {int out[3]={};return command_parse::integers(s,out,3);}
int main() {
  // Exhaust the byte alphabet with an independent accepted-character table.
  const char* printable=" !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";
  for(unsigned b=0;b<256;++b) {
    bool expected=b==9||b==10||b==13;
    for(const char* p=printable;*p;++p)
      if(b==static_cast<unsigned char>(*p)) expected=true;
    assert(command_parse::allowedByte(static_cast<unsigned char>(b))==expected);
  }
  const char hiddenSuffix[]={'4','0',' ','4','0',' ','1','0','0','\0',' ','9','9','9','\n'};
  bool safe=true;
  for(unsigned i=0;i<sizeof(hiddenSuffix);++i)
    if(!command_parse::allowedByte(static_cast<unsigned char>(hiddenSuffix[i]))) safe=false;
  assert(!safe); // receive must discard/fault, never pass prefix to integers()
  int out[3]={};
  assert(command_parse::integers(" -40 -40 100 ",out,3));
  assert(out[0]==-40 && out[1]==-40 && out[2]==100);
  assert(command_parse::integers("\t+40\t-40\t050\r\n",out,3));
  assert(out[0]==40 && out[1]==-40 && out[2]==50);
  assert(command_parse::integers("2147483647 -2147483648 0",out,3));
  assert(out[0]==INT_MAX && out[1]==INT_MIN && out[2]==0);
  const char* rejected[]={
    "", " ", "\t\r\n", "40", "40 40", "40 40 100 1", "40 40 100 extra",
    "40.0 40 100", "4e1 40 100", "0x28 40 100", "--40 40 100",
    "+ 40 40 100", "40-40 100", "40 40 100ms", "40 40 nan", "40 40 inf",
    "2147483648 40 100", "-2147483649 40 100", "40 4294967256 100",
    "40 40 4294967396", "999999999999999999999999 40 100",
    "40 -999999999999999999999999 100", "40 40 100\xff"
  };
  for(const char* s:rejected) assert(!three(s));
  const char startsNul[]={'\0','4','0',' ','4','0',' ','1','0','0','\0'};
  assert(!three(startsNul));
  assert(command_parse::integers("-1 +1 -1",out,3));
  assert(out[0]==-1 && out[1]==1 && out[2]==-1);

  // Parseable int limits must still be rejected by the motor policy, never
  // silently converted into a clamped command or negated as INT_MIN.
  motor_safety::Controller c;
  motor_safety::Health h;
  h.sensorValid=h.zeroSet=h.signsVerified=h.link=true;h.goodSamples=100;
  assert(c.arm(0,h));
  assert(command_parse::integers("-2147483648 40 100",out,3));
  assert(!c.pulse(0,out[0],out[1],static_cast<unsigned>(out[2])));
  assert(c.state()==motor_safety::State::Armed && !c.enabled());
  assert(command_parse::integers("40 40 2147483647",out,3));
  assert(!c.pulse(0,out[0],out[1],static_cast<unsigned>(out[2])));
  assert(!c.enabled() && c.left()==0 && c.right()==0);
  std::cout << "PASS command_parse: all 256 bytes, embedded NUL, signed bounds, 32-bit overflow fraud, whitespace, extra fields, decimal junk and policy rejection.\n";
}
