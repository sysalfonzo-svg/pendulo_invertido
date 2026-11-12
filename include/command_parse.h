#pragma once
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <ctype.h>
namespace command_parse {
// Validate each incoming byte BEFORE constructing the C string. In particular,
// an embedded NUL must never hide an unparsed suffix of a motor command.
inline bool allowedByte(unsigned char c) {
  return (c>=32u&&c<=126u)||c=='\t'||c=='\r'||c=='\n';
}
inline bool integers(const char* s,int* out,unsigned count) {
  for(unsigned i=0;i<count;++i) {
    while(isspace((unsigned char)*s)) ++s;
    if(!*s) return false;
    char* end=nullptr;errno=0;
    long n=strtol(s,&end,10);
    if(errno==ERANGE||end==s||n<INT_MIN||n>INT_MAX) return false;
    if(*end&&!isspace((unsigned char)*end)) return false;
    out[i]=(int)n;s=end;
  }
  while(isspace((unsigned char)*s)) ++s;
  return !*s;
}
}
