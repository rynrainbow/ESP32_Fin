#include "util.h"

void append2vector(uint8_t* toAppend, size_t s, std::vector<uint8_t>* growBuffer){
  if(s > 0){
    for(int i=0; i < s; i++){
      growBuffer->push_back(toAppend[i]);
    }
  }
}

void vector2array(uint8_t* buffer, size_t* s, std::vector<uint8_t>* growBuffer){
  *s = growBuffer->size();
  for(int i=0; i < *s; i++){
    buffer[i] = growBuffer->at(i);
  }
}