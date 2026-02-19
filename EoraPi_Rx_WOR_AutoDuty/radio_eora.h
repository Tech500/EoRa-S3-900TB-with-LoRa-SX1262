#pragma once
#include <RadioLib.h>

extern SX1262 radio;   // <-- declaration only
// in radio_eora.cpp
//volatile bool receivedFlag = false;  // ← must be volatile

void initRadio();
void initBoard();

