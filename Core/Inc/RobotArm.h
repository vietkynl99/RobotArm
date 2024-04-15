#ifndef INC_ROBOTARM_H_
#define INC_ROBOTARM_H_

#include "main.h"

#ifdef __cplusplus

#define EXTERNC extern "C"

#else

#define EXTERNC

#endif

EXTERNC void setup();

EXTERNC void loop();

#endif /* INC_ROBOTARM_H_ */
