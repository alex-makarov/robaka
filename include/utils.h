#ifndef _ROBAKA_UTILS_H
#define _ROBAKA_UTILS_H

#ifdef ARDUINO_SAM_DUE
//#define vLog SerialUSB.println
#define vLog Serial.println
#else
#define vLog Serial.println
#endif

#ifndef vLog
  #define vLog(x)
#endif


#endif
