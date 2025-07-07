#ifndef TIME_SETUP_H
#define TIME_SETUP_H

#include "time.h"
#include <string.h>

struct timeBase {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
  };
  timeBase getTimeBase();
  extern timeBase espTime;

#endif