#ifndef TIME_SETUP_H
#define TIME_SETUP_H

#include "time.h"

struct timeBase {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
  };
  timeBase getTimeBase();

  String month[12] = {
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"
  };

#endif