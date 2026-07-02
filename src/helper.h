
#pragma once
#ifndef HELPER_H
  #define HELPER_H

  #ifndef I2SCLOCKLESS_ENABLE_STATUS_MESSAGES
    #define I2SCLOCKLESS_ENABLE_STATUS_MESSAGES 0
  #endif

  #if I2SCLOCKLESS_ENABLE_STATUS_MESSAGES
    #define I2SCLOCKLESS_STATUS_PRINTF(...) printf(__VA_ARGS__)
  #else
    #define I2SCLOCKLESS_STATUS_PRINTF(...) \
      do {                                  \
      } while (0)
  #endif

  #define HOW_LONG(name, func)                                                                                                                       \
    do {                                                                                                                                             \
      uint32_t _time1_ = ESP.getCycleCount();                                                                                                        \
      func;                                                                                                                                          \
      uint32_t _time2_ = ESP.getCycleCount() - _time1_;                                                                                              \
      I2SCLOCKLESS_STATUS_PRINTF("The function *** %s *** took %.2f ms or %.2f fps\n", name, (float)_time2_ / 240000, (float)240000000 / _time2_); \
    } while (0)

  #define RUN_SKETCH_FOR(name, duration, func)                                              \
    do {                                                                                    \
      I2SCLOCKLESS_STATUS_PRINTF("Start Sketch: %s\n", name);                               \
      uint32_t _timer1_ = ESP.getCycleCount();                                              \
      uint32_t _timer2_ = ESP.getCycleCount();                                              \
      while ((_timer2_ - _timer1_) / 240000 < duration) {                                   \
        func;                                                                               \
        _timer2_ = ESP.getCycleCount();                                                     \
      }                                                                                     \
      I2SCLOCKLESS_STATUS_PRINTF("End Sketch: %s after %.2fms\n", name, (float)(_timer2_ - _timer1_) / 240000); \
    } while (0)

  #define RUN_SKETCH_N_TIMES(name, ntimes, func)               \
    do {                                                       \
      I2SCLOCKLESS_STATUS_PRINTF("Start Sketch: %s\n", name);  \
      for (int i = 0; i < ntimes; i++) {                       \
        func;                                                  \
      }                                                        \
      I2SCLOCKLESS_STATUS_PRINTF("End Sketch: %s after %d times\n", name, ntimes); \
    }  while (0)

#endif
