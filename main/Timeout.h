#ifndef TIMEOUT_H
#define TIMEOUT_H
#include "Arduino.h"

/// A simple helper class to implement a timeout
/// Initialize it with a duration and then check if it already elapsed
class Timeout {
 public:
  /// Creates a new timeout of a certain duration. It is starting NOW.
  Timeout(unsigned long duration) : start(millis()), duration(duration) {}

  /// Checks if the timeout already elapsed since its creation
  bool elapsed() { return millis() - start >= duration; }

 private:
  unsigned long start, duration;
};

#endif  // TIMEOUT_H
