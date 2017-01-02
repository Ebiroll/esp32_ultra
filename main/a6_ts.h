#ifndef A6_TS_H
#define A6_TS_H
// A6 GPRS module send data to thingspeak
// remodeled to c from c++ 
#include <stdbool.h>

#define THINGSPEAK_CHANNEL_KEY "8FKJRMLXT2CPYCVO"
#define GPRS_APN "online.telia.se"
#define GPRS_USER "web"
#define GPRS_PASSWORD "web"

/// Timeouts are in milliseconds!
/// To be used for operations for which we expect a fast response like setting
/// parameters on the modem etc.
#define FAST_TIMEOUT 2000

/// To be used for modem operations which might take a little longer like gprs
/// network operations or powering up the modem
#define SLOW_TIMEOUT 60000

#define NUM_TCP_CONNECTION_RETRIES 5

/// A class that sends AT commands to a GPRS modem which make it send data to a
/// thingspeak channel. It was written and tested with a Quectel M10 modem.

  /// Tries to push the battery status and two sensor values to thingspeak.
  /// Returns true if it was successfull (HTTP 200 and nonzero answer from
  /// thingspeak), fals if something went wrong (even after wainting long and
  /// trying some things a couple of times).
  bool tryPushToThingSpeak(float bat, int sensor1, int sensor2);

  /// Sends a command and waits for an expected answer by the modem as long as
  /// the timeout did not elapse. Returns true, if the answer was actually
  /// returned in time, false it the timeout elapsed or the answer was a
  /// different one.
  bool sendCommand(char* command, char* expectedAnswer,
                      unsigned int timeout);

  /// To be called repeatedly in the main loop in order to establish a bridge
  /// between the main Serial interface and the serial interface of the modem.
  void modemSerialBridgeLoop();

#endif
