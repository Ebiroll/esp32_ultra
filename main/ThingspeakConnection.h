#ifndef THINGSPEAKCONNECTION_H
#define THINGSPEAKCONNECTION_H
#include "Debugging.h"
#include "HardwareSerial.h"
#include "Timeout.h"


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
class ThingspeakConnection {
 public:
  ThingspeakConnection(HardwareSerial& serial)
      : serial(&serial) {};

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

  /// Reads from the serial inteface and checks if the incoming bytes match an
  /// expected answer. Leading carriage returns and newlines are skipped. As
  /// soon as a byte does not match the expected answer, it skips to the first
  /// newline. Returns true if the full answer was matched, false otherwise.
  bool waitFor(char* answer, unsigned int timeout);

 private:
  /// The serial interface to be used to communicate with the modem
  HardwareSerial* serial;

  /// Waits until the TCP stack of the modem is ready. For this the state of the
  /// TCP stack is constantly checked using AT+QISTAT in 50ms intervals. If the
  /// stack is not ready before the timeout elapsed we give up and return false,
  /// otherwise true.
  bool waitForTCPStack(unsigned int timeout);

  /// Reads from the serial interface until a certain character sequence is
  /// found or the timeout elapsed. Returns true if the character sequence was
  /// found, false if the timout elapsed.
  bool readUntil(char* answer, unsigned int timeout);


  /// Reads from the serial interface until a given char c is read or the
  /// timeout elapsed. Returns true if we actually found the char, false if the
  /// timeout elapsed.
  bool readUntil(char c, unsigned int timeout);

  /// Actively (blocking) waits for a new char to be available on the serial
  /// interface. Returns true if there actually was a next char, false if we got
  /// impatient and cancelled waiting because of the timeout
  bool waitForNextChar(unsigned int timeout);
};

#endif
