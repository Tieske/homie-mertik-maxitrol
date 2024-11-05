// Board selected: "NodeMCU 1.0 (ESP-12E Module)"
// Hardware: AZ-Delivery NodeMCU LUA Amica V2
// CPU: ESP8266MOD 12-F

/*
Mertik Maxitrol controller
==========================

ESP8266 based device to control a gas-fired fire-place that is equiped with a
Mertik Maxitrol remote control. This is based on a GV60 type, but most are
similar. Check the documentation for the connections.

The device has a pilot flame. Hence has 3 "states":
1. off (pilot is off)
2. pilot (pilot is on, but flames are off
3. on, pilot is on, gas is flowing and fireplace is burning

Controlling the device:
-----------------------
Interface: 4 pins
 - common
 - input 1
 - input 2
 - input 3

Commands:
 - ignite:  close input 1+3 to common for 1 second
 - higher:  close contact 1 to common (12 seconds, to move from 'pilot' to 'highest')
 - lower:   close contact 3 to common (12 seconds, to move from 'highest' to 'pilot')
 - off:     close input 1+2+3 to common for 1 second
 If available:
 - enable 2nd burner:    close input 1+2 to common for 1 second
 - shutdown 2nd burner:  close input 2+3 to common for 1 second

Reading pilot status:
---------------------
Using a MAX31855 thermocouple reader (Adafruit), with software SPI.

Afaik the fireplace doesn't use a thermocouple, but a thermopile (which are multiple
thermocouples that together provide enough power to keep a safety valve open).

Based on reported temperature the pilot is considered to be on or off.

*/


// ----------------------------------------------------------------------------------
//   Initialization and globals
// ----------------------------------------------------------------------------------


#include <SPI.h>
//#include <WiFi.h>         // ESP32
#include <ESP8266WiFi.h>  // ESP8266
#include <PubSubClient.h>
#include <Adafruit_MAX31855.h>
#include "homie-mertik-maxitrol.h"        // Check this file for setup and secrets !!!!!


// Error flags
#define THERMOCOUPLE_FAULT_OPEN         (0x01)     // TC open circuit fault
#define THERMOCOUPLE_FAULT_SHORT_GND    (0x02)     // TC short to GND fault
#define THERMOCOUPLE_FAULT_SHORT_VCC    (0x04)     // TC short to VCC fault
#define THERMOCOUPLE_FAULT_INIT_FAILED  (0x08)     // TC SPI initialization failed
#define THERMOCOUPLE_FAULT_RANGE        (0x10)     // TC reading out of range
unsigned long errorState = 0;


#define CMD_IDLE                  0  // no command is in progress
#define CMD_EXTINGUISH            1  // extinguish command was set, and waiting for delay to pass
#define CMD_EXTINGUISH_DELAY      2  // extinguish command was finished, now waiting for motor/valve to return to off-position, and thermo-couple to report off
#define CMD_IGNITE                4  // ignite command was set, and waiting for command delay to pass
#define CMD_IGNITE_DELAY          8  // ignite command was finished, now waiting for start procedure to finish
#define CMD_HIGHER               16  // Valve is running to open currently
#define CMD_LOWER                32  // Valve is running to close currently
unsigned int currentCmdState = CMD_IDLE;  // Global to hold current state of the commands
unsigned long cmdStartTime = 0;           // Global to store the start time of commands currently set


#define NEXT_NONE                 0  // no new command; if pilot is on, we balance towards targetPosition
#define NEXT_EXTINGUISH           1  // extinguish, turn it off
#define NEXT_IGNITE               2  // ignite, turn it on
unsigned int nextCmd = NEXT_NONE;    // Global to hold the next command after current one finishes


unsigned long currentPosition = 0;        // position where we currently are, in millis between closed and fully open
unsigned long targetPositionPerc = 0;     // target position to move to (in %).
unsigned long targetPositionMillis = 0;   // target position to move to (in millis).


// Homie and network
WiFiClient espClient;
PubSubClient client(espClient);
String deviceId = TIESKE_DEVICE_ID;


// ----------------------------------------------------------------------------------
//   Logging
// ----------------------------------------------------------------------------------


// Function to set up logging
void setupLogging() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("\nLogging initialized");
}


// Function to write log messages to the serial console
void logMessage(const char *format, ...) {
  char logBuffer[128]; // Adjust the buffer size as needed
  va_list args;
  va_start(args, format);
  vsnprintf(logBuffer, sizeof(logBuffer), format, args); // Format the log message
  va_end(args);

  Serial.println(logBuffer);
}


// ----------------------------------------------------------------------------------
//   WiFi connection
// ----------------------------------------------------------------------------------


void wifiSetup()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(TIESKE_WIFI_SSID, TIESKE_WIFI_PASSWORD);
}


const char* getWiFiStatusString(int status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "SCAN_COMPLETED";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN_STATUS";
  }
}


// DO NOT call this directly, use checkMQTT() instead.
// check the wifi connection and reconnects if needed.
// returns the current status of the wifi connection.
wl_status_t wifiCheck() {
  static unsigned long lastWifiCheck = 0;
  static wl_status_t lastStatus = WL_IDLE_STATUS;

  unsigned long now = millis();
  wl_status_t currentStatus = WiFi.status();

  if (currentStatus != lastStatus) {
    logMessage("WiFi status changed from %s to %s", getWiFiStatusString(lastStatus), getWiFiStatusString(currentStatus));
    if (currentStatus == WL_CONNECTED) {
      logMessage("WiFi connected. IP address: %s", WiFi.localIP().toString().c_str());
    }
    lastStatus = currentStatus;
    return currentStatus;
  }

  // Connected is ok, IDLE means it is in between retries
  if (currentStatus == WL_CONNECTED || currentStatus == WL_IDLE_STATUS) {
    lastStatus = currentStatus;
    lastWifiCheck = now;
    return currentStatus;  // nothing to do here
  }

  if (now - lastWifiCheck < 5000) {
    return lastStatus;  // not time yet
  }
  lastWifiCheck = now;

  wifiSetup(); // retry connecting
  return currentStatus;
}


// ----------------------------------------------------------------------------------
//   Current status on/off, from thermocouple
// ----------------------------------------------------------------------------------


Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);  // initialize the thermocouple object
bool _pilotOnStatus = false;                           // the current status of the pilot flame


// Returns a boolean indicating if the pilot flame is currently on or not.
bool isPilotOn() {
  return _pilotOnStatus;
}


// Initialiation function for the thermocouple
void setupThermocouple() {
  delay(500);                           // wait for MAX chip to stabilize at startup
  if (!thermocouple.begin()) {
    logMessage("ERROR: Initializing thermocouple failed!");
    errorState = errorState | THERMOCOUPLE_FAULT_INIT_FAILED;
    return;
  }

  logMessage("Thermocouple initialized");
  errorState = errorState & ~THERMOCOUPLE_FAULT_INIT_FAILED;
  // read thermo couple, check, wait and check again, to handle time roll-overs
  checkPilotStatus();
  delay(THERMOCOUPLE_INTERVAL + 50);
  checkPilotStatus();
}


// Function to check the status of the pilot flame.
// This function reads the thermocouple and checks if the pilot flame is on or off.
// Reads every THERMOCOUPLE_INTERVAL milliseconds, call this function in the loop.
// Upon success it will set the _pilotOnStatus variable to true.
// Upon failure it will set the errorState variable to flag any errors.
void checkPilotStatus() {
  static unsigned long lastCheck = 0;
  static unsigned long consecutiveReadErrors = 0;
  static unsigned long consecutiveRangeErrors = 0;

  // if initialization failed, skip the checks
  if (errorState & THERMOCOUPLE_FAULT_INIT_FAILED) {
    return;  // initialization failed, nothing to do here
  }

  // check if it's time to check the thermocouple
  unsigned long elapsedSince = millis() - lastCheck; // Using subtraction handles roll-over automatically
  if (elapsedSince < THERMOCOUPLE_INTERVAL) {
    return;  // not time yet
  }
  lastCheck = millis();

  // read the thermocouple
  double c = thermocouple.readCelsius();
  if (isnan(c)) {
    consecutiveReadErrors++; // increase the error counter

    // Handle error only if more than the threshold
    if (consecutiveReadErrors >= THERMOCOUPLE_MAX_ERROR) {
      uint8_t e = thermocouple.readError();
      // set error states accordingly
      if (e & MAX31855_FAULT_OPEN) {
        logMessage("FAULT: Thermocouple is open circuit.");
        errorState = errorState | THERMOCOUPLE_FAULT_OPEN;
      } else {
        errorState = errorState & ~THERMOCOUPLE_FAULT_OPEN;
      }
      if (e & MAX31855_FAULT_SHORT_GND) {
        logMessage("FAULT: Thermocouple is short-circuited to GND.");
        errorState = errorState | THERMOCOUPLE_FAULT_SHORT_GND;
      } else {
        errorState = errorState & ~THERMOCOUPLE_FAULT_SHORT_GND;
      }
      if (e & MAX31855_FAULT_SHORT_VCC) {
        logMessage("FAULT: Thermocouple is short-circuited to VCC.");
        errorState = errorState | THERMOCOUPLE_FAULT_SHORT_VCC;
      } else {
        errorState = errorState & ~THERMOCOUPLE_FAULT_SHORT_VCC;
      }
    }
    return;
  }

  // no read errors, clear the read error flags
  consecutiveReadErrors = 0;  // reset the error counter
  errorState = errorState & ~THERMOCOUPLE_FAULT_OPEN;
  errorState = errorState & ~THERMOCOUPLE_FAULT_SHORT_GND;
  errorState = errorState & ~THERMOCOUPLE_FAULT_SHORT_VCC;

  // check if the temperature is within the valid range
  if (c < THERMOCOUPLE_MIN_TEMP || c > THERMOCOUPLE_MAX_TEMP) {
    consecutiveRangeErrors++; // increase the error counter

    // Handle error only if more than the threshold
    if (consecutiveRangeErrors >= THERMOCOUPLE_MAX_ERROR) {
      logMessage("WARN: Thermocouple reading out of range: %f", c);
      errorState = errorState | THERMOCOUPLE_FAULT_RANGE;
    }
    return;
  }

  // no range error, clear the range error flag
  consecutiveRangeErrors = 0;  // reset the error counter
  errorState = errorState & ~THERMOCOUPLE_FAULT_RANGE;

  // check if the pilot is on or off based on the reported temperature
  bool newState = (c > THERMOCOUPLE_THRESHOLD);
  if (newState != _pilotOnStatus) {
    logMessage("Pilot flame switched to %s", (newState ? "ON" : "OFF"));
    if (nextCmd == NEXT_NONE) {
      // if we don't have a "next" command, we update the "value/$target" value
      // the switch was initiated by the remote-control and not through Homie
      writeTopic("homie/5/" + deviceId + "/pilot/value/$target", (newState ? "true" : "false"), true);
    }
    // set the "value" topic
    writeTopic("homie/5/" + deviceId + "/pilot/value", (newState ? "true" : "false"), true);
    // update the "status" property, but not if we're in the middle of a delay
    // inside either of the 2 delays, the status gets updated at the end of the delay
    if (currentCmdState != CMD_IGNITE_DELAY && currentCmdState != CMD_EXTINGUISH_DELAY) {
      writeTopic("homie/5/" + deviceId + "/pilot/status", (newState ? "on" : "off"), true);
    }
    _pilotOnStatus = newState;
  }
}


// ----------------------------------------------------------------------------------
//   Commands
// ----------------------------------------------------------------------------------


// IO pins setup for controlling the 3 relais
void setupRelais() {
  pinMode(RELAIS1, OUTPUT);
  logMessage("Relais for Input 1 initialized on pin %d", RELAIS1);
  pinMode(RELAIS2, OUTPUT);
  logMessage("Relais for Input 2 initialized on pin %d", RELAIS2);
  pinMode(RELAIS3, OUTPUT);
  logMessage("Relais for Input 3 initialized on pin %d", RELAIS3);

  endCommand();  // open all relais
}


// Returns the elapsed milliseconds since the last command was set
unsigned long getElapsedMillis() {
  return millis() - cmdStartTime;            // Using subtraction handles roll-over automatically
}


// Reset the command start time to the current time.
// This function will update the current position, if the existing command was to move the valve (open or close).
void resetCmdStartTime() {
  unsigned long msecs = millis();
  unsigned long elapsed = msecs - cmdStartTime;

  if (currentCmdState == CMD_HIGHER) {
    updateCurrentPosition(elapsed);          // add elapsed time
  }
  if (currentCmdState == CMD_LOWER) {
    updateCurrentPosition(-1 * elapsed);     // subtract elapsed time
  }

  cmdStartTime = msecs;
}


// DO NOT call this directly, use resetCmdStartTime() instead.
// update the curent position based on the delta provided.
// Will ensure the position stays between 0 and POSITION_RANGE_MAX.
void updateCurrentPosition(long delta) {
  static unsigned long lastTimeSet = 0; // ensure on first call we will set the value
  static int lastPerc = -1;             // ensure there is a change on first call

  if (delta < 0) {
    // subtracting, we were running the valve closed
    if ((-1 * delta) > currentPosition) {
      currentPosition = 0;
    } else {
      currentPosition = currentPosition + delta;
    }

  } else {
    // adding, we were running the valve open
    currentPosition = currentPosition + delta;
    if (currentPosition > POSITION_RANGE_MAX) {
      currentPosition = POSITION_RANGE_MAX;
    }
  }

  unsigned long msec = millis();
  if (msec - lastTimeSet > 1000) {
    lastTimeSet = msec;
    // 1 sec passed since last update, so publish a new value
    // calculate the new value in % and publish it
    int newPerc;
    if (currentPosition < POSITION_USER_MIN - POSITION_PRECISION/2) {
      newPerc = 0;

    } else if (currentPosition < POSITION_USER_MIN) {
      newPerc = 1;

    } else if (currentPosition >= POSITION_USER_MAX) {
      newPerc = 100;

    } else {
      newPerc = map(currentPosition,
          POSITION_USER_MIN, POSITION_USER_MAX,
          1, 100
      );
    };

    if (lastPerc != newPerc) {
      writeTopic("homie/5/" + deviceId + "/burner/value", String(newPerc), true);
      logMessage("burner new value: %d%%", newPerc);
      lastPerc = newPerc;
    }
  }
}


// Ends the current command, moves to IDLE state
void endCommand() {
  // open relais 1 + 2 + 3
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
  digitalWrite(RELAIS3, HIGH);
  unsigned long elapsed = getElapsedMillis();
  resetCmdStartTime();
  currentCmdState = CMD_IDLE;
  logMessage("EndCommand set after %d", elapsed);
}


// Starts the IGNITE command.
void startIgniteCommand() {
  // close relais 1 + 3
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, HIGH);
  digitalWrite(RELAIS3, LOW);
  unsigned long elapsed = getElapsedMillis();
  resetCmdStartTime();
  currentCmdState = CMD_IGNITE;
  writeTopic("homie/5/" + deviceId + "/pilot/status", "igniting", true);
  logMessage("IgniteCommand set after %d", elapsed);
}


// Starts the EXTINGUISH command.
void startExtinguishCommand() {
  // close relais 1 + 2 + 3
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, LOW);
  digitalWrite(RELAIS3, LOW);
  unsigned long elapsed = getElapsedMillis();
  resetCmdStartTime();
  currentCmdState = CMD_EXTINGUISH;
  writeTopic("homie/5/" + deviceId + "/pilot/status", "extinguishing", true);
  logMessage("ExtinguishCommand set after %d", elapsed);
}


// Starts the HIGHER command.
void startHigherCommand() {
  // close relay 1
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, HIGH);
  digitalWrite(RELAIS3, HIGH);
  unsigned long elapsed = getElapsedMillis();
  resetCmdStartTime();
  currentCmdState = CMD_HIGHER;
  logMessage("HigherCommand set after %d", elapsed);
}


// Starts the LOWER command.
void startLowerCommand() {
  // close relay 3
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
  digitalWrite(RELAIS3, LOW);
  unsigned long elapsed = getElapsedMillis();
  resetCmdStartTime();
  currentCmdState = CMD_LOWER;
  logMessage("LowerCommand set after %d", elapsed);
}


// Sets the target position to move to, in % (0-100)
// Will calculate the targetPositionMillis between POSITION_USER_MIN and POSITION_USER_MAX.
// Exception: if target is less than 1% then it will be set to 0 to close the valve.
//
// It will not change any commands, just set the target.
void setTargetPositionPerc(int newTarget) {
  if (newTarget > 100) newTarget = 100;
  if (newTarget < 0)   newTarget = 0;

  writeTopic("homie/5/" + deviceId + "/burner/value/$target", String(newTarget), true);

  if (newTarget == 0) {
    targetPositionPerc = 0;
    targetPositionMillis = 0;
  } else {
    targetPositionPerc = newTarget;
    targetPositionMillis = map(newTarget,
        1, 100,
        max(0, POSITION_USER_MIN), min(POSITION_USER_MAX, POSITION_RANGE_MAX)
    );
  };
  logMessage("new target set: %d%%, %d ms", targetPositionPerc, targetPositionMillis);
}


// returns the current target position in 0-100%.
unsigned long getTargetPositionPerc() {
  return targetPositionPerc;
}


void checkCommandStatus() {
  // Handle current status, check elapsed time and update
  switch (currentCmdState) {

    case CMD_EXTINGUISH:
      if (getElapsedMillis() >= 1000) {
        // Extinguish command is complete, stop the command and move to the delay state
        endCommand();
        currentCmdState = CMD_EXTINGUISH_DELAY;
        logMessage("Extinguish-delay started");
      }
      return; // nothing more to do here


    case CMD_EXTINGUISH_DELAY:
      if (getElapsedMillis() < EXTINGUISH_WAIT_DELAY) {
        // waiting for the extinguish delay to pass, so we're done for now
        return;
      }
      // Extinguish command is complete, delay has passed, so we're idle now
      logMessage("Extinguish-delay finished");
      writeTopic("homie/5/" + deviceId + "/pilot/status", (isPilotOn() ? "on" : "off"), true);
      if (isPilotOn()) {
        logMessage("WARN: extinguishing seems to have failed!");
      }
      currentCmdState = CMD_IDLE;
      updateCurrentPosition(POSITION_AFTER_IGNITE - currentPosition); // pass the delta to set the absolute AFER_IGNITE position
      return; // nothing more to do here


    case CMD_IGNITE:
      if (getElapsedMillis() >= 1000) {
        // Ignite command is complete, stop the command and move to the delay state
        endCommand();
        currentCmdState = CMD_IGNITE_DELAY;
        logMessage("Ignite-delay started");
      }
      return; // nothing more to do here


    case CMD_IGNITE_DELAY:
      if (getElapsedMillis() < IGNITE_WAIT_DELAY) {
        // waiting for the ignite delay to pass, so we're done for now
        return;
      }
      // Ignite command is complete, delay has passed, so we're idle now
      logMessage("Ignite-delay finished");
      writeTopic("homie/5/" + deviceId + "/pilot/status", (isPilotOn() ? "on" : "off"), true);
      if (!isPilotOn()) {
        logMessage("WARN: igniting seems to have failed!");
      }
      currentCmdState = CMD_IDLE;
      return; // nothing more to do here


    case CMD_HIGHER:
    case CMD_LOWER:
    case CMD_IDLE:
      // So we seem to be balancing towards a specific position
      resetCmdStartTime();  // update the currentPosition

//logMessage("positions: target=%d  current=%d", targetPositionMillis, currentPosition);
      // we only balance if the pilot is on, otherwise we stay put
      if (isPilotOn()) {
        unsigned long upperBound = targetPositionMillis + POSITION_PRECISION / 2;
        unsigned long lowerBound = (targetPositionMillis > POSITION_PRECISION / 2) ?
                                   (targetPositionMillis - POSITION_PRECISION / 2) : 0;

        if (currentPosition >= lowerBound && currentPosition <= upperBound) {
          // we're on target position, so we're done
          if (currentCmdState != CMD_IDLE) {
            endCommand();
            writeTopic("homie/5/" + deviceId + "/burner/value", String(targetPositionPerc), true);
          }

        } else if (currentPosition > targetPositionMillis) {
          // we're too high, so switch to lower if not already doing so
          if (currentCmdState != CMD_LOWER) {
            endCommand();
            delay(300);
            startLowerCommand();
          }

        } else if (currentPosition < targetPositionMillis) {
          // we're too low, so switch to higher if not already doing so
          if (currentCmdState != CMD_HIGHER) {
            endCommand();
            delay(300);
            startHigherCommand();
          }
        }
      }
      break;
  }  // end switch (currentCmdState)


  // By now we are either balancing (HIGHER/LOWER) or we're IDLE
  // so check if there is another command to run
  switch (nextCmd) {

    case NEXT_IGNITE:
      nextCmd = NEXT_NONE;
      if (!isPilotOn()) startIgniteCommand();  // only ignite if we're not already on
      break;

    case NEXT_EXTINGUISH:
      nextCmd = NEXT_NONE;
      startExtinguishCommand();  // extinguish we always run, even if pilot is off, for safety.
      break;

  } // end switch (nextCmd)
}


// ----------------------------------------------------------------------------------
//   Reporting stuff to the user
// ----------------------------------------------------------------------------------


// Function to report the error state to the user using Homie5 Alerts
// This function should be called in the loop, to report any errors to the user.
// It will only report the changes in errorState, unless force = true, then all will be handled.
void reportErrorState(bool force) {
  static unsigned long lastReportedErrorState = 0;
  if (errorState == lastReportedErrorState) {
    // nothing changed
    if (!force) {
      // we're not forced either
      return;  // so nothing to report
    }
  }

  // for each of the THERMOCOUPLE_FAULT_xx flags, we report an error
  if (errorState & THERMOCOUPLE_FAULT_OPEN) {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-open", "Thermocouple connector is open", true);
  } else {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-open", "", true);
  }

  if (errorState & THERMOCOUPLE_FAULT_SHORT_GND) {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-short-gnd", "Thermocouple is shorted to GND", true);
  } else {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-short-gnd", "", true);
  }

  if (errorState & THERMOCOUPLE_FAULT_SHORT_VCC) {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-short-vcc", "Thermocouple is shorted to VCC", true);
  } else {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-short-vcc", "", true);
  }

  if (errorState & THERMOCOUPLE_FAULT_INIT_FAILED) {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-init-failed", "Thermocouple initialization failed", true);
  } else {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-init-failed", "", true);
  }

  if (errorState & THERMOCOUPLE_FAULT_RANGE) {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-range", "Thermocouple reading out of range", true);
  } else {
    writeTopic("homie/5/" + deviceId + "/$alert/tc-range", "", true);
  }

  lastReportedErrorState = errorState;
}


// Get status of the pilot-flame according to the command cycle.
// Returns a string with the current user-facing status.
// Possible values are: "off", "igniting", "on", "extinguishing"
String getPilotCmdStatus() {
  switch (currentCmdState) {
    case CMD_EXTINGUISH:
    case CMD_EXTINGUISH_DELAY:
      return "extinguishing";

    case CMD_IGNITE:
    case CMD_IGNITE_DELAY:
      return "igniting";
  }

  if (!isPilotOn()) {
    return "off";
  }

  return "on";
}


// Returns the status of the pilot-flame according to the thermocouple.
// Returns a string with the current user-facing status.
// Possible values are: "false", "true"
String getPilotStatus() {
  return isPilotOn() ? "true" : "false";
}


// ----------------------------------------------------------------------------------
//   Homie device setup
// ----------------------------------------------------------------------------------


// Returns the nth segment of a topic string, where segments are separated by '/'.
// 0-indexed. If the index is out of bounds, an empty string is returned.
String getTopicSegment(String topic, int index) {
  const char* topicCStr = topic.c_str();
  int segmentCount = 0;
  const char* startPtr = topicCStr;

  // Traverse through the topic string
  for (const char* ptr = topicCStr; *ptr != '\0'; ++ptr) {
    if (*ptr == '/') {
      if (segmentCount == index) {
        return String(startPtr).substring(0, ptr - startPtr);
      }
      // Move to the start of the next segment
      segmentCount++;
      startPtr = ptr + 1;
    }
  }

  // Handle the last segment (if no trailing slash)
  if (segmentCount == index) {
    return String(startPtr);
  }

  // Index out of bounds, return an empty string
  return "";
}


// Check if a received payload is a float
bool isFloat(const String& value) {
  char* endPtr;
  float result = strtof(value.c_str(), &endPtr);
  return *endPtr == '\0';
}


// Callback handling subscribed topic values received
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to a null-terminated string
  char payloadStr[length + 1];  // +1 for the null terminator
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';  // Null-terminate the string

  logMessage("received MQTT data on: %s='%s'", topic, payloadStr);
  if (length > 20) { // we expect: true, false, or number 0:100:1
    logMessage("received MQTT payload is too long: %d", length);
    return;
  }

  payload[length] = '\0'; // Null-terminate the payload
  String value = String((char*)payload);
  // segements 0,1,2 are: "homie/5/" + deviceId
  String nodeId = getTopicSegment(String(topic), 3);
  String propId = getTopicSegment(String(topic), 4);

  if (nodeId == "pilot") {

    // select by PropertyId
    if (propId == "value") {    // handle topic: ../pilot/value/set
      if (value != "true" && value != "false") {
        logMessage("received bad value on '%s/%s/set': '%s'", nodeId, propId, value);
        return;
      }

      // post value on $target attribute
      writeTopic("homie/5/" + deviceId + "/pilot/value/$target", value, true);

      // either ignite or extinguish based on the value
      if (value == "true") {
        nextCmd = NEXT_IGNITE;
      } else {
        nextCmd = NEXT_EXTINGUISH;
      }

    } else {
      logMessage("received value on unknown property: '%s/%s/set'", nodeId, propId);
    }


  } else if (nodeId == "burner") {
    // select by PropertyId
    if (propId == "value") {      // handle topic: ../level/value/set
      if (!isFloat(value)) {
        logMessage("received bad value on '%s/%s/set': '%s'", nodeId, propId, value);
        return;
      }

      // convert value to a float and round it to the nearest integer
      int newTarget = round(value.toFloat());
      if (newTarget < 0 || newTarget > 100) {
        logMessage("received bad value on '%s/%s/set': '%s'", nodeId, propId, value);
        return;
      }

      // we're accepting the value, post on $target attribute, and update internal value
      writeTopic("homie/5/" + deviceId + "/burner/value/$target", value, true);
      setTargetPositionPerc(newTarget);

    } else {
      logMessage("received value on unknown property: '%s/%s/set'", nodeId, propId);
    }


  } else {
    logMessage("received value on unknown node: '%s/%s/set'", nodeId, propId);
  }
}


// write a value to a topic, but only if we're connected
void writeTopic(String topic, String value, bool retain) {
  if (client.connected()) {
    client.publish(topic.c_str(), value.c_str(), retain);
  }
}

// Function to send large MQTT payload in chunks using beginPublish(), write(), and endPublish()
// Returns true if successful, false if any error occurs
bool writeBigTopic(String topic, String payload, bool retain) {
  size_t totalLength = payload.length();

  if (!client.beginPublish(topic.c_str(), totalLength, retain)) {
    logMessage("Error: Failed to begin publishing to topic %s", topic.c_str());
    return false;
  }

  for (size_t i = 0; i < totalLength; i += MQTT_MAX_PACKET_SIZE) {
    size_t remainingLength = totalLength - i;
    size_t lengthToSend = min((size_t)MQTT_MAX_PACKET_SIZE, remainingLength);

    if (client.write((const uint8_t*)payload.c_str() + i, lengthToSend) != lengthToSend) {
      logMessage("Error: Failed to send chunk to topic %s", topic.c_str());
      client.endPublish();  // Ensure publishing is properly terminated
      return false;
    }
  }

  if (!client.endPublish()) {
    logMessage("Error: Failed to end publishing to topic %s", topic.c_str());
    return false;
  }
  return true;
}


// Write the Homie description to the MQTT topics and subscribe to topics
void publishHomieDeviceDescription() {
  // mark device for (re)configuration
  writeTopic("homie/5/" + deviceId + "/$state", "init", true);

  /* send device description, we're sending the minified and escaped version, the full version is:
  {
    "name": "Smart Fireplace",
    "homie": "5.0",
    "version": 1,
    "nodes": {
      "pilot": {
        "properties": {
          "value": {
            "datatype": "boolean",
            "format": "off,on",
            "settable": true,
            "retained": true
          },
          "status": {
            "datatype": "enum",
            "format": "off,igniting,on,extinguishing",
            "settable": false,
            "retained": true
          }
        }
      },
      "burner": {
        "properties": {
          "value": {
            "datatype": "float",
            "format": "0:100:1",
            "settable": true,
            "retained": true,
            "unit": "%"
          }
        }
      }
    }
  }
  */

  writeBigTopic("homie/5/" + deviceId + "/$description",
    "{\"name\":\"Smart Fireplace\",\"homie\":\"5.0\",\"version\":1,\"nodes\":{\"pilot\":{\"properties\":{\"value\":{\"datatype\":\"boolean\",\"format\":\"off,on\",\"settable\":true,\"retained\":true},\"status\":{\"datatype\":\"enum\",\"format\":\"off,igniting,on,extinguishing\",\"settable\":false,\"retained\":true}}},\"burner\":{\"properties\":{\"value\":{\"datatype\":\"float\",\"format\":\"0:100:1\",\"settable\":true,\"retained\":true,\"unit\":\"%\"}}}}}",
    true);

  // publish current state
  writeTopic("homie/5/" + deviceId + "/pilot/value/$target", getPilotStatus(), true);
  writeTopic("homie/5/" + deviceId + "/pilot/value", getPilotStatus(), true);
  writeTopic("homie/5/" + deviceId + "/pilot/status", getPilotCmdStatus(), true);

  String targetPos = String(getTargetPositionPerc());
  writeTopic("homie/5/" + deviceId + "/burner/value/$target", targetPos, true);
  writeTopic("homie/5/" + deviceId + "/burner/value", targetPos, true);

  logMessage("Published homie 5 device '%s'", deviceId.c_str());

  // subscribe to setter topics
  client.subscribe(("homie/5/" + deviceId + "/+/+/set").c_str(), 1);  // QoS level 1; at least once
  logMessage("Subscribed to homie 5 device '%s' set topics", deviceId.c_str());

  // set/clear alerts
  reportErrorState(true);

  // Mark device as ready
  writeTopic("homie/5/" + deviceId + "/$state", "ready", true);
  logMessage("Marked homie 5 device '%s' as 'ready'", deviceId.c_str());
}


// ----------------------------------------------------------------------------------
//   MQTT setup
// ----------------------------------------------------------------------------------


void setupMQTT() {
  client.setServer(TIESKE_MQTT_SERVER, TIESKE_MQTT_PORT);
  client.setCallback(callback);
}


// DO NOT call this directly, use checkMQTT() instead.
// Reconncts to MQTT broker, using the global settings.
// Returns true if the connection was successful, false otherwise.
bool reconnectMQTT() {
  // only try once every 10 sconds
  static unsigned long lastReconnectAttempt = 0;
  if (millis() - lastReconnectAttempt < 10000) {
    return false; // Not time to retry yet
  }
  lastReconnectAttempt = millis();

  if (!client.connect(
        TIESKE_DEVICE_ID,
        TIESKE_MQTT_USER,
        TIESKE_MQTT_PASSWORD,
        ("homie/5/" + deviceId + "/$state").c_str(),  // LWT: topic
        1,                                            // LWT: QoS
        true,                                         // LWT: retain
        "lost",                                       // LWT: value
        false))                                       // no clean session
    {
    logMessage("Failed to connect to MQTT broker, rc=%d", client.state());
    return false;
  }

  logMessage("Connected to MQTT broker");
  publishHomieDeviceDescription();
  return true;
}


// checks the MQTT connection and reconnects if needed. If connected does
// a loop-step, so call this from the main loop.
// returns the current status of the MQTT connection.
bool checkMQTT() {
  if (client.connected()) {
    client.loop(); // do a loop-step, check messages, etc.
    return true;
  }

  // check WiFI
  if (wifiCheck() != WL_CONNECTED) {
    return false;  // no wifi, no mqtt
  }

  return reconnectMQTT();
}


// ----------------------------------------------------------------------------------
//   Main application setup and loop
// ----------------------------------------------------------------------------------


// runs once at strtup
void setup() {
  errorState = 0;  // disable all error flags
  setupLogging();
  setupRelais();
  setupThermocouple();
  wifiSetup();
  setupMQTT();   // only set up, connect is automatic in the main loop
}


// the loop function runs over and over again forever
void loop() {
  checkMQTT();              // step/connect/reconnect mqtt
  checkPilotStatus();       // read thermocouple and set status
  checkCommandStatus();     // main logic for fire-place control
  reportErrorState(false);  // report any errors to the user

  // Wait for a while, minimum POSITION_PRECISION/4 to ensure we can move the valve
  // without keeping flipping between HIGHER and LOWER.
  // Maximum wait is 100ms to remain responsive to user commands.
  delay(max(POSITION_PRECISION/4, 100));
}
