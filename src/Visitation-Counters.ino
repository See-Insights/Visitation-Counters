
/*
* Project NC-State-Parks - new carrier for NC State Parks contract
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:20 November 2020
*/

/*  This is a refinement on the Boron Connected Counter firmware and incorporates new watchdog and RTC
*   capabilities as laid out in AN0023 - https://github.com/particle-iot/app-notes/tree/master/AN023-Watchdog-Timers
*   This software will work with both pressure and PIR sensor counters
*/

/* Alert Code Definitions
* 0 = Normal Operations - No Alert
// device alerts
* 10 = Battery temp too high / low to charge
* 11 = PMIC Reset required
* 12 = Initialization error (likely FRAM)
* 13 = Excessive resets
* 14 = Out of memory
* 15 = Particle disconnect or Modem Power Down Failure
// deviceOS or Firmware alerts
* 20 = Firmware update completed - deleted
* 21 = Firmware update timed out - deleted
* 22 = Firmware update failed - deleted
* 23 = Update attempt limit reached - done for the day
// Connectivity alerts
* 30 = Particle connection timed out but Cellular connection completed
* 31 = Failed to connect to Particle or cellular - skipping to next hour
* 32 = Failed to connect quickly - resetting to keep trying
// Particle cloud alerts
* 40 = Failed to get Webhook response when connected
*/

//v1 - Adapted from the Boron Connected Counter Code at release v10
//v2 - Made some significant improvements: temp dependent charging, avoiding use of "enable" sleep, better battery "context" -
//V3 - defaults to trail counters
//v4 - defaults to car counters - norm going forward - note this is only applied with a new device
//v4.02 - Added watchdog petting to connecttoparticle and got rid of srtcpy
//v4.03 - Added and out of memory reset into the main loop as recommended in AN023 above
//v5.00 - Updated and deployed to the Particle product group
//v6.00 - Update to support 24 hour operation / took out a default setting for sensor type / Added some DOXYGEN comments / Fixed sysStatus object / Added connection reporting and reset logic
//v7.00 - Fix for "white light bug".
//v8.00 - Simpler setup() and new state for connecting to particle cloud, reporting connection duration in webhook
//v9.00 - Testing some new features; 1) No ProductID!  2) bounds check on connect time, 3) Function to support seeding a daily value 4) Deleted unused "reset FRAM" function
//v9.01 - Updated .gitignore, removed lastConnectDuration unneeded tests
//v9.02 - Only a minor fix of a DOXYGEN comment
//v10.00 - Adding a publish to Google Sheets with device configuration, fix for sleepwalking and midnight wake and unit mismatch in Particle Connect Blocking
//v10.01 - Added two fields to the Google Sheets publish - minbattery and maxconnection time. Set the solar value to true - need to take out over time.
//v10.02 - Updated the Google Strings to make them more clear.  Changed to Google Sheets only on opening hour and when using "Send Now" function
//v10.03 - Fixed two minor issues - Reporting at Opening hour and "Low Power" labeling
//v10.04 - Need to put a bounds check on the connect time
//v11.00  - Fixed major bug that caused repeated publishes to Google sheets - Verion to be deployed to all devices
//v11.01 - Working out some issues with state of charge after sleep - https://community.particle.io/t/soc-returns-0-00-at-times-bsom-eval-board/60000/3
//v11.02 - Fixed bug that let device get stuck in green flashing light mode
//v11.03 - Working on way to capture connection error
//v11.04 - Adding logic to reset the PMIC if needed adding a check to make sure temp measurement is more accurate
//v11.05 - Added step to ensure graceful shutdown of celular modem
//v11.06 - Got rif of the last Serial statement and added the Serial Log Handler for montitoring
//  Particle has identified an issue where certain devices are going into a cycle of power being disconnected just after the device connects.  This is causing excessive data consumption.  Need to focus on all parts of code where power can be bounced.
//v12.00 - Based on conversation with Particle support - changed full modem reset function, update to deviceOS@2.1.0 - and will do two test legs (low risk / power cycle to reset , high risk / disable WDT) will see if this fixes issues - v12 will be the power cycle to system reset version
//v13.00 - In this version, we will disable the Watchdog timer in Setup.
//v14.00 - In this version, we will update v12 with a longer timeout in the connection process to 5 seconds.  This is needed due to recent changes in the Particle backend systems.
//v14.01 - Trying a longer timout as 5 seconds does not seem to be long enough
//v14.02 - Going to add another step to catch this issue potentially also going to take the penalty out of not connecting.
//v15.00 - Major changes - working to stop or slow down the reset loop.
//v16.00 - Moving the Particle connection function to the main loop to eliminate blocking issue.  Removed line from ResponseWait that was causing repeated session restarts
//v17.00 - Added a line in setup to fix connectedStatus.  Fixed issue with multiple sends for non-lowPowerMode devices.
// Particle back-end issue resolved - higher releases are all functional
//v18.00 - Updated Full modem Reset
//v19.00 - Recompiled for deviceOS@2.0.1 so we could update low-bandwidth devices.  Had to comment out the Cellular.isOff() in two places.  Need to add an update handler
//v20.00 - For the Wake County counters and new default going forward standard day is 6am to 10pm.  This update will force the 10pm close and this will be removed for future releases keeping the "system defaults to 10pm"
//v21.00 - Major Update - 1) Queueing only Webhooks, 2) New PublishSyncPOSIX, 3) No more "in flight" counts 4) Enforce low battery limits
//v21.01 - Fixed error that slowed device going to sleep.
//v21.02 - Removed conditional connection code used for testing and added logic to report every other hour when capacity is less than 65%
//v21.03 - Fixed but that caused multiple reports when the battery is low
//v21.04 - Found an issue that lost the current daily count
//v22.00 - Fixed an issue in the structure of the current counts srray that caused a loss of daily count data.  Changed the way that Alerts are coded: (1 - too hot or cold, 2 - timed out connection, 3 - timed out webhook)
//v23.00 - Changed behaviour to sent first puslish at open to set zero counts, removed the WITH_ACK for all but Error messages
//v24.00 - Changed the program flow so Connecting state always returns to IDLE state.  Reporting works with or without connecting.
//v25.00 - Fixed issue where not low power device looses connection to Particle cloud - also turns off cell radio after time out connecting
//v26.00 - Simplified ERROR state, got rid of FullModemReset and cleaned up little nits to shorted code.  Added Real-Time Audit feature.
//v27.00 - Updated color for testing to "Blue-Red" and streamlined the Connecting state flow.  Also, time wake sends to IDLE not CONNECTING and Failure to connect moves to LowPowerMode in Solar devices, reset clears verbose counting
//v28.00 - Recompiled for deviceOS@2.2.0 - should bring better results for long connection times
//v29.00 - Adding a new state for receiving a firmware update - this state delays napping / sleeping to receive the update
//v30.00 - Same as v29 but compiled for deviceOS@2.2.0 - Keeps the firmware update state but with less debug messaging.  One change - goes to reporting only every 2 hours at 65% and 4 hours less than 50%
//v31.00 - Added a bounds check on the lastConnectionDuration
//v32.00 - Explicitly enable updates at the new day, Battery sense in low power, Connection time logic to millis, check for Cellular off for napping and sleep, check for lost connection and add connection limits, 96 messages in POSIX queue
//v33.00 - Minor, removed battery SoC and VCell messaging to save data costs, improved reporting on update status
//v34.00 - Recompiled for deviceOS@2.0.1 for low-bandwidth devices.  Cellular.isOFf is commented out in disconnectFromParticle()
//v34.01 - Need some additional delay for Cellular Off since we don't have  - backed off v34 - back to v31
//v33.01 - Moved back to deviceOS@2.2.0 - baseline for moving forward.
//v33.03 - Minor updates - System.on collected, Solar panel current limits adjusted, messages after load defaults, String message fix
//v33.04 - Removed current limits from 33.03
//v35.00 - Fixed issue with the PublishQueuePosix that could cause lockups, Fixed DST calculation for 2021 when DST changes on November 7th, fixed issue with sleeping too fast
//v36.00 - Fix for location of queue and better handling for connection issues
//v37.00 - Got rid of DSTRULES define - USA only for now, fixed issue with lowPowerMode loosing connection,
//v37.02 - Fixed ternary in CONNECTING state and revamped alert codes
//v38.00 - v37.02 for general release
//v39.00 - Fix for Alerts webhook and reduced max connection time to 10 minutes
//v40.00 - Updates in logic - enhanced ERROR section and better commenting throughout, Added WITH_ACK to webhook pucblishes, reordered some commands in startup, added new alert codes
//v41.00 - Added a daily reset of the resetCounts, Fixed issue with Error state, took out debugging delays
//v42.00 - Fixed potential issue in Response Wait State
//v43.00 - Updated to move sync time to CONNECTING and add WITH_ACK
//v44.00 - Minor update to disconenct from Particle and added a step to power down the modem if in ERROR_STATE
//v44.10 - Update to add remote logging.  New feature for testing not production across fleet
//v45.00 - Update to remote logging level (moved to ALL) reduced log.info statements throughout to save bandwidth. Added a second to the Sleep time to reduce round tripping (ILDE-SLEEP)
//v46.10 - Fixing minor bugs - sleep time +1 second, placement of setup alert codes, serVerboseMode()
//v46.20 - Removed remote logging
//v46.30 - Added support for Verizon / 3rd Party SIMS, Added a daily update to RSSI and Location to Ubidots
//v46.40 - Added logic to handle an error where Particle would not disconnect or the cellular modem would not shut down
//v46.50 - Added support for user button while connected to trigger GPS and Cell Status event
//v46.60 - New approach for connecting - connection decision moved to reporting.  Using resets to speed connection.  Moving reset logic to ERROR_STATE
//v46.61 - Added logic in check values for the connection limit and a "backoff" for when not in low power mode
//v46.62 - Fixed formatting of time_t in the disconnectFromParticle function
//v46.63 - Fixed calculation of connection duration, took out 30 second delay before resetting
//v46.64 - Removed unit test header file, fixed login on back off flag in connecting state, changed consequence for cellular/no particle - this version will be deployed to the fleet. 
//v46.65 - Moving to a new model to report to Google Firestore - Raising excessive resets threshold
//v46.66 - Moving to deviceOS@4.0.0 - removing firmware update code / complexity - will increase integer once tested, increase reset tolerance, getting rid of Fuel and name helper
        //  - getting rid of the complexity of back-offs
//v47.00 - Recompiled for deviceOS@4.0.1 to fix connectivity issues.
//v48.00 - Updated to address issue with Error 30 assignment by mistake
//v49.00 - Fixed issue that could cause device to get stuck if not connected
//v50.00 - Fixed issue where new devices would have a connectiion time of 0
//v51.00 - Added an upper bound check on the connection time limit, set connection time limit in set defaults, modified verbose mode (publish / time limit)
//v52.00 - Added more commentary on connect and made it harder to reset the PMIC based on observations of the fleet.

// Particle Product definitions
PRODUCT_VERSION(52);
char currentPointRelease[6] ="52.00";

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Version of the FRAM memory map
    systemStatusAddr      = 0x01,                   // Where we store the system status data structure
    currentCountsAddr     = 0x50                    // Where we store the current counts data structure
  };
};

const int FRAMversionNumber = 3;                    // Increment this number each time the memory map is changed

struct currentCounts_structure {                    // currently 10 bytes long
  int hourlyCount;                                  // In period hourly count
  int currentConnectionLimit;                       // Here we will store the connection limit (180, 300 or 420 seconds)  - 180 indicates a new connection cycle
  int dailyCount;                                   // In period daily count
  unsigned long lastCountTime;                      // When did we record our last count
  int temperature;                                  // Current Temperature inside the enclosure
  int alerts;                                       // What is the current alert value - see secret decoder ring at top of comments
  int maxMinValue;                                  // Highest count in one minute in the current period
  uint16_t maxConnectTime = 0;                      // Longest connect time for the hour
  int minBatteryLevel = 100;                        // Lowest Battery level for the day - not sure this is needed
  uint8_t updateAttempts = 0;                       // Number of attempted updates each day
  bool backOff;                                     // If we are not in lowPowerMode but are struggling to connect
} current;


// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                  // Pinout Documentation File
#include "AB1805_RK.h"                              // Watchdog and Real Time Clock - https://github.com/rickkas7/AB1805_RK
#include "MB85RC256V-FRAM-RK.h"                     // Rickkas Particle based FRAM Library
#include "PublishQueuePosixRK.h"                    // Allows for queuing of messages - https://github.com/rickkas7/PublishQueuePosixRK
#include "DiagnosticsHelperRK.h"                    // For sending locationa dn device vitals information

// Start code block for remote logging
// Libraries with helper functions
#include "time_zone_fn.h"
#include "sys_status.h"
#include "particle_fn.h"

struct systemStatus_structure sysStatus;

// If updating, we need to delay sleep in order to give the download time to come through before sleeping
const std::chrono::milliseconds firmwareUpdateMaxTime = 10min; // Set at least 5 minutes

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
SystemSleepConfiguration config;                    // Initialize new Sleep 2.0 Api
MB85RC64 fram(Wire, 0);                             // Rickkas' FRAM library
AB1805 ab1805(Wire);                                // Rickkas' RTC / Watchdog library
FuelGauge fuelGauge;                                // Needed to address issue with updates in low battery state

// For monitoring / debugging, you have some options on the next few lines
SerialLogHandler logHandler(LOG_LEVEL_TRACE);
//SerialLogHandler logHandler(LOG_LEVEL_ALL);         // All the loggings 
// SerialLogHandler logHandler(LOG_LEVEL_INFO);     // Easier to see the program flow
// Serial1LogHandler logHandler1(57600);            // This line is for when we are using the OTII ARC for power analysis

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, FIRMWARE_UPDATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;

// Battery Conect variables
// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

// Pin Constants - Boron Carrier Board v1.x
const int tmp36Pin =      A4;                       // Simple Analog temperature sensor - on the carrier board - inside the enclosure
const int wakeUpPin =     D8;                       // This is the Particle Electron WKP pin
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D4;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin =        SCK;                      // Pressure Sensor inerrupt pin
const int disableModule = MOSI;                     // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      MISO;                     // Allows us to control the indicator LED on the sensor board

// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // Sets a reporting frequency of 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 30000;            // How long will we wait for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
char currentOffsetStr[10];                          // What is our offset from UTC
unsigned long lastReportedTime = 0;                 // Need to keep this separate from time so we know when to report
unsigned long connectionStartTime;                  // Timestamp to keep track of how long it takes to connect

// Program Variables
bool dataInFlight = false;                          // Tracks if we have sent data but not yet received a response
bool firmwareUpdateInProgress = false;              // Helps us track if a firmware update is in progress
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
char batteryContextStr[16];                         // Tracks the battery context
char lowPowerModeStr[16];                           // In low power mode?
char simCardStr[12];
char openTimeStr[8]="NA";                           // Park Open Time
char closeTimeStr[8]="NA";                          // Park close Time
char sensorTypeConfigStr[16];
bool systemStatusWriteNeeded = false;               // Keep track of when we need to write - system object
bool currentCountsWriteNeeded = false;              // Current counts object write needed

// System Health Variables
int outOfMemory = -1;                               // From reference code provided in AN0023 (see above)

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Interrupt Variables
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
volatile bool userSwitchDetect = false;              // Flag for a user switch press while in connected state

Timer countSignalTimer(1000, countSignalTimerISR, true);      // This is how we will ensure the BlueLED stays on long enough for folks to see it.

void setup()                                        // Note: Disconnected Setup()
{
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output

  // Sensor Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Disables the module when pulled high
  pinMode(ledPower,OUTPUT);                         // Turn on the lights

  digitalWrite(blueLED,HIGH);                       // Turn on the led so we can see how long the Setup() takes

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event
  System.on(out_of_memory, outOfMemoryHandler);     // Enabling an out of memory handler is a good safety tip. If we run out of memory a System.reset() is done.

  Particle.variable("HourlyCount", current.hourlyCount);                // Define my Particle variables
  Particle.variable("DailyCount", current.dailyCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", sysStatus.resetCount);
  Particle.variable("Temperature",current.temperature);
  Particle.variable("Release",currentPointRelease);
  Particle.variable("stateOfChg", sysStatus.stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerModeStr);
  Particle.variable("OpenTime", openTimeStr);
  Particle.variable("CloseTime",closeTimeStr);
  Particle.variable("Alerts",current.alerts);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("BatteryContext",batteryContextMessage);
  Particle.variable("SensorStatus",sensorTypeConfigStr);
  Particle.variable("SIM-Status", simCardStr);

  Particle.function("setDailyCount", setDailyCount);                  // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-SensorType",setSensorType);
  Particle.function("setVerizonSIM",setVerizonSIM);

  // Particle and System Set up next
  Particle.setDisconnectOptions(CloudDisconnectOptions().graceful(true).timeout(5s));  // Don't disconnect abruptly

  // Watchdog Timer and Real Time Clock Initialization
  ab1805.withFOUT(D8).setup();                                         // The carrier board has D8 connected to FOUT for wake interrupts
  ab1805.setWDT(AB1805::WATCHDOG_MAX_SECONDS);                         // Enable watchdog

  // Next we will load FRAM and check or reset variables to their correct values
  fram.begin();                                                        // Initialize the FRAM module
  byte tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);                            // Load the FRAM memory map version into a variable for comparison
  if (tempVersion != FRAMversionNumber) {                              // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                      // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr, FRAMversionNumber);                    // Put the right value in
    fram.get(FRAM::versionAddr, tempVersion);                          // See if this worked
    if (tempVersion != FRAMversionNumber) {
      state = ERROR_STATE;                                             // Device will not work without FRAM will need to reset
      resetTimeStamp = millis();                                       // Likely close to zero but, for form's sake
      current.alerts = 12;                                             // FRAM is messed up so can't store but will be read in ERROR state
    }
    else loadSystemDefaults();                                         // Out of the box, we need the device to be awake and connected
  }
  else {
    fram.get(FRAM::systemStatusAddr,sysStatus);                        // Loads the System Status array from FRAM
    fram.get(FRAM::currentCountsAddr,current);                         // Loead the current values array from FRAM
  }

  // Now that the system object is loaded - let's make sure the values make sense
  checkSystemValues();                                                // Make sure System values are all in valid range

  if (sysStatus.verizonSIM) Particle.keepAlive(60);                   // If we have a Verizon SIM, we need to issue this keep alive command

  // Take note if we are restarting due to a pin reset - either by the user or the watchdog - could be sign of trouble
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER) { // Check to see if we are starting from a pin reset or a reset in the sketch
    sysStatus.resetCount++;
    if (sysStatus.resetCount > 8) current.alerts = 13;                 // Excessive resets - increased to 8 due to back off resets
  }

  // Publish Queue Posix is used exclusively for sending webhooks and update alerts in order to conserve RAM and reduce writes / wear
  PublishQueuePosix::instance().setup();                               // Start the Publish Queie

  // Next we set the timezone and check is we are in daylight savings time
  Time.setDSTOffset(sysStatus.dstOffset);                              // Set the value from FRAM if in limits
  isDSTusa() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here
  Time.zone(sysStatus.timezone);                                       // Set the Time Zone for our device
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);   // Load the offset string

  // Next - check to make sure we are not in an endless update loop
  if (current.updateAttempts >= 3) {
    char data[64];
    System.disableUpdates();                                           // We will only try to update three times in a day
    current.alerts = 23;                                                // Set an alert that we have maxed out our updates for the day
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }

  // If  the user is holding the user button - we will load defaults
  if (!digitalRead(userSwitch)) loadSystemDefaults();                  // Make sure the device wakes up and connects - reset to defaults and exit low power mode

  // Strings make it easier to read the system values in the console / mobile app
  makeUpStringMessages();                                              // Updated system settings - refresh the string messages

  // Make sure we have the right power settings
  setPowerConfig(true);                                                    // Executes commands that set up the Power configuration between Solar and DC-Powered

  // Done with the System Stuff - now we will focus on the current counts values
  if (current.hourlyCount) lastReportedTime = current.lastCountTime;
  else lastReportedTime = Time.now();                                  // Initialize it to now so that reporting can begin as soon as the hour changes

  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  if (Time.day() != Time.day(current.lastCountTime)) {                 // Check to see if the device was last on in a different day
    resetEverything();                                                 // Zero the counts for the new day
  }

  takeMeasurements();                                                  // Populates values so you can read them before the hour
  if (sysStatus.lowBatteryMode) setLowPowerMode("1");                  // If battery is low we need to go to low power state

  if ((Time.hour() >= sysStatus.openTime) && (Time.hour() < sysStatus.closeTime)) { // Park is open let's get ready for the day
    sensorControl(true);                                               // Turn on the sensor
    attachInterrupt(intPin, sensorISR, RISING);                        // Pressure Sensor interrupt from low to high
    stayAwake = stayAwakeLong;                                         // Keeps Boron awake after reboot - helps with recovery
    if (!sysStatus.lowPowerMode && !current.backOff) {
      state = CONNECTING_STATE;                                         // If we are not in low power mode, we should connect
      Log.info("Connecting because not low power and not back off");
    }
    else if (current.currentConnectionLimit != 180) {
      state = CONNECTING_STATE;                                        // We are in the middle of a connection attempt
      Log.info("Connecting with a connection limit of %i", current.currentConnectionLimit);
    } 
  }

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;               // IDLE unless otherwise from above code

  systemStatusWriteNeeded = true;                                      // Update FRAM with any changes from setup
  Log.info("Startup complete");
  digitalWrite(blueLED,LOW);                                           // Signal the end of startup
}


void loop()
{
  switch(state) {
  case IDLE_STATE:                                                     // Where we spend most time - note, the order of these conditionals is important
    if (state != oldState) publishStateTransition();
    if (sysStatus.lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;         // When in low power mode, we can nap between taps
    if (Time.hour() != Time.hour(lastReportedTime)) state = REPORTING_STATE;                                  // We want to report on the hour but not after bedtime
    if ((Time.hour() >= sysStatus.closeTime) || (Time.hour() < sysStatus.openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                               // This state is triggered once the park closes and runs until it opens - Sensor is off and interrupts disconnected
    if (state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                           // Done sensing for the day
    sensorControl(false);                                              // Turn off the sensor module for the hour
    if (current.hourlyCount) {                                         // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (Particle.connected() || !Cellular.isOff()) {
      if (!disconnectFromParticle()) {                                 // Disconnect cleanly from Particle and power down the modem
        state = ERROR_STATE;
        current.alerts = 15;
        resetTimeStamp = millis();
        break;
      }
    }    
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Head back to the idle state after we sleep
    ab1805.stopWDT();                                                  // No watchdogs interrupting our slumber
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary) + 1;  // Adding one second to reduce prospect of round tripping to IDLE
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep device continues operations from here
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) {                            // If the user woke the device we need to get up - device was sleeping so we need to reset opening hours
      setLowPowerMode("0");                                            // We are waking the device for a reason
      Log.info("Resetting opening hours");
      sysStatus.openTime = 0;                                          // This is for the edge case where the clock is not set and the device won't connect as it thinks it is off hours
      sysStatus.closeTime = 24;                                        // This only resets if the device beleives it is off-hours
      stayAwakeTimeStamp = millis();
      stayAwake = stayAwakeLong;
      systemStatusWriteNeeded = true;
    }
    else if (Time.hour() < sysStatus.closeTime && Time.hour() >= sysStatus.openTime) { // It is opening time.  Park is open let's get ready for the day
      sensorControl(true);                                             // Turn on the sensor module
      attachInterrupt(intPin, sensorISR, RISING);                      // Pressure Sensor interrupt from low to high
      stayAwake = stayAwakeLong;                                       // Keeps Boron awake after deep sleep - may not be needed
    }
    } break;

  case NAPPING_STATE: {                                                // This state puts the device in low power mode quickly - napping supports the sensor activity and interrupts
    if (state != oldState) publishStateTransition();
    if (sensorDetect || countSignalTimer.isActive())  break;           // Don't nap until we are done with event - exits back to main loop but stays in napping state
    if (Particle.connected() || !Cellular.isOff()) {
      if (!disconnectFromParticle()) {                                 // Disconnect cleanly from Particle and power down the modem
        state = ERROR_STATE;
        current.alerts = 15;
        resetTimeStamp = millis();
        break;
      }
    }
    stayAwake = 1000;                                                  // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour
    state = IDLE_STATE;                                                // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    ab1805.stopWDT();                                                  // If we are sleeping, we will miss petting the watchdog
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    config.mode(SystemSleepMode::ULTRA_LOW_POWER)
      .gpio(userSwitch,CHANGE)
      .gpio(intPin,RISING)
      .duration(wakeInSeconds * 1000);
    SystemSleepResult result = System.sleep(config);                   // Put the device to sleep
    ab1805.resumeWDT();                                                // Wakey Wakey - WDT can resume
    fuelGauge.wakeup();                                                // Make sure the fuelGauge is woke
    stayAwakeTimeStamp = millis();
    if (result.wakeupPin() == userSwitch) setLowPowerMode("0");        // The user woke the device and we need to make sure it stays awake
    } break;

  
  case REPORTING_STATE:                                                // In this state we will publish the hourly totals to the queue and decide whether we should connect
    if (state != oldState) publishStateTransition();
    lastReportedTime = Time.now();                                     // We are only going to report once each hour from the IDLE state.  We may or may not connect to Particle
    if (current.backOff) current.backOff = false;                      // Let's try to connect again - new hour
    takeMeasurements();                                                // Take Measurements here for reporting
    if (Time.hour() == sysStatus.openTime) dailyCleanup();             // Once a day, clean house and publish to Google Sheets
    sendEvent();                                                       // Publish hourly but not at opening time as there is nothing to publish
    setVerboseMode("0");
    state = CONNECTING_STATE;                                          // Default behaviour would be to connect and send report to Ubidots

    // Let's see if we need to connect 
    if (Particle.connected()) {                                        // We are already connected go to response wait
      stayAwake = stayAwakeLong;                                       // Keeps device awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
      state = RESP_WAIT_STATE;
    }
    // If we are in a low battery state - we are not going to connect unless we are over-riding with user switch (active low)
    else if (sysStatus.lowBatteryMode && digitalRead(userSwitch)) {
      Log.info("Not connecting - low battery mode");
      state = IDLE_STATE;
    }
    // If we are in low power mode, we may bail if battery is too low and we need to reduce reporting frequency
    else if (sysStatus.lowPowerMode && digitalRead(userSwitch)) {      // Low power mode and user switch not pressed
      if (sysStatus.stateOfCharge > 65) {
        Log.info("Sufficient battery power connecting");
      }
      else if (sysStatus.stateOfCharge <= 50 && (Time.hour() % 4)) {   // If the battery level is <50%, only connect every fourth hour
        Log.info("Not connecting - <50%% charge - four hour schedule");
        state = IDLE_STATE;                                            // Will send us to connecting state - and it will send us back here
      }                                                                // Leave this state and go connect - will return only if we are successful in connecting
      else if (sysStatus.stateOfCharge <= 65 && (Time.hour() % 2)) {   // If the battery level is 50% -  65%, only connect every other hour
        Log.info("Not connecting - 50-65%% charge - two hour schedule");
        state = IDLE_STATE;                                            // Will send us to connecting state - and it will send us back here
        break;                                                         // Leave this state and go connect - will return only if we are successful in connecting
      }
    }
  break;

  case RESP_WAIT_STATE: {
    static unsigned long webhookTimeStamp = 0;                         // Webhook time stamp
    if (state != oldState) {
      webhookTimeStamp = millis();                                     // We are connected and we have published, head to the response wait state
      dataInFlight = true;                                             // set the data inflight flag
      publishStateTransition();
    }
    if (!dataInFlight)  {                                              // Response received --> back to IDLE state
      state = IDLE_STATE;
    }
    else if (millis() - webhookTimeStamp > webhookWait) {              // If it takes too long - will need to reset
      current.alerts = 40;                                             // Raise the missed webhook flag
      currentCountsWriteNeeded = true;
      state = ERROR_STATE;                                             // Go to the ERROR state to decide our fate
    }
  } break;

  case CONNECTING_STATE:{                                              // Will connect - or not and head back to the Idle state - We are using a 3,5, 7 minute back-off approach as recommended by Particle
    static State retainedOldState;                                     // Keep track for where to go next (depends on whether we were called from Reporting)
    static unsigned long connectionStartTimeStamp;                     // Time in Millis that helps us know how long it took to connect
    char data[64];                                                     // Holder for message strings

    if (state != oldState) {                                           // Non-blocking function - these are first time items
      Log.info("Initializing connection with a limit of %i", current.currentConnectionLimit);
      retainedOldState = oldState;                                     // Keep track for where to go next
      sysStatus.lastConnectionDuration = 0;                            // Will exit with 0 if we do not connect or are already connected.  If we need to connect, this will record connection time.
      publishStateTransition();
      connectionStartTimeStamp = millis();                             // Have to use millis as the clock may get reset on connect
      Particle.connect();                                              // Tells Particle to connect, now we need to wait
    }

    sysStatus.lastConnectionDuration = int((millis() - connectionStartTimeStamp)/1000);

    if (Particle.connected()) {
      systemStatusWriteNeeded = true;
      currentCountsWriteNeeded = true;
      if (!sysStatus.clockSet ) {                                      // Set the clock once a day - and send the location / signal strength
        sysStatus.clockSet = true;
        Particle.syncTime();                                           // Set the clock each day
        publishCellularInformation();                                  // Publish device cellular information once a day as well
      }
      if (current.currentConnectionLimit != 180) {                     // If we have reset due to connection speed, we need to add these back in
        if (current.currentConnectionLimit >= 300) sysStatus.lastConnectionDuration += 180;
        if (current.currentConnectionLimit == 420) sysStatus.lastConnectionDuration += 300;
      }
      current.currentConnectionLimit = 180;                            // Successful connection - resetting the connection timer
      sysStatus.lastConnection = Time.now();                           // This is the last time we last connected
      stayAwake = stayAwakeLong;                                       // Keeps device awake after reconnection - allows us some time to catch the device before it sleeps
      stayAwakeTimeStamp = millis();                                   // Start the stay awake timer now
      if (sysStatus.lastConnectionDuration > current.maxConnectTime) current.maxConnectTime = sysStatus.lastConnectionDuration; // Keep track of longest connection attempt each day
      getSignalStrength();                                             // Test signal strength since the cellular modem is on and ready
      snprintf(data, sizeof(data),"Connected in %i secs",sysStatus.lastConnectionDuration);  // Make up connection string and publish
      Log.info(data);
      if (sysStatus.verboseMode) Particle.publish("Cellular",data,PRIVATE);
      attachInterrupt(userSwitch, userSwitchISR,FALLING);              // Attach interrupt for the user switch to enable more verbose details if we are connected and the User button is pressed
      (retainedOldState == REPORTING_STATE) ? state = RESP_WAIT_STATE : state = IDLE_STATE; // so, if we are connecting to report - next step is response wait - otherwise IDLE
      if (sysStatus.verizonSIM && !sysStatus.lowPowerMode) Particle.keepAlive(60);    // Keeps connection alive if we are not in low power mode (Verizon has a shorter keep alive)
    }
    else if (sysStatus.lastConnectionDuration > current.currentConnectionLimit) { // What happens if we do not connect
      Log.info("Current connection duration = %i while the current connection limit is %i", sysStatus.lastConnectionDuration, current.currentConnectionLimit);
      currentCountsWriteNeeded = true;                                 // Record in FRAM as we will soon reset
      state = ERROR_STATE;                                             // Note - not setting the ERROR timestamp to make this go quickly
      switch (current.currentConnectionLimit) {
        case (180):                                                    // A connection limit of 180 indicates that this is a new attempt
          current.currentConnectionLimit = 300;                        // Increment the limit to the next value
          current.alerts = 32;                                         // Indicates we will reset and keep trying
          Log.info("3 Minute connection time exceeded - resetting");
          break; 
        case (300): 
          current.currentConnectionLimit = 420;                        // Increment the limit to the next value
          current.alerts = 32;                                         // Indicates we will reset and keep trying
          Log.info("5 Minute connection time exceeded - resetting");
          break;  
       case (420):
          current.currentConnectionLimit = 180;                        // Giving up - we are not going to connect this hour - reset for next
          current.alerts = 31;                                         // Indicates we are done with this attempt
          if (!sysStatus.lowPowerMode) current.backOff = true;         // If not in low power mode - wait till next hour before connecting
          Log.info("7 Minute connection time exceeded - giving up");
          break;                              
       }
       if (Cellular.ready()) {                                          // We connected to the cellular network but not to Particle
         Log.info("Connected to cellular but not Particle");
         current.alerts = 30;                                           // Record alert for timeout on Particle but connected to cellular
       }
    }
  } break;

  case ERROR_STATE: {                                                  // New and improved - now looks at the alert codes
    if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
    if (millis() > resetTimeStamp + resetWait) {                       // This simply gives us some time to catch the device if it is in a reset loop                           
      char data[64];                                                   // Let's publish to let folks know what is going on
      snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
      PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE);
      Log.info(data);
      delay(2000);

      if (Particle.connected() || !Cellular.isOff()) disconnectFromParticle();  // Disconnect cleanly from Particle and power down the modem

      switch (current.alerts) {                                        // All of these events will reset the device
        case 12:                                                       // This is an initialization error - likely FRAM - need to power cycle to clear
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 13:                                                       // Excessive resets of the device - time to power cycle
          sysStatus.resetCount = 0;                                    // Reset so we don't do this too often
          fram.put(FRAM::systemStatusAddr,sysStatus);                  // Won't get back to the main loop
          delay (100);
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 14:                                                       // This is an out of memory error
          System.reset();
          break;

        case 15:                                                       // This is a modem or cellular power down error - need to power cycle to clear
          ab1805.deepPowerDown();                                      // 30 second power cycle of Boron including cellular modem, carrier board and all peripherals
          break;

        case 30:                                                       // We connected to cellular but not to Particle
          System.reset();
          break;

        case 31:                                                       // Device failed to connect - bailing for this hour or resetting depending
          if (Time.now() - sysStatus.lastConnection > 2 * 3600L) {     // If we fail to connect for a couple hours in a row, let's power cycle the device
            sysStatus.lastConnection = Time.now();                     // Make sure we don't do this very often
            fram.put(FRAM::systemStatusAddr,sysStatus);                // Unless a FRAM error sent us here - store alerts value
            delay(100);                                                // Time to write to FRAM
            ab1805.deepPowerDown();                                    // Time to power cycle the device
          }
          state = IDLE_STATE;                                          // Go back to the main loop - try again next hour
          Log.info("Failed to connect - try again next hour");
          break;

        case 32:                                                       // We are still trying to connect but need to reset first as it is not happening quickly enough
          System.reset();
          break;

        case 40:                                                       // This is for failed webhook responses over three hours
          if (Time.now() - sysStatus.lastHookResponse > 3 * 3600L) {      // Failed to get a webhook response for over three hours
            System.reset();
          }
          state = IDLE_STATE;
          Log.info("Failed to get Webhook response - try again next hour");
          break;

        default:                                                       // Make sure that, no matter what - we do not get stuck here
          System.reset();
          break;
      }
    }
    } break;
  }

  // Take care of housekeeping items here

  if (sensorDetect) recordCount();                                     // The ISR had raised the sensor flag - this will service interrupts regardless of state

  if (userSwitchDetect) {                                              // If connected, this will trigger publishing the device location and cellular signal strength
    userSwitchDetect = false;
    state = REPORTING_STATE;                                           // This will send both the current counts / information and the cellular data
    publishCellularInformation();                                      // Send the data
  } 

  ab1805.loop();                                                       // Keeps the RTC synchronized with the Boron's clock

  PublishQueuePosix::instance().loop();                                // Check to see if we need to tend to the message queue

  if (systemStatusWriteNeeded) {                                       // These flags get set when a value is changed
    fram.put(FRAM::systemStatusAddr,sysStatus);
    systemStatusWriteNeeded = false;
  }
  if (currentCountsWriteNeeded) {
    fram.put(FRAM::currentCountsAddr,current);
    currentCountsWriteNeeded = false;
  }

  if (outOfMemory >= 0) {                                              // In this function we are going to reset the system if there is an out of memory error
    current.alerts = 14;                                               // Out of memory alert
    resetTimeStamp = millis();
    state = ERROR_STATE;
  }
  // End of housekeeping - end of main loop
}


void sensorControl(bool enableSensor) {                                // What is the sensor type - 0-Pressure Sensor, 1-PIR Sensor

  if (enableSensor) {
    digitalWrite(disableModule,false);                                 // Enable or disable the sensor

    if (sysStatus.sensorType == 0) {                                   // This is the pressure sensor and we are enabling it
        digitalWrite(ledPower,HIGH);                                   // For the pressure sensor, this is how you activate it
    }
    else {
        digitalWrite(ledPower,LOW);                                    // Turns on the LED on the PIR sensor board
    }
  }

  else {
    digitalWrite(disableModule,true);

    if (sysStatus.sensorType == 0) {                                   // This is the pressure sensor and we are enabling it
        digitalWrite(ledPower,LOW);                                    // Turns off the LED on the pressure sensor board
    }
    else {
        digitalWrite(ledPower,HIGH);                                   // Turns off the LED on the PIR sensor board
    }
  }

}

/**
 * @brief This function is called once a hardware interrupt is triggered by the device's sensor
 * 
 * @details The sensor may change based on the settings in sysSettings but the overall concept of operations
 * is the same regardless.  The sensor will trigger an interrupt, which will set a flag. In the main loop
 * that flag will call this function which will determine if this event should "count" as a visitor.
 * 
 */
void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the device
{
  static byte currentMinutePeriod;                                    // Current minute

  pinSetFast(blueLED);                                                // Turn on the blue LED
  countSignalTimer.reset();                                           // Keep the LED on for a set time so we can see it.

  if (currentMinutePeriod != Time.minute()) {                         // Done counting for the last minute
    currentMinutePeriod = Time.minute();                              // Reset period
    current.maxMinValue = 1;                                          // Reset for the new minute
  }
  current.maxMinValue++;

  current.lastCountTime = Time.now();
  current.hourlyCount++;                                              // Increment the PersonCount
  current.dailyCount++;                                               // Increment the PersonCount
  Log.info("Count, hourly: %i. daily: %i",current.hourlyCount,current.dailyCount);
  delay(200);
  if (sysStatus.verboseMode && Particle.connected()) {
    char data[256];                                                   // Store the date in this character array - not global
    if (!sysStatus.verboseCounts) {
      snprintf(data, sizeof(data), "Count, hourly: %i, daily: %i",current.hourlyCount,current.dailyCount);
      waitUntil(meterParticlePublish);
      Particle.publish("Count",data, PRIVATE);                        // Helpful for monitoring and calibration
    }
    else {
      snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i}",current.hourlyCount, current.dailyCount);
      waitUntil(meterParticlePublish);
      Particle.publish("Verbose_Counts_Hook", data, PRIVATE);
    }
  }
  currentCountsWriteNeeded = true;                                    // Write updated values to FRAM
  sensorDetect = false;                                               // Reset the flag
}

/**
 * @brief This functions sends the current data payload to Ubidots using a Webhook
 *
 * @details This idea is that this is called regardless of connected status.  We want to send regardless and connect if we can later
 * The time stamp is the time of the last count or the beginning of the hour if there is a zero hourly count for that period
 *
 *
 */
void sendEvent() {
  char data[256];                                                     // Store the date in this character array - not global
  unsigned long timeStampValue;                                       // Going to start sending timestamps - and will modify for midnight to fix reporting issue
  if (current.hourlyCount) {
    timeStampValue = current.lastCountTime;                           // If there was an event in the past hour, send the most recent event's timestamp
  }
  else {                                                              // If there were no events in the past hour/recording period, send the time when the last report was sent
    timeStampValue = lastReportedTime;                                // This should be the beginning of the current hour
  }
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i,\"key1\":\"%s\",\"temp\":%i, \"resets\":%i, \"alerts\":%i,\"maxmin\":%i,\"connecttime\":%i,\"timestamp\":%lu000}",current.hourlyCount, current.dailyCount, sysStatus.stateOfCharge, batteryContext[sysStatus.batteryState], current.temperature, sysStatus.resetCount, current.alerts, current.maxMinValue, sysStatus.lastConnectionDuration, timeStampValue);
  PublishQueuePosix::instance().publish("Ubidots-Counter-Hook-v1", data, PRIVATE | WITH_ACK);
  Log.info("Ubidots Webhook: %s", data);                              // For monitoring via serial
  current.hourlyCount = 0;                                            // Reset the hourly count
  current.alerts = 0;                                                 // Reset the alert after publish
}

/**
 * @brief This function published system status information daily to a Google Sheet where I can monitor config / health for the fleet
 *
 * @details These are values that don't need to be reported hourly and many have string values that Ubidots struggles with.  Testing this approach
 * to see if it can give me a more consistent view of fleet health and allow me to see device configuration when it is off-line
 *
 * @link https://docs.particle.io/datasheets/app-notes/an011-publish-to-google-sheets/ @endlink
 *
 */
void publishToGoogleSheets() {
  char data[256];                                                     // Store the date in this character array - not global
  char solarString[16];
  char verboseString[16];
  (sysStatus.solarPowerMode) ? strncpy(solarString,"Solar",sizeof(solarString)) : strncpy(solarString,"Utility",sizeof(solarString));
  (sysStatus.verboseMode) ? strncpy(verboseString, "Verbose",sizeof(verboseString)) : strncpy(verboseString, "Not Verbose",sizeof(verboseString));

  snprintf(data, sizeof(data), "[\"%s\",\"%s\",\"%s\",\"%s\",\"%s\",\"%s\",\"%s\",\"%i sec\",\"%i%%\",\"%s\"]", solarString, lowPowerModeStr, currentOffsetStr, openTimeStr, closeTimeStr, sensorTypeConfigStr, verboseString, current.maxConnectTime, current.minBatteryLevel,simCardStr);
  PublishQueuePosix::instance().publish("GoogleSheetsExport", data, PRIVATE | WITH_ACK);
  Log.info("Google Sheets: %s", data);

}


/**
 * @brief This function publishes deviceVitals data that can be used to show were the device is on a map and what the signal strength is
 * 
 * @details Uses the device Vitals helper library to access the required data.  Note, this function needs to be called when connected
 * 
 * @link: https://help.ubidots.com/en/articles/2133222-ubifunctions-integrate-google-s-geolocation-api-with-ubidots
 * 
 */
void publishCellularInformation() {
  char data[256];

  Particle.publishVitals();

  CellularSignal sig = Cellular.RSSI();
  float strength = sig.getStrength();

  snprintf(data,sizeof(data),"{\"cellstr\":%4.2f, \"cellId\":%li,\"locationAreaCode\":%li,\"mobileCountryCode\":%li, \"mobileNetworkCode\":%li}",strength, DiagnosticsHelper::getValue(DIAG_ID_NETWORK_CELLULAR_CELL_GLOBAL_IDENTITY_CELL_ID), DiagnosticsHelper::getValue(DIAG_ID_NETWORK_CELLULAR_CELL_GLOBAL_IDENTITY_LOCATION_AREA_CODE), DiagnosticsHelper::getValue(DIAG_ID_NETWORK_CELLULAR_CELL_GLOBAL_IDENTITY_MOBILE_COUNTRY_CODE), DiagnosticsHelper::getValue(DIAG_ID_NETWORK_CELLULAR_CELL_GLOBAL_IDENTITY_MOBILE_NETWORK_CODE));
  PublishQueuePosix::instance().publish("Daily-Location-Signal-Webhook", data, PRIVATE | WITH_ACK);
  Log.info(data);

}

void UbidotsHandler(const char *event, const char *data) {            // Looks at the response from Ubidots - Will reset Photon if no successful response
  char responseString[64];
    // Response is only a single number thanks to Template
  if (!strlen(data)) {                                                // No data in response - Error
    snprintf(responseString, sizeof(responseString),"No Data");
  }
  else if (atoi(data) == 200 || atoi(data) == 201) {
    snprintf(responseString, sizeof(responseString),"Response Received");
    sysStatus.lastHookResponse = Time.now();                          // Record the last successful Webhook Response
    systemStatusWriteNeeded = true;
    dataInFlight = false;                                             // Data has been received
  }
  else {
    snprintf(responseString, sizeof(responseString), "Unknown response recevied %i",atoi(data));
  }
  if (sysStatus.verboseMode && Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Ubidots Hook", responseString, PRIVATE);
  }
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  getTemperature();                                                    // Get Temperature at startup as well

  sysStatus.batteryState = System.batteryState();                      // Call before isItSafeToCharge() as it may overwrite the context

  sysStatus.stateOfCharge = int(fuelGauge.getSoC());                   // Assign to system value

  if (isItSafeToCharge() && sysStatus.stateOfCharge < 65 && sysStatus.batteryState == 1) {  // This is an issue - it is safe to charge, the battery needs charging and yet there is no charging (not this is different than discharging)
    System.setPowerConfiguration(SystemPowerConfiguration());          // Reset the PMIC
    current.alerts = 11;                                               // Keep track of this
  }

  if (sysStatus.stateOfCharge < current.minBatteryLevel) {
    current.minBatteryLevel = sysStatus.stateOfCharge;                 // Keep track of lowest value for the day
    currentCountsWriteNeeded = true;
  }

  if (sysStatus.stateOfCharge < 30) {
    sysStatus.lowBatteryMode = true;                                   // Check to see if we are in low battery territory
    if (!sysStatus.lowPowerMode) setLowPowerMode("1");                 // Should be there already but just in case...
  }
  else sysStatus.lowBatteryMode = false;                               // We have sufficient to continue operations

  systemStatusWriteNeeded = true;
}


void getSignalStrength() {
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();

  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {                                                // Get temperature and make sure we are not getting a spurrious value

  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  if (reading < 400) {                                                // This ocrresponds to 0 degrees - less than this and we should take another reading to be sure
    delay(50);
    reading = analogRead(tmp36Pin);
  }
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  current.temperature = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  currentCountsWriteNeeded=true;
  return current.temperature;
}


bool isItSafeToCharge()                                               // Returns a true or false if the battery is in a safe charging range.
{
  if (current.temperature < 36 || current.temperature > 100 )  {      // Reference: https://batteryuniversity.com/learn/article/charging_at_high_and_low_temperatures (32 to 113 but with safety)
    setPowerConfig(false);                                            // Disables charging
    sysStatus.batteryState = 1;                                       // Overwrites the values from the batteryState API to reflect that we are "Not Charging"
    current.alerts = 10;                                              // Set a value of 1 indicating that it is not safe to charge due to high / low temps
    return false;
  }
  else {
    setPowerConfig(true);
    if (current.alerts == 10) current.alerts = 0;                      // Reset the alerts flag if we previously had disabled charging
    return true;
  }
  
 return true;
}


// Power Management function
int setPowerConfig(bool enableCharging) {
  SystemPowerConfiguration conf;
  System.setPowerConfiguration(SystemPowerConfiguration());            // To restore the default configuration

  if (!enableCharging) {
    conf.feature(SystemPowerFeature::DISABLE_CHARGING);
    int res = System.setPowerConfiguration(conf);
    return res;
  }
  else if (sysStatus.solarPowerMode) {
    conf.powerSourceMaxCurrent(900)                                    // Set maximum current the power source can provide (applies only when powered through VIN)
        .powerSourceMinVoltage(5080)                                   // Set minimum voltage the power source can provide (applies only when powered through VIN)
        .batteryChargeCurrent(900)                                     // Set battery charge current
        .batteryChargeVoltage(4208)                                    // Set battery termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST);  // For the cases where the device is powered through VIN
                                                                       // but the USB cable is connected to a USB host, this feature flag
                                                                       // enforces the voltage/current limits specified in the configuration
                                                                       // (where by default the device would be thinking that it's powered by the USB Host)
    int res = System.setPowerConfiguration(conf);                      // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
  else  {
    conf.powerSourceMaxCurrent(900)                                   // default is 900mA
        .powerSourceMinVoltage(4208)                                  // This is the default value for the Boron
        .batteryChargeCurrent(900)                                    // higher charge current from DC-IN when not solar powered
        .batteryChargeVoltage(4112)                                   // default is 4.112V termination voltage
        .feature(SystemPowerFeature::USE_VIN_SETTINGS_WITH_USB_HOST) ;
    int res = System.setPowerConfiguration(conf);                     // returns SYSTEM_ERROR_NONE (0) in case of success
    return res;
  }
}


// Here are the various hardware and timer interrupt service routines
void outOfMemoryHandler(system_event_t event, int param) {
    outOfMemory = param;
}


void sensorISR()
{
  static bool frontTireFlag = false;
  if (frontTireFlag || sysStatus.sensorType == 1) {                   // Counts the rear tire for pressure sensors and once for PIR
    sensorDetect = true;                                              // sets the sensor flag for the main loop
    frontTireFlag = false;
  }
  else frontTireFlag = true;
}

void userSwitchISR() {
  userSwitchDetect = true;                                            // The the flag for the user switch interrupt
}

void countSignalTimerISR() {
  digitalWrite(blueLED,LOW);
}



void loadSystemDefaults() {                                           // Default settings for the device - connected, not-low power and always on
  if (Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("Mode","Loading System Defaults", PRIVATE);
  }
  Log.info("Loading system defaults");
  sysStatus.structuresVersion = 1;
  sysStatus.verboseMode = false;
  sysStatus.verboseCounts = false;
  sysStatus.lowBatteryMode = false;
  if (digitalRead(userSwitch)) setLowPowerMode("1");                  // Low power mode or not depending on user switch
  else setLowPowerMode("0");

  sysStatus.timezone = -5;                                            // Default is East Coast Time
  sysStatus.dstOffset = 1;
  sysStatus.openTime = 6;
  sysStatus.closeTime = 22;                                           // New standard with v20
  sysStatus.solarPowerMode = true;
  sysStatus.lastConnectionDuration = 0;                               // New measure
  fram.put(FRAM::systemStatusAddr,sysStatus);                         // Write it now since this is a big deal and I don't want values over written

  current.currentConnectionLimit = 180;
  fram.put(FRAM::currentCountsAddr,current);
}

 /**
  * @brief This function checks to make sure all values that we pull from FRAM are in bounds
  *
  * @details As new devices are comissioned or the sysStatus structure is changed, we need to make sure that values are
  * in bounds so they do not cause unpredectable execution.
  *
  */
void checkSystemValues() {                                          // Checks to ensure that all system values are in reasonable range
  if (sysStatus.sensorType > 1) {                                   // Values are 0 for Pressure and 1 for PIR
    sysStatus.sensorType = 0;
    strncpy(sensorTypeConfigStr,"Pressure Sensor",sizeof(sensorTypeConfigStr));
  }
  if (sysStatus.resetCount < 0 || sysStatus.resetCount > 255) sysStatus.resetCount = 0;
  if (sysStatus.timezone < -12 || sysStatus.timezone > 12) sysStatus.timezone = -5;
  if (sysStatus.dstOffset < 0 || sysStatus.dstOffset > 2) sysStatus.dstOffset = 1;
  if (sysStatus.openTime < 0 || sysStatus.openTime > 12) sysStatus.openTime = 0;
  if (sysStatus.closeTime < 12 || sysStatus.closeTime > 24) sysStatus.closeTime = 24;
  if (sysStatus.verizonSIM != 0 && sysStatus.verizonSIM != 1) {
    delay(3000);
    Log.info("resetting the SIM card");
    sysStatus.verizonSIM = 0; // Default to non-Verizon SIM
  }
  if (current.currentConnectionLimit < 180 || current.currentConnectionLimit > 420) current.currentConnectionLimit = 180;

  // None for lastHookResponse
  systemStatusWriteNeeded = true;
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

 /**
  * @brief Simple Function to construct the strings that make the console easier to read
  *
  * @details Looks at all the system setting values and creates the appropriate strings.  Note that this
  * is a little inefficient but it cleans up a fair bit of code.
  *
  */
void makeUpStringMessages() {

  if (sysStatus.openTime == 0 && sysStatus.closeTime == 24) {                                 // Special case for 24 hour operations
    snprintf(openTimeStr, sizeof(openTimeStr), "NA");
    snprintf(closeTimeStr, sizeof(closeTimeStr), "NA");
  }
  else {
    snprintf(openTimeStr, sizeof(openTimeStr), "%i:00", sysStatus.openTime);                  // Open and Close Times
    snprintf(closeTimeStr, sizeof(closeTimeStr), "%i:00", sysStatus.closeTime);
  }

  if (sysStatus.lowPowerMode) strncpy(lowPowerModeStr,"Low Power", sizeof(lowPowerModeStr));  // Low Power Mode Strings
  else strncpy(lowPowerModeStr,"Not Low Power", sizeof(lowPowerModeStr));


  if (sysStatus.sensorType == 0) strncpy(sensorTypeConfigStr,"Pressure Sensor",sizeof(sensorTypeConfigStr));  // Sensor strings
  else if (sysStatus.sensorType == 1) strncpy(sensorTypeConfigStr,"PIR Sensor",sizeof(sensorTypeConfigStr));
  else strncpy(sensorTypeConfigStr,"Unknown Sensor",sizeof(sensorTypeConfigStr));

  if (sysStatus.verizonSIM) strncpy(simCardStr,"Verizon", sizeof(simCardStr));
  else strncpy(simCardStr,"Particle", sizeof(simCardStr));

  return;
}

/**
 * @brief Disconnect from Particle function has one purpose - to disconnect the Boron from Particle, the cellular network and to power down the cellular modem to save power
 * 
 * @details This function is supposed to be able to handle most issues and tee up a power cycle of the Boron and the Cellular Modem if things do not go as planned
 * 
 * @return true - Both disconnect from Particle and the powering down of the cellular modem are successful
 * @return false - One or the other failed and the device will go to the ERROR_STATE to be power cycled.
 */

bool disconnectFromParticle()                                          // Ensures we disconnect cleanly from Particle
                                                                       // Updated based on this thread: https://community.particle.io/t/waitfor-particle-connected-timeout-does-not-time-out/59181
{
  time_t startTime = Time.now();
  Log.info("In the disconnect from Particle function");
  detachInterrupt(userSwitch);                                         // Stop watching the userSwitch as we will no longer be connected
  // First, we need to disconnect from Particle
  Particle.disconnect();                                               // Disconnect from Particle
  waitForNot(Particle.connected, 15000);                               // Up to a 15 second delay() 
  Particle.process();
  if (Particle.connected()) {                      // As this disconnect from Particle thing can be a·syn·chro·nous, we need to take an extra step to wait, 
    Log.info("Failed to disconnect from Particle");
    return(false);
  }
  else Log.info("Disconnected from Particle in %i seconds", (int)(Time.now() - startTime));
  // Then we need to disconnect from Cellular and power down the cellular modem
  startTime = Time.now();
  Cellular.disconnect();                                               // Disconnect from the cellular network
  Cellular.off();                                                      // Turn off the cellular modem
  waitFor(Cellular.isOff, 30000);                                      // As per TAN004: https://support.particle.io/hc/en-us/articles/1260802113569-TAN004-Power-off-Recommendations-for-SARA-R410M-Equipped-Devices
  Particle.process();
  if (Cellular.isOn()) {                                               // At this point, if cellular is not off, we have a problem
    Log.info("Failed to turn off the Cellular modem");
    return(false);                                                     // Let the calling function know that we were not able to turn off the cellular modem
  }
  else {
    Log.info("Turned off the cellular modem in %i seconds", (int)(Time.now() - startTime));
    return true;
  }
}

int resetCounts(String command)                                        // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    current.dailyCount = 0;                                            // Reset Daily Count in memory
    current.hourlyCount = 0;                                           // Reset Hourly Count in memory
    sysStatus.resetCount = 0;                                          // If so, store incremented number - watchdog must have done This
    dataInFlight = false;
    currentCountsWriteNeeded = true;                                   // Make sure we write to FRAM back in the main loop
    systemStatusWriteNeeded = true;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the device
{
  if (command == "1")
  {
    if (Particle.connected()) Particle.publish("Reset","Hard Reset in 2 seconds",PRIVATE);
    delay(2000);
    ab1805.deepPowerDown(10);                                         // Power cycles the Boron and carrier board
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    publishCellularInformation();
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

/**
 * @brief Resets all counts to start a new day.
 *
 * @details Once run, it will reset all daily-specific counts and trigger an update in FRAM.
 */
void resetEverything() {                                              // The device is waking up in a new day or is a new install
  char data[64];
  current.dailyCount = 0;                                             // Reset the counts in FRAM as well
  current.hourlyCount = 0;
  current.lastCountTime = Time.now();                                 // Set the time context to the new day
  current.maxConnectTime = 0;                                         // Reset values for this time period
  current.minBatteryLevel = 100;
  currentCountsWriteNeeded = true;
  if (current.alerts == 23 || current.updateAttempts >=3) {           // We had tried to update enough times that we disabled updates for the day - resetting
    System.enableUpdates();
    current.alerts = 0;
    snprintf(data, sizeof(data), "{\"alerts\":%i,\"timestamp\":%lu000 }",current.alerts, Time.now());
    PublishQueuePosix::instance().publish("Ubidots_Alert_Hook", data, PRIVATE); // Put in publish queue
  }
  current.updateAttempts = 0;                                         // Reset the update attempts counter for the day
  currentCountsWriteNeeded=true;                                      // Make sure that the values are updated in FRAM

  sysStatus.resetCount = 0;                                           // Reset the reset count as well
  systemStatusWriteNeeded = true;
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    sysStatus.solarPowerMode = true;
    setPowerConfig(true);                                               // Change the power management Settings
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.solarPowerMode = false;
    systemStatusWriteNeeded=true;
    setPowerConfig(true);                                                // Change the power management settings
    if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setVerizonSIM(String command)                                   // If we are using a Verizon SIM, we will need to execute "keepAlive" calls in the main loop when not in low power mode
{
  if (command == "1")
  {
    sysStatus.verizonSIM = true;
    systemStatusWriteNeeded = true;
    Particle.keepAlive(60);                                         // send a ping every minute
    makeUpStringMessages();
    if (Particle.connected()) Particle.publish("Mode","Set to Verizon SIM", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verizonSIM = false;
    systemStatusWriteNeeded = true;
    Particle.keepAlive(23 * 60);                                     // send a ping every 23 minutes
    makeUpStringMessages();
    if (Particle.connected()) Particle.publish("Mode","Set to Particle SIM", PRIVATE);
    return 1;
  }
  else return 0;
}


/**
 * @brief Set the Sensor Type object
 *
 * @details Over time, we may want to develop and deploy other sensot types.  The idea of this code is to allow us to select the sensor
 * we want via the console so all devices can run the same code.
 *
 * @param command a string equal to "0" for pressure sensor and "1" for PIR sensor.  More sensor types possible in the future.
 *
 * @return returns 1 if successful and 0 if not.
 */
int setSensorType(String command)                                     // Function to force sending data in current hour
{
  if (command == "0")
  {
    sysStatus.sensorType = 0;
    strncpy(sensorTypeConfigStr,"Pressure Sensor", sizeof(sensorTypeConfigStr));
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Sensor Mode to Pressure", PRIVATE);

    return 1;
  }
  else if (command == "1")
  {
    sysStatus.sensorType = 1;
    strncpy(sensorTypeConfigStr,"PIR Sensor", sizeof(sensorTypeConfigStr));
    systemStatusWriteNeeded=true;
    if (Particle.connected()) Particle.publish("Mode","Set Sensor Mode to PIR", PRIVATE);
    return 1;
  }

  else return 0;
}

/**
 * @brief Turns on/off verbose mode.
 *
 * @details Extracts the integer command. Turns on verbose counts mode if command is "2", verbose mode if the command is "1" and turns
 * off verbose mode if the command is "0".
 *
 * @param command A string with the integer command indicating to turn on or off verbose mode.
 * Only values of "0", "1" or "2" are accepted. Values outside this range will cause the function
 * to return 0 to indicate an invalid entry.
 *
 * @return 1 if successful, 0 if invalid command
 */
int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "2")
  {
    sysStatus.verboseMode = true;
    sysStatus.verboseCounts = true;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Counts Mode - resets at top of hour", PRIVATE);
    return 1;
  }
  if (command == "1")
  {
    sysStatus.verboseMode = true;
    sysStatus.verboseCounts = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    sysStatus.verboseMode = false;
    sysStatus.verboseCounts = false;
    systemStatusWriteNeeded = true;
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

/**
 * @brief Returns a string describing the battery state.
 *
 * @return String describing battery state.
 */
String batteryContextMessage() {
  return batteryContext[sysStatus.batteryState];
}

/**
 * @brief Sets the closing time of the facility.
 *
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                             // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                     // Make sure it falls in a valid range or send a "fail" result
  sysStatus.openTime = tempTime;
  makeUpStringMessages();                                              // Updated system settings - refresh the string messages
  systemStatusWriteNeeded = true;                                      // Need to store to FRAM back in the main loop
  if (Particle.connected()) {
    snprintf(data, sizeof(data), "Open time set to %i",sysStatus.openTime);
    Particle.publish("Time",data, PRIVATE);
  }
  return 1;
}

/**
 * @brief Sets the closing time of the facility.
 *
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating what the closing hour of the facility is in 24-hour time.
 * Inputs outside of "0" - "24" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 24)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  sysStatus.closeTime = tempTime;
  makeUpStringMessages();                                           // Updated system settings - refresh the string messages
  systemStatusWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",sysStatus.closeTime);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

/**
 * @brief Sets the daily count for the park - useful when you are replacing sensors.
 *
 * @details Since the hourly counts are not retained after posting to Ubidots, seeding a value for
 * the daily counts will enable us to follow this process to replace an old counter: 1) Execute the "send now"
 * command on the old sensor.  Note the daily count.  2) Install the new sensor and perform tests to ensure
 * it is counting correclty.  3) Use this function to set the daily count to the right value and put the
 * new device into operation.
 *
 * @param command A string for the new daily count.
 * Inputs outside of "0" - "1000" will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setDailyCount(String command)
{
  char * pEND;
  char data[256];
  int tempCount = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempCount < 0) || (tempCount > 2000)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  current.dailyCount = tempCount;
  current.lastCountTime = Time.now();
  currentCountsWriteNeeded = true;                          // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Daily count set to %i",current.dailyCount);
  if (Particle.connected()) Particle.publish("Daily",data, PRIVATE);
  return 1;
}

/**
 * @brief Toggles the device into low power mode based on the input command.
 *
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * A "1" indicates low power mode, a "0" indicates normal mode. Inputs that are neither of these commands
 * will cause the function to return 0 to indicate an invalid entry.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    sysStatus.lowPowerMode = true;
    makeUpStringMessages();                                           // Updated system settings - refresh the string messages
    Log.info("Set Low Power Mode");
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    sysStatus.lowPowerMode = false;
    makeUpStringMessages();                                           // Updated system settings - refresh the string messages
    Log.info("Cleared Low Power Mode");
    if (!Particle.connected()) {                                 // In case we are not connected, we will do so now.
      state = CONNECTING_STATE;                                       // Will connect - if connection fails, will need to reset device
    }
    else {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode",lowPowerModeStr, PRIVATE);
    }
  }
  systemStatusWriteNeeded = true;
  return 1;
}


/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 *
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if (sysStatus.verboseMode) {
    if (Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("State Transition",stateTransitionString, PRIVATE);
    }
  }
  Log.info(stateTransitionString);
}


/**
 * @brief Cleanup function that is run at the beginning of the day.
 *
 * @details May or may not be in connected state.  Syncs time with remote service and sets low power mode.
 * Called from Reporting State ONLY. Cleans house at the beginning of a new day.
 */
void dailyCleanup() {
  if (Particle.connected()) Particle.publish("Daily Cleanup","Running", PRIVATE);   // Make sure this is being run
  Log.info("Running Daily Cleanup");
  sysStatus.verboseMode = false;                                       // Saves bandwidth - keep extra chatter off
  sysStatus.clockSet = false;                                          // Once a day we need to reset the clock
  isDSTusa() ? Time.beginDST() : Time.endDST();                        // Perform the DST calculation here - once a day

  if (sysStatus.solarPowerMode || sysStatus.stateOfCharge <= 65) {     // If Solar or if the battery is being discharged
    setLowPowerMode("1");
  }

  publishToGoogleSheets();                                             // Send data to Google Sheets on Product Status - whether we are connected or not
  resetEverything();                                                   // If so, we need to Zero the counts for the new day

  systemStatusWriteNeeded = true;
}

