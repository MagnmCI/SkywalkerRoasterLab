/***************************************************
 * HiBean ESP32 BLE Roaster Control
 *
 * Libraries Required: EMWA 1.0.2, PID 1.2.0
 ***************************************************/

#include <Arduino.h>
//#include <MedianFilterLib.h>
//#include <PID_v1.h>
#include <AutoTunePID.h>
#include <Ewma.h>

#include "SkiPinDefns.h"
#include "SerialDebug.h"
#include "SkiBLE.h"
#include "SkiComms.h"
#include "SkiLED.h"
#include "SkiCMD.h"

// -----------------------------------------------------------------------------
// Current Sketch and Release Version (for BLE device info)
// -----------------------------------------------------------------------------
String firmWareVersion = String("1.0.3");
String sketchName = String(__FILE__).substring(String(__FILE__).lastIndexOf('/')+1);

// -----------------------------------------------------------------------------
// Global Bean Temperature Variable
// -----------------------------------------------------------------------------
double temp          = 0.0;           // Filtered temperature

// -----------------------------------------------------------------------------
// Define PID variables
// -----------------------------------------------------------------------------
double pSetpoint = 0.0; // Desired temperature (adjustable on the fly)
float kP, kI, kD = 1.5, 0.2, 5.0;
enum PIDModes { OFF = 0, ON = 1, TUNE = 2 };
int CurrentPIDMode = 0;
AutoTunePID tempController(0, 100, TuningMethod::ZieglerNichols);
int manualHeatLevel = 50;

void setup() {
    rgbLedWrite(LED_PIN, LED_GREEN[0], LED_GREEN[1], LED_GREEN[2]);
    Serial.begin(115200);
    D_println("Starting HiBean ESP32 BLE Roaster Control.");
    delay(3000); //let fw upload finish before we take over hwcdc serial tx/rx

    D_println("Serial SERIAL_DEBUG ON!");
    
    #if SERIAL_DEBUG == 0
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);
    pinMode(RX_PIN, INPUT);
    #endif

    initBLE();
	
    // interrupt that flags when roaster preamble is found
    attachInterrupt(RX_PIN, watchRoasterStart, FALLING);

    // Set PID to start in MANUAL mode
    
    tempController.setOperationalMode(OperationalMode::Hold); // Don't run for now
    tempController.enableAntiWindup(true, 0.8); // Enable anti-windup
    tempController.setManualGains(float kP, float kI, float kD); // Set manual PID gains

    // Ensure heat starts at 0% for safety
    manualHeatLevel = 0;
    handleHEAT(manualHeatLevel);

    shutdown();
}

void loop() {
    // roaster shut down, clear our buffers   
    if (itsbeentoolong()) { shutdown(); }

    // roaster message start found, go get it
    if (roasterStartFound) { getRoasterMessage(); }

    // send roaster commands if any
    sendRoasterMessage();

    // Ensure PID or manual heat control is handled
    handlePIDControl();
    
    // update the led so user knows we're running
    handleLED();
}
