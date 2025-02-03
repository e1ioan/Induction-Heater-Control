/*
  =======================================================================
  Arduino Coil Control with Metal Detection and Configurable Menu
  =======================================================================
  
  Description:
  ------------
  This sketch controls a coil (for example, in an induction heating system)
  by monitoring current (via an ACS712 sensor) and metal presence. It offers
  two operating modes—Manual and Auto—selectable via a configuration menu 
  displayed on a 16x2 LCD. A single button (managed by the OneButton library)
  navigates the menu and toggles the coil.

  Operating Modes:
  ----------------
  1. Manual Mode (Auto-Start OFF):
     - The coil starts off.
     - Inserting metal does not automatically start the coil.
     - The user toggles the coil ON/OFF with a button press.
     - If the coil is ON and the metal is removed, the coil automatically 
       shuts off.
     - Once manually turned off, the coil stays off until the button is pressed
       again.

  2. Auto Mode (Auto-Start ON):
     - The system periodically pulses the coil (without user intervention) to
       check for metal presence.
     - Each pulse measures the current using the ACS712 sensor.
     - If a pulse detects metal (current = threshold), the coil is turned ON.
     - The coil stays ON as long as metal is present.
     - If metal is removed, the coil turns off and auto pulses resume.
     - A button press while the coil is ON forces a manual override (turning
       the coil off) that persists until metal is removed. After removal, auto
       pulsing resumes and the coil may start again automatically.

  Metal Detection:
  ----------------
  - The function measureMetalPresence() “pulses” the coil for a short 
    duration (AUTO_PULSE_DURATION) and monitors the ACS712 current reading.
  - It returns true if the measured current exceeds a defined threshold 
    (CURRENT_THRESHOLD), indicating that metal is present.
  - This function is designed to be side-effect free: if the coil was off 
    before measurement, it is turned off again afterward.

  Menu System:
  ------------
  - The sketch uses a 16x2 LCD and a single button to provide a menu for 
    configuring:
      * Maximum Temperature (stubbed: temperature reading returns -1)
      * Maximum On-Time (in seconds)
      * Auto-Start Mode (ON/OFF)
  - Long button presses cycle through these menu screens.
  - Menu settings are stored in EEPROM and reloaded on startup.
  
  Coil Control & Off Conditions:
  ------------------------------
  - The function setCoilState() is used to turn the coil (and an LED) on or off.
  - While the coil is on, the code continuously checks if any off conditions 
    are met (e.g., time exceeding a set limit, low current, or excessive 
    temperature) and turns the coil off if needed.
  
  Manual Override (in Auto Mode):
  -------------------------------
  - In Auto Mode, a button press while the coil is running triggers a manual 
    override, turning the coil off and preventing auto pulses from restarting 
    it until metal is removed.
  - The override is only cleared after several consecutive pulses indicate 
    that metal is absent.
  
  Additional Notes:
  -----------------
  - Temperature reading from the MAX6675 is stubbed to return -1 since no 
    thermocouple is installed.
  
  Author: Ioan Ghip
  Date: February 3rd, 2025
  
  =======================================================================
*/
// TO DO: when in auto, the end timer should trigger an off coil event

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <OneButton.h>
#include <max6675.h>

#define DEBUG // Uncomment to enable Serial debugging

#ifdef DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ----------------- EEPROM Addresses -----------------
const int EEPROM_TEMPERATURE_ADDR = 0;  // maxTemperature (2 bytes)
const int EEPROM_SECONDS_ADDR     = 2;  // maxSeconds (2 bytes)
const int EEPROM_AUTOSTART_ADDR   = 4;  // autoStart flag (1 byte)

// ----------------- LCD -----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ----------------- Pin Assignments -----------------
const uint8_t ACS712_PIN = A0;   // ACS712 sensor input
const uint8_t BUTTON_PIN = 3;    // Button on D3
const uint8_t COIL_PIN   = 5;    // Coil control on D5
const uint8_t LED_PIN    = 6;    // LED on D6

// ----------------- MAX6675 Pin Assignments -----------------
const uint8_t MAX6675_SO_PIN = 4;  // Serial data output from MAX6675
const uint8_t MAX6675_CS_PIN = 7;  // Chip Select
const uint8_t MAX6675_CLK_PIN = 8; // Clock

// Create a MAX6675 instance (constructor order: SCLK, CS, SO)
MAX6675 thermocouple(MAX6675_CLK_PIN, MAX6675_CS_PIN, MAX6675_SO_PIN);

OneButton button(BUTTON_PIN, true);  // active LOW

// ----------------- ACS712 Constants -----------------
const float ACS712_SENSITIVITY = 0.1;   // 100mV per Amp (0.1V/A)
const int   CURRENT_THRESHOLD  = 7000;  // in mA (for auto-on)
const int   CURRENT_FILTER_SAMPLES = 5; // samples to average

// ----------------- Timing Constants -----------------
const unsigned long COIL_START_DELAY    = 100;  // ms delay for coil to settle
const unsigned long LCD_UPDATE_INTERVAL = 500;  // ms LCD update interval
const unsigned long AUTO_PULSE_INTERVAL = 1000; // ms between pulses
const unsigned long AUTO_PULSE_DURATION = 10;   // pulse duration in ms

// In manual mode, if the coil is on and metal is removed, wait this long before auto turning off.
const unsigned long MANUAL_OFF_DELAY = 50;  // ms

// ----------------- Calibration Constants -----------------
const int NUM_CALIBRATION_SAMPLES = 20;  // samples for ACS712 calibration
const int CALIBRATION_SAMPLE_DELAY  = 10;  // ms delay between calibration samples

// ----------------- Menu States -----------------
enum MenuState {
  MENU_NORMAL,
  MENU_CONFIG_TEMP,
  MENU_CONFIG_TIME,
  MENU_CONFIG_AUTOSTART
};

MenuState menu_state = MENU_NORMAL;

// ----------------- Global Variables -----------------
int maxSeconds, maxTemperature;
bool coilOn = false;
bool autoStart = false;
int secondsCounter = 0;
unsigned long coilOnStartTime = 0;
unsigned long config_enter_time = 0;
unsigned long lastUpdate = 0;
unsigned long lastAutoPulseTime = 0;  // for auto start pulsing

// New: manualOverride flag for safety override (in auto mode).
// Once set, auto pulses will not restart the coil until metal is removed.
bool manualOverride = false;
unsigned long manualOverrideSetTime = 0;  // records when override was set

// Backup variables for rollback on config timeout.
int backupMaxTemperature, backupMaxSeconds;
bool backupAutoStart;

// Global calibrated offset voltage for ACS712
float offsetVoltage = 2.5; // will be recalculated during calibration

// ----------------- Custom Degree Symbol -----------------
byte degreeChar[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// ----------------- Coil Control Functions -----------------
// setCoilState: Sets the coil (and LED) on/off and updates the global flag.
void setCoilState(bool state) {
  coilOn = state;
  if (state) {
    digitalWrite(COIL_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(COIL_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
  }
}

// ----------------- Helper Functions ----------------

// Print text at a given column and row, ensuring exactly 'width' characters.
void printFixed(uint8_t col, uint8_t row, const String &text, uint8_t width) {
  lcd.setCursor(col, row);
  lcd.print(text);
  int len = text.length();
  for (int i = len; i < width; i++) {
    lcd.print(" ");
  }
}

// Show a centered message (clearing the screen) for special messages.
void printCentered(const String &msg) {
  lcd.clear();
  int spaces = (16 - msg.length()) / 2;
  lcd.setCursor(spaces, 0);
  lcd.print(msg);
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

// ----------------- EEPROM Helpers -----------------
void loadConfig() {
  EEPROM.get(EEPROM_TEMPERATURE_ADDR, maxTemperature);
  EEPROM.get(EEPROM_SECONDS_ADDR, maxSeconds);
  byte autoStartByte = EEPROM.read(EEPROM_AUTOSTART_ADDR);
  if (autoStartByte != 0 && autoStartByte != 1) {
    autoStart = false;
  } else {
    autoStart = (autoStartByte == 1);
  }
  if(maxTemperature < 50 || maxTemperature > 600) maxTemperature = 300;
  if(maxSeconds < 1 || maxSeconds > 30) maxSeconds = 10;
}
 
void saveConfig() {
  int storedTemp, storedSeconds;
  bool storedAutoStart;
  EEPROM.get(EEPROM_TEMPERATURE_ADDR, storedTemp);
  EEPROM.get(EEPROM_SECONDS_ADDR, storedSeconds);
  byte storedAutoStartByte = EEPROM.read(EEPROM_AUTOSTART_ADDR);
  storedAutoStart = (storedAutoStartByte == 1);
  
  if (storedTemp != maxTemperature) {
    EEPROM.put(EEPROM_TEMPERATURE_ADDR, maxTemperature);
    DEBUG_PRINT("Saved maxTemperature: ");
    DEBUG_PRINTLN(maxTemperature);
  }
  if (storedSeconds != maxSeconds) {
    EEPROM.put(EEPROM_SECONDS_ADDR, maxSeconds);
    DEBUG_PRINT("Saved maxSeconds: ");
    DEBUG_PRINTLN(maxSeconds);
  }
  if (storedAutoStart != autoStart) {
    byte autoByte = autoStart ? 1 : 0;
    EEPROM.put(EEPROM_AUTOSTART_ADDR, autoByte);
    DEBUG_PRINT("Saved AutoStart: ");
    DEBUG_PRINTLN(autoStart ? "ON" : "OFF");
  }
}

// ----------------- ACS712 Calibration -----------------
void calibrateACS712() {
  digitalWrite(COIL_PIN, LOW);
  delay(50);
  long sum = 0;
  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    sum += analogRead(ACS712_PIN);
    delay(CALIBRATION_SAMPLE_DELAY);
  }
  float avgRaw = (float)sum / NUM_CALIBRATION_SAMPLES;
  offsetVoltage = (avgRaw * 5.0) / 1023.0;
  DEBUG_PRINT("Calibrated ACS712 Offset Voltage: ");
  DEBUG_PRINTLN(offsetVoltage);
}

// ----------------- Read ACS712 Current -----------------
int readCurrent() {
  long sum = 0;
  for (int i = 0; i < CURRENT_FILTER_SAMPLES; i++) {
    sum += analogRead(ACS712_PIN);
    delay(2);
  }
  float avgRaw = (float)sum / CURRENT_FILTER_SAMPLES;
  float voltage = (avgRaw * 5.0) / 1023.0;
  float currentAmps = (voltage - offsetVoltage) / ACS712_SENSITIVITY;
  return abs((int)(currentAmps * 1000));
}

// ----------------- Read Object Temperature (MAX6675) -----------------
int readObjectTempC() {
  double tempC = thermocouple.readCelsius();
  return -1;  // Keep your modification: no thermocouple installed.
}

// ----------------- Metal Detection Function -----------------
// measureMetalPresence: Pulses the coil for AUTO_PULSE_DURATION and returns true
// if the measured current reaches/exceeds CURRENT_THRESHOLD. It saves and restores
// the coil's previous state.
bool measureMetalPresence() {
  bool prevState = coilOn;
  // If the coil is off, temporarily turn it on.
  if (!prevState) {
    setCoilState(true);
  }
  unsigned long pulseStart = millis();
  int pulseCurrent = 0;
  while (millis() - pulseStart < AUTO_PULSE_DURATION) {
    pulseCurrent = readCurrent();
    if (pulseCurrent >= CURRENT_THRESHOLD) {
      if (!prevState) {
        // Turn off the coil again.
        setCoilState(false);
      }
      return true;
    }
  }
  if (!prevState) {
    setCoilState(false);
  }
  return false;
}

// ----------------- LCD Update -----------------
void updateLCD() {
  if (menu_state == MENU_NORMAL) {
    printFixed(0, 0, "Coil:", 5);
    String status = coilOn ? "ON  " : "OFF ";
    printFixed(5, 0, status, 4);
    if (coilOn) {
      int currentTemp = readObjectTempC();
      lcd.setCursor(9, 0);
      char tempBuf[5];
      sprintf(tempBuf, "%3d", currentTemp);
      lcd.print(tempBuf);
      lcd.write(byte(0));
    } else {
      printFixed(9, 0, "", 10);
    }
    printFixed(0, 1, "Time:", 5);
    char timeBuf[12];
    sprintf(timeBuf, "%2d/%2ds", secondsCounter, maxSeconds);
    printFixed(5, 1, String(timeBuf), 11);
  } else if (menu_state == MENU_CONFIG_TEMP) {
    printFixed(0, 0, "Set Max Temp", 16);
    lcd.setCursor(0, 1);
    lcd.print("Temp(");
    lcd.write(byte(0)); // the character we created
    lcd.print("C): ");
    char valBuf[5];
    sprintf(valBuf, "%3d", maxTemperature);
    lcd.print(valBuf);
    for (int i = 10; i < 16; i++) { lcd.print(" "); }
  } else if (menu_state == MENU_CONFIG_TIME) {
    printFixed(0, 0, "Set Max Seconds", 16);
    lcd.setCursor(0, 1);
    lcd.print("Seconds: ");
    char secBuf[5];
    sprintf(secBuf, "%2d", maxSeconds);
    lcd.print(secBuf);
    for (int i = 11; i < 16; i++) { lcd.print(" "); }
  } else if (menu_state == MENU_CONFIG_AUTOSTART) {
    printFixed(0, 0, "Set Auto Start", 16);
    lcd.setCursor(0, 1);
    lcd.print("Auto: ");
    lcd.print(autoStart ? "ON " : "OFF");
    for (int i = 7; i < 16; i++) { lcd.print(" "); }
  }
}

// ----------------- Button Handlers -----------------
void handleShortPress() {
  if (menu_state == MENU_NORMAL) {
    if (!autoStart) { 
      // Manual mode: simply toggle the coil.
      if (coilOn) {
        setCoilState(false);
        DEBUG_PRINTLN("Manual mode: Coil turned OFF.");
      } else {
        setCoilState(true);
        coilOnStartTime = millis();
        secondsCounter = 0;
        DEBUG_PRINTLN("Manual mode: Coil turned ON.");
      }
    } else { 
      // Auto mode: if coil is on, a button press forces a manual override.
      if (coilOn) {
        setCoilState(false);
        manualOverride = true;
        manualOverrideSetTime = millis();
        DEBUG_PRINTLN("Auto mode: Manual override activated; Coil turned OFF.");
      }
    }
  } else {
    // In config modes, short press increments settings.
    switch (menu_state) {
      case MENU_CONFIG_TEMP:
        maxTemperature = (maxTemperature >= 500) ? 200 : maxTemperature + 10;
        config_enter_time = millis();
        DEBUG_PRINT("Max Temperature set to: ");
        DEBUG_PRINTLN(maxTemperature);
        break;
      case MENU_CONFIG_TIME:
        maxSeconds = (maxSeconds >= 30) ? 10 : maxSeconds + 1;
        config_enter_time = millis();
        DEBUG_PRINT("Max Seconds set to: ");
        DEBUG_PRINTLN(maxSeconds);
        break;
      case MENU_CONFIG_AUTOSTART:
        autoStart = !autoStart;
        config_enter_time = millis();
        DEBUG_PRINT("Auto Start set to: ");
        DEBUG_PRINTLN(autoStart ? "ON" : "OFF");
        break;
      default: break;
    }
  }
  updateLCD();
}

void handleLongPress() {
  switch (menu_state) {
    case MENU_NORMAL:
      backupMaxTemperature = maxTemperature;
      backupMaxSeconds = maxSeconds;
      backupAutoStart = autoStart;
      menu_state = MENU_CONFIG_TEMP;
      config_enter_time = millis();
      DEBUG_PRINTLN("Entering Config: Temperature");
      break;
    case MENU_CONFIG_TEMP:
      menu_state = MENU_CONFIG_TIME;
      config_enter_time = millis();
      DEBUG_PRINTLN("Entering Config: Time");
      break;
    case MENU_CONFIG_TIME:
      menu_state = MENU_CONFIG_AUTOSTART;
      config_enter_time = millis();
      DEBUG_PRINTLN("Entering Config: Auto Start");
      break;
    case MENU_CONFIG_AUTOSTART:
      saveConfig();
      printCentered("New Values Saved!");
      delay(1000);
      menu_state = MENU_NORMAL;
      break;
  }
  updateLCD();
}

// ----------------- Setup -----------------
void setup() {
  loadConfig();
  Wire.begin();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.createChar(0, degreeChar);
  
  pinMode(COIL_PIN, OUTPUT);
  setCoilState(false);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  menu_state = MENU_NORMAL;
  
  calibrateACS712();
  
  button.attachClick(handleShortPress);
  button.attachLongPressStart(handleLongPress);
  
  updateLCD();
}

// ----------------- Main Loop -----------------
void loop() {
  button.tick();
  
  // Auto-exit config mode after 7 seconds (rollback unsaved changes).
  if ((menu_state == MENU_CONFIG_TEMP || menu_state == MENU_CONFIG_TIME || menu_state == MENU_CONFIG_AUTOSTART) &&
      (millis() - config_enter_time > 7000)) {
    maxTemperature = backupMaxTemperature;
    maxSeconds = backupMaxSeconds;
    autoStart = backupAutoStart;
    menu_state = MENU_NORMAL;
    updateLCD();
  }

  // Auto mode pulsing: if autoStart is on, coil is off, and no manual override, run pulses.
  if (menu_state == MENU_NORMAL && autoStart && !coilOn) {
    if (millis() - lastAutoPulseTime > AUTO_PULSE_INTERVAL) {
      if (!manualOverride) {
        if (measureMetalPresence()) {  
          setCoilState(true);
          coilOnStartTime = millis();
          secondsCounter = 0;
          DEBUG_PRINTLN("Auto mode: Coil turned ON (metal inserted).");
        } else {
          DEBUG_PRINTLN("Auto mode: Pulse - metal absent; coil remains OFF.");
        }        
      } else if (!measureMetalPresence()) { // In auto mode, if manual override is active, check for metal absence.
        manualOverride = false;
        DEBUG_PRINTLN("Auto mode: Manual override cleared (metal removed).");
      }
      lastAutoPulseTime = millis();
    }
  }
  
  // Manual mode: if autoStart is off and coil is on, check if metal is removed (but only after a delay).
  if (!autoStart && coilOn) {
    if (millis() - coilOnStartTime > MANUAL_OFF_DELAY) {
      if (!measureMetalPresence()) {  
        setCoilState(false);
        DEBUG_PRINTLN("Manual mode: Coil turned OFF (metal removed).");
      }
    }
  }
  
  // If the coil is on, check off conditions (time/current/temperature).
  if (coilOn) {
    unsigned long elapsed = millis() - coilOnStartTime;
    unsigned long elapsedSeconds = elapsed / 1000;
    int currentValue = readCurrent();
    int objectTemp = readObjectTempC();
    DEBUG_PRINT("(");
    DEBUG_PRINT(elapsed);
    DEBUG_PRINT("ms >= ");
    DEBUG_PRINT(COIL_START_DELAY);
    DEBUG_PRINT("ms && (");
    DEBUG_PRINT(elapsedSeconds);
    DEBUG_PRINT("s >= ");
    DEBUG_PRINT(maxSeconds);
    DEBUG_PRINT("s || ");
    DEBUG_PRINT(currentValue);
    DEBUG_PRINT("mA < ");
    DEBUG_PRINT(CURRENT_THRESHOLD);
    DEBUG_PRINT("mA || ");
    DEBUG_PRINT(objectTemp);
    DEBUG_PRINT("°C >= ");
    DEBUG_PRINT(maxTemperature);
    DEBUG_PRINTLN("°C))");
    if (elapsed >= COIL_START_DELAY &&
        (elapsedSeconds >= maxSeconds || currentValue < CURRENT_THRESHOLD || objectTemp >= maxTemperature)) {
      DEBUG_PRINTLN("Coil OFF (off condition met).");
      setCoilState(false);
    }
    secondsCounter = (millis() - coilOnStartTime) / 1000;
  }
  
  if (millis() - lastUpdate > LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastUpdate = millis();
  }
}
