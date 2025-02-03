/*
  =======================================================================
  Arduino Coil Control with Metal Detection and Configurable Menu
  =======================================================================
  
  Description:
  ------------
  This sketch controls a coil (e.g., in an induction heating system) by 
  monitoring current (via an ACS712 sensor) and metal presence. It offers two 
  modes—Manual and Auto—selectable via a configuration menu displayed on a 
  16×2 LCD. A single button (using the OneButton library) navigates the menu 
  and toggles the coil.
  
  Operating Modes:
  ----------------
  Manual Mode (Auto-Start OFF):
    - The coil remains off until toggled.
    - Metal insertion does not auto–start the coil.
    - A manual toggle turns the coil ON/OFF.
    - If the coil is ON and metal is removed, the coil shuts off.
  
  Auto Mode (Auto-Start ON):
    - The system periodically pulses the coil to check for metal.
    - Each pulse measures current via the ACS712 sensor.
    - If current exceeds a set threshold, the coil turns ON.
    - The coil remains on while metal is present.
    - A button press while running forces a manual override.
  
  Menu:
  -----
  The LCD menu lets you configure:
#ifdef USE_TEMPERATURE_SENSOR
    • Maximum Temperature (if a MAX6675 sensor is installed)
#endif
    • Maximum On–Time (in seconds)
    • Auto–Start Mode (ON/OFF)
  Long–pressing the button cycles through menu screens. Settings are stored 
  in EEPROM.
  
  Coil Off Conditions:
  --------------------
  The coil is turned off when any of these occur:
    - Maximum on–time exceeded
    - Current drops below threshold
#ifdef USE_TEMPERATURE_SENSOR
    - Temperature exceeds the configured maximum
#endif
  
  Manual Override:
  ----------------
  In Auto Mode, a button press while the coil is on forces an override until metal 
  is removed.
  
  Author: Ioan Ghip
  Date: February 3rd, 2025
  =======================================================================
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <OneButton.h>
#include <max6675.h>

#define DEBUG  // Uncomment to enable Serial debugging

#ifdef DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Uncomment the following line to enable temperature sensor features
// #define USE_TEMPERATURE_SENSOR

// ======================== Meaningful Constants ==========================
// --- Timing Constants (in milliseconds) ---
const unsigned long COIL_START_DELAY_MS      = 100;   // Delay for coil to settle
const unsigned long LCD_UPDATE_INTERVAL_MS   = 500;   // LCD update interval
const unsigned long AUTO_PULSE_INTERVAL_MS   = 1000;  // Interval between auto pulses
const unsigned long AUTO_PULSE_DURATION_MS   = 10;    // Duration of auto pulse
const unsigned long MANUAL_OFF_DELAY_MS      = 50;    // Delay before turning coil off (manual mode)
const unsigned long MENU_CONFIG_TIMEOUT_MS   = 7000;  // Timeout to exit menu without saving

// --- ACS712 & Current Sensing Constants ---
const float ACS712_SENSITIVITY_V_PER_A       = 0.1;   // Sensitivity (V/A)
const int   CURRENT_THRESHOLD_mA           = 7000;  // Threshold current (mA)
const int   CURRENT_FILTER_SAMPLES         = 5;     // Samples for current averaging

// --- Calibration Constants ---
const int NUM_CALIBRATION_SAMPLES            = 20;    // Number of samples for calibration
const int CALIBRATION_SAMPLE_DELAY_MS        = 10;    // Delay between calibration samples

// ======================== EEPROM Addresses ==============================
#ifdef USE_TEMPERATURE_SENSOR
const int EEPROM_TEMPERATURE_ADDR = 0;  // 2 bytes for maxTemperature
#endif
const int EEPROM_SECONDS_ADDR     = 2;  // 2 bytes for maxSeconds
const int EEPROM_AUTOSTART_ADDR   = 4;  // 1 byte for autoStart flag

// ======================== Pin Assignments ===============================
const uint8_t ACS712_PIN   = A0;        // ACS712 sensor input
const uint8_t BUTTON_PIN   = 3;         // Button (active LOW)
const uint8_t COIL_PIN     = 5;         // Coil control
const uint8_t LED_PIN      = 6;         // LED indicator

#ifdef USE_TEMPERATURE_SENSOR
const uint8_t MAX6675_SO_PIN = 4;       // MAX6675 Serial Data Out
const uint8_t MAX6675_CS_PIN = 7;       // MAX6675 Chip Select
const uint8_t MAX6675_CLK_PIN = 8;      // MAX6675 Clock
#endif

// ======================== LCD Settings ==================================
const uint8_t LCD_I2C_ADDRESS = 0x27;     // I2C address for LCD
const uint8_t LCD_COLUMNS     = 16;       // Columns on LCD
const uint8_t LCD_ROWS        = 2;        // Rows on LCD

// ======================== Custom Characters =============================
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

// ======================== Global Objects =================================
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
OneButton button(BUTTON_PIN, true);  // active LOW

#ifdef USE_TEMPERATURE_SENSOR
// Create MAX6675 instance (order: SCLK, CS, SO)
MAX6675 thermocouple(MAX6675_CLK_PIN, MAX6675_CS_PIN, MAX6675_SO_PIN);
#endif

// ======================== Menu States ====================================
enum MenuState {
  MENU_NORMAL,
#ifdef USE_TEMPERATURE_SENSOR
  MENU_CONFIG_TEMP,
#endif
  MENU_CONFIG_TIME,
  MENU_CONFIG_AUTOSTART
};
MenuState menu_state = MENU_NORMAL;

// ======================== Global Variables ===============================
#ifdef USE_TEMPERATURE_SENSOR
int maxTemperature;
#endif
int maxSeconds;
bool coilOn = false;
bool autoStart = false;
int secondsCounter = 0;
unsigned long coilOnStartTime = 0;
unsigned long config_enter_time = 0;
unsigned long lastUpdate = 0;
unsigned long lastAutoPulseTime = 0;
bool manualOverride = false;
unsigned long manualOverrideSetTime = 0;

// Backup variables (for menu timeout rollback)
#ifdef USE_TEMPERATURE_SENSOR
int backupMaxTemperature;
#endif
int backupMaxSeconds;
bool backupAutoStart;

// Global calibrated offset voltage for the ACS712 sensor.
float offsetVoltage = 2.5;

// ======================== Function Prototypes ============================
void updateLCD();
void handleShortPress();
void handleLongPress();
void calibrateACS712();
int readCurrent();
#ifdef USE_TEMPERATURE_SENSOR
int readObjectTempC();
#else
int readObjectTempC();
#endif
bool measureMetalPresence();
void loadConfig();
void saveConfig();
void setCoilState(bool state);

// ======================== Function Definitions ===========================

// Set the coil and LED state.
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

// Load configuration from EEPROM.
void loadConfig() {
#ifdef USE_TEMPERATURE_SENSOR
  EEPROM.get(EEPROM_TEMPERATURE_ADDR, maxTemperature);
#else
  int dummyTemp = 300;
#endif
  EEPROM.get(EEPROM_SECONDS_ADDR, maxSeconds);
  byte autoStartByte = EEPROM.read(EEPROM_AUTOSTART_ADDR);
  if (autoStartByte != 0 && autoStartByte != 1)
    autoStart = false;
  else
    autoStart = (autoStartByte == 1);
#ifdef USE_TEMPERATURE_SENSOR
  if (maxTemperature < 50 || maxTemperature > 600) maxTemperature = 300;
#endif
  if (maxSeconds < 1 || maxSeconds > 30) maxSeconds = 10;
}
 
// Save configuration to EEPROM.
void saveConfig() {
#ifdef USE_TEMPERATURE_SENSOR
  int storedTemp;
  EEPROM.get(EEPROM_TEMPERATURE_ADDR, storedTemp);
  if (storedTemp != maxTemperature) {
    EEPROM.put(EEPROM_TEMPERATURE_ADDR, maxTemperature);
    DEBUG_PRINT("Saved maxTemperature: ");
    DEBUG_PRINTLN(maxTemperature);
  }
#endif
  int storedSeconds;
  EEPROM.get(EEPROM_SECONDS_ADDR, storedSeconds);
  if (storedSeconds != maxSeconds) {
    EEPROM.put(EEPROM_SECONDS_ADDR, maxSeconds);
    DEBUG_PRINT("Saved maxSeconds: ");
    DEBUG_PRINTLN(maxSeconds);
  }
  byte storedAutoStartByte = EEPROM.read(EEPROM_AUTOSTART_ADDR);
  bool storedAutoStart = (storedAutoStartByte == 1);
  if (storedAutoStart != autoStart) {
    byte autoByte = autoStart ? 1 : 0;
    EEPROM.put(EEPROM_AUTOSTART_ADDR, autoByte);
    DEBUG_PRINT("Saved AutoStart: ");
    DEBUG_PRINTLN(autoStart ? "ON" : "OFF");
  }
}

// Calibrate the ACS712 sensor.
void calibrateACS712() {
  digitalWrite(COIL_PIN, LOW);
  delay(50);
  long sum = 0;
  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    sum += analogRead(ACS712_PIN);
    delay(CALIBRATION_SAMPLE_DELAY_MS);
  }
  float avgRaw = (float)sum / NUM_CALIBRATION_SAMPLES;
  offsetVoltage = (avgRaw * 5.0) / 1023.0;
  DEBUG_PRINT("Calibrated ACS712 Offset Voltage: ");
  DEBUG_PRINTLN(offsetVoltage);
}

// Read current from the ACS712 sensor (in milliamps).
int readCurrent() {
  long sum = 0;
  for (int i = 0; i < CURRENT_FILTER_SAMPLES; i++) {
    sum += analogRead(ACS712_PIN);
    delay(2);
  }
  float avgRaw = (float)sum / CURRENT_FILTER_SAMPLES;
  float voltage = (avgRaw * 5.0) / 1023.0;
  float currentAmps = (voltage - offsetVoltage) / ACS712_SENSITIVITY_V_PER_A;
  return abs((int)(currentAmps * 1000));
}

#ifdef USE_TEMPERATURE_SENSOR
// Read the object temperature in Celsius.
int readObjectTempC() {
  double tempC = thermocouple.readCelsius();
  return (int)tempC;
}
#else
int readObjectTempC() {
  return -1;  // No sensor installed.
}
#endif

// Pulse the coil briefly and check if current exceeds the threshold.
bool measureMetalPresence() {
  bool prevState = coilOn;
  if (!prevState) {
    setCoilState(true);
  }
  unsigned long pulseStart = millis();
  int pulseCurrent = 0;
  while (millis() - pulseStart < AUTO_PULSE_DURATION_MS) {
    pulseCurrent = readCurrent();
    if (pulseCurrent >= CURRENT_THRESHOLD_mA) {
      if (!prevState) {
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

// ----------------------- LCD Update -----------------------
// This function reproduces the original layout:
//  • Line 0: "Coil:" (cols 0–4), coil state (cols 5–8),
//    temperature reading (if enabled) starting at col 9,
//    then always prints "A:ON " or "A:OFF" starting at col 11.
//  • Line 1: Time information.
void updateLCD() {
  if (menu_state == MENU_NORMAL) {
    // Line 0
    lcd.setCursor(0, 0);
    lcd.print("Coil:");
    lcd.setCursor(5, 0);
    if (coilOn)
      lcd.print("ON  ");
    else
      lcd.print("OFF ");
#ifdef USE_TEMPERATURE_SENSOR
    if (coilOn) {
      lcd.setCursor(9, 0);
      char tempBuf[5];
      sprintf(tempBuf, "%3d", readObjectTempC());
      lcd.print(tempBuf);
      lcd.write(byte(0));  // degree symbol
    } else {
      lcd.setCursor(9, 0);
      lcd.print("        ");  // clear ~8 characters
    }
#else
    lcd.setCursor(9, 0);
    lcd.print("        ");
#endif
    // Always print auto mode info at fixed column 11.
    lcd.setCursor(11, 0);
    if (autoStart)
      lcd.print("A:ON ");
    else
      lcd.print("A:OFF");
      
    // Line 1
    lcd.setCursor(0, 1);
    lcd.print("Time:");
    lcd.setCursor(5, 1);
    char timeBuf[12];
    sprintf(timeBuf, "%02d/%02ds", secondsCounter, maxSeconds);
    lcd.print(timeBuf);
    
  }
#ifdef USE_TEMPERATURE_SENSOR
  else if (menu_state == MENU_CONFIG_TEMP) {
    lcd.setCursor(0, 0);
    lcd.print("Set Max Temp     ");  // exactly 16 characters
    lcd.setCursor(0, 1);
    lcd.print("Temp(");
    lcd.write(byte(0)); // degree symbol
    lcd.print("C): ");
    char valBuf[5];
    sprintf(valBuf, "%3d", maxTemperature);
    lcd.print(valBuf);
    int printed = 7 + strlen(valBuf);  // "Temp(" + "C): " = 7 characters + number
    for (int i = printed; i < LCD_COLUMNS; i++) {
      lcd.print(" ");
    }
  }
#endif
  else if (menu_state == MENU_CONFIG_TIME) {
    lcd.setCursor(0, 0);
    lcd.print("Set Max Seconds   ");  // 16 characters
    lcd.setCursor(0, 1);
    lcd.print("Seconds: ");
    char secBuf[5];
    sprintf(secBuf, "%2d", maxSeconds);
    lcd.print(secBuf);
    int printed = 9 + strlen(secBuf);  // "Seconds: " is 9 characters
    for (int i = printed; i < LCD_COLUMNS; i++) {
      lcd.print(" ");
    }
  }
  else if (menu_state == MENU_CONFIG_AUTOSTART) {
    lcd.setCursor(0, 0);
    lcd.print("Set Auto Start    ");  // 16 characters
    lcd.setCursor(0, 1);
    lcd.print("Auto: ");
    if (autoStart)
      lcd.print("ON ");
    else
      lcd.print("OFF");
    int printed = 6 + 3;  // "Auto: " (6 chars) + "ON " or "OFF" (3 chars)
    for (int i = printed; i < LCD_COLUMNS; i++) {
      lcd.print(" ");
    }
  }
}

void handleShortPress() {
  if (menu_state == MENU_NORMAL) {
    if (!autoStart) { 
      // Manual mode: toggle the coil.
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
      // Auto mode: a press while running forces a manual override.
      if (coilOn) {
        setCoilState(false);
        manualOverride = true;
        manualOverrideSetTime = millis();
        DEBUG_PRINTLN("Auto mode: Manual override activated; Coil turned OFF.");
      }
    }
  } else {
    switch (menu_state) {
#ifdef USE_TEMPERATURE_SENSOR
      case MENU_CONFIG_TEMP:
        maxTemperature = (maxTemperature >= 500) ? 200 : maxTemperature + 10;
        config_enter_time = millis();
        DEBUG_PRINT("Max Temperature set to: ");
        DEBUG_PRINTLN(maxTemperature);
        break;
#endif
      case MENU_CONFIG_TIME:
        maxSeconds = (maxSeconds >= 30) ? 1 : maxSeconds + 1;
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
      default:
        break;
    }
  }
  updateLCD();
}

void handleLongPress() {
#ifdef USE_TEMPERATURE_SENSOR
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
      lcd.setCursor(0, 0);
      lcd.print("New Values Saved");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      delay(1000);
      menu_state = MENU_NORMAL;
      break;
  }
#else
  switch (menu_state) {
    case MENU_NORMAL:
      backupMaxSeconds = maxSeconds;
      backupAutoStart = autoStart;
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
      lcd.setCursor(0, 0);
      lcd.print("New Values Saved");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      delay(1000);
      menu_state = MENU_NORMAL;
      break;
  }
#endif
  updateLCD();
}

void setup() {
  loadConfig();
  Wire.begin();
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.backlight();
#ifdef USE_TEMPERATURE_SENSOR
  lcd.createChar(0, degreeChar);
#endif
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

void loop() {
  button.tick();
  
  // Auto–exit configuration mode after timeout (rollback unsaved changes).
  if ((menu_state == MENU_CONFIG_TIME
#ifdef USE_TEMPERATURE_SENSOR
       || menu_state == MENU_CONFIG_TEMP
#endif
       || menu_state == MENU_CONFIG_AUTOSTART) &&
      (millis() - config_enter_time > MENU_CONFIG_TIMEOUT_MS)) {
#ifdef USE_TEMPERATURE_SENSOR
    maxTemperature = backupMaxTemperature;
#endif
    maxSeconds = backupMaxSeconds;
    autoStart = backupAutoStart;
    menu_state = MENU_NORMAL;
    updateLCD();
  }
  
  // Auto mode pulsing: in Auto Mode, if the coil is off and no manual override.
  if (menu_state == MENU_NORMAL && autoStart && !coilOn) {
    if (millis() - lastAutoPulseTime > AUTO_PULSE_INTERVAL_MS) {
      if (!manualOverride) {
        if (measureMetalPresence()) {
          setCoilState(true);
          coilOnStartTime = millis();
          secondsCounter = 0;
          DEBUG_PRINTLN("Auto mode: Coil turned ON (metal inserted).");
        } else {
          DEBUG_PRINTLN("Auto mode: Pulse - metal absent; coil remains OFF.");
        }
      } else if (!measureMetalPresence()) {
        manualOverride = false;
        DEBUG_PRINTLN("Auto mode: Manual override cleared (metal removed).");
      }
      lastAutoPulseTime = millis();
    }
  }
  
  // In Manual Mode, if the coil is on, check for metal removal.
  if (!autoStart && coilOn) {
    if (millis() - coilOnStartTime > MANUAL_OFF_DELAY_MS) {
      if (!measureMetalPresence()) {
        setCoilState(false);
        DEBUG_PRINTLN("Manual mode: Coil turned OFF (metal removed).");
      }
    }
  }
  
  // While the coil is on, check off conditions.
  if (coilOn) {
    unsigned long elapsed = millis() - coilOnStartTime;
    unsigned long elapsedSeconds = elapsed / 1000;
    int currentValue = readCurrent();
#ifdef USE_TEMPERATURE_SENSOR
    int objectTemp = readObjectTempC();
#endif
    DEBUG_PRINT("(");
    DEBUG_PRINT(elapsed);
    DEBUG_PRINT("ms >= ");
    DEBUG_PRINT(COIL_START_DELAY_MS);
    DEBUG_PRINT("ms && (");
    DEBUG_PRINT(elapsedSeconds);
    DEBUG_PRINT("s >= ");
    DEBUG_PRINT(maxSeconds);
    DEBUG_PRINT("s || ");
    DEBUG_PRINT(currentValue);
    DEBUG_PRINT("mA < ");
    DEBUG_PRINT(CURRENT_THRESHOLD_mA);
    DEBUG_PRINT("mA || ");
#ifdef USE_TEMPERATURE_SENSOR
    DEBUG_PRINT(objectTemp);
    DEBUG_PRINT("°C >= ");
    DEBUG_PRINT(maxTemperature);
    DEBUG_PRINTLN("°C))");
#else
    DEBUG_PRINTLN(")");
#endif
#ifdef USE_TEMPERATURE_SENSOR
    if (elapsed >= COIL_START_DELAY_MS &&
        (elapsedSeconds >= maxSeconds || currentValue < CURRENT_THRESHOLD_mA || objectTemp >= maxTemperature)) {
      DEBUG_PRINTLN("Coil OFF (off condition met).");
      setCoilState(false);
      manualOverride = true;
    }
#else
    if (elapsed >= COIL_START_DELAY_MS &&
        (elapsedSeconds >= maxSeconds || currentValue < CURRENT_THRESHOLD_mA)) {
      DEBUG_PRINTLN("Coil OFF (off condition met).");
      setCoilState(false);
      manualOverride = true;
    }
#endif
    secondsCounter = (millis() - coilOnStartTime) / 1000;
  }
  
  if (millis() - lastUpdate > LCD_UPDATE_INTERVAL_MS) {
    updateLCD();
    lastUpdate = millis();
  }
}
