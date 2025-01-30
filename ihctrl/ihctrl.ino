#include <TinyWireM.h>
#include <TinyLiquidCrystal_I2C.h>
#include "EEPROM.h"

// ----------------- Config -----------------
#define MLX90614_ADDR   0x5A
#define MLX90614_OBJ_REG    0x07
#define EEPROM_TEMPERATURE_ADDR   0  // maxTemperature address
#define EEPROM_SECONDS_ADDR   2  // maxSeconds address

TinyLiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t BUTTON_PIN = 3;
const uint8_t COIL_PIN = 1;//4;
const uint8_t LED_PIN = 5;

enum { MENU_NORMAL, MENU_CONFIG } menu_state;
uint8_t menuIndex;
int maxSeconds, maxTemperature;
bool coilOn;
int secondsCounter;

// ----------------- EEPROM Helpers -----------------
void loadConfig() {
  EEPROM.get(EEPROM_TEMPERATURE_ADDR, maxTemperature);
  EEPROM.get(EEPROM_SECONDS_ADDR, maxSeconds);
  if(maxTemperature < 50) maxTemperature = 150;  // Set defaults if invalid
  if(maxSeconds < 10) maxSeconds = 30;
}

void saveConfig() {
  int storedTemp, storedSeconds;
  EEPROM.get(EEPROM_TEMPERATURE_ADDR, storedTemp);
  EEPROM.get(EEPROM_SECONDS_ADDR, storedSeconds);
  if (storedTemp != maxTemperature) EEPROM.put(EEPROM_TEMPERATURE_ADDR, maxTemperature);
  if (storedSeconds != maxSeconds) EEPROM.put(EEPROM_SECONDS_ADDR, maxSeconds);
}

// ----------------- Optimized LCD -----------------
void updateLCD() {
  static uint8_t last_state = 255;
  static int last_temp, last_counter;
  static int last_maxSeconds = 0, last_maxTemperature = 0;
  
  // Only update changed parts
  if(menu_state != last_state) {
    lcd.clear();
    last_state = menu_state;
    lcd.setCursor(0, 0);
    lcd.print(F("Coil:"));
    // Force a temperature redraw next time we’re in MENU_NORMAL
    last_temp = -9999; // Some number that guarantees abs(t - last_temp) > 1    
  }

  // Line 1: Coil status + counter
  lcd.setCursor(5, 0);
  lcd.print(coilOn ? F("ON ") : F("OFF        "));
  if (!coilOn) secondsCounter = 0; // make counter 0
  if(secondsCounter != last_counter) {
    lcd.setCursor(9, 0);
    lcd.print(secondsCounter);
    lcd.print("/");
    lcd.print(maxSeconds);
    lcd.print(F("s"));

    last_counter = secondsCounter;
  }

  // Line 2: Dynamic content
  lcd.setCursor(0, 1);
  if(menu_state == MENU_NORMAL) {
    int t = readObjectTempC();
    if(abs(t - last_temp) > 1) {  // Only update if temp changed     
      lcd.print(t);
      lcd.print(F("\337C"));
      lcd.print(F(" Max:"));
      lcd.print(maxTemperature);
      lcd.print(F("\337C"));
      last_temp = t;
    }
  } 
  else {
    lcd.setCursor(0, 0);
    lcd.print(menuIndex ? F("Set Max Seconds") : F("Set Max Temp"));
    lcd.setCursor(0, 1);
    lcd.print(menuIndex ? F("Seconds:        ") : F("Temp\337C:        "));
    lcd.setCursor(9, 1);
    lcd.print(menuIndex ? maxSeconds : maxTemperature);
  }
}

// ----------------- Simplified Temp Read -----------------
int readObjectTempC() {
  // TinyWireM.beginTransmission(MLX90614_ADDR);
  // TinyWireM.write(MLX90614_OBJ_REG);
  // TinyWireM.endTransmission(false);
  // if(TinyWireM.requestFrom(MLX90614_ADDR, 3) == 3) {
  //   uint16_t raw = TinyWireM.read() | (TinyWireM.read() << 8);
  //   return (raw * 2) / 100 - 273;  // Combined calculation
  // }
  // return -127;
  
  // simulate temperature increase
  static int someTestTemp = 150;
  if (coilOn) {
    if (someTestTemp < 510) someTestTemp++;
  } else {
    if (someTestTemp > 150) someTestTemp--;  // Simulate cooling
  }
  return someTestTemp;   
}

// ----------------- Setup -----------------
void setup() {
  loadConfig();
  TinyWireM.begin();
  lcd.begin(16, 2);
  lcd.backlight();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(COIL_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  menu_state = MENU_NORMAL;
}

void loop() {
  static uint32_t press_start = 0;
  static bool was_held = false;
  static uint32_t last_sec = 0;
  static uint32_t lastButtonPress = 0; // Debounce timer

  static int originalTemp;
  static int originalTime;
  static uint32_t config_enter_time = 0;

  bool btn = digitalRead(BUTTON_PIN);

  // Debounce check (ignore rapid button presses)
  if (millis() - lastButtonPress < 50) {
    return;
  }

  if (!btn) { // Button pressed
    lastButtonPress = millis();  // Update debounce timer
    if (press_start == 0) {
      press_start = millis();
      was_held = false;
    }
    if ((millis() - press_start > 2000) && !was_held && !coilOn) {
      was_held = true;
      if (menu_state == MENU_NORMAL) {
        originalTemp = maxTemperature;
        originalTime = maxSeconds;
        menu_state = MENU_CONFIG;
        menuIndex = 0;
        config_enter_time = millis();
      } else if (menu_state == MENU_CONFIG) {
        if (menuIndex == 0) {
          menuIndex = 1;
          config_enter_time = millis();
        } else {
          saveConfig();
          menu_state = MENU_NORMAL;
          menuIndex = 0;
        }
      }
      press_start = millis();
    }
  } else {
    if (press_start > 0 && !was_held && (millis() - press_start > 50)) {
      if (menu_state == MENU_NORMAL) {
        coilOn = !coilOn;
        digitalWrite(COIL_PIN, coilOn);
        digitalWrite(LED_PIN, coilOn);
      } else if (menu_state == MENU_CONFIG) {
        if (menuIndex == 0) {
          maxTemperature += 10;
          if (maxTemperature > 500) maxTemperature = 200;
        } else {
          maxSeconds++;
          if (maxSeconds > 30) maxSeconds = 10;
        }
        config_enter_time = millis();
      }
    }
    press_start = 0;
  }

  if (menu_state == MENU_CONFIG) {
    if (millis() - config_enter_time > 7000) {
      maxTemperature = originalTemp;
      maxSeconds = originalTime;
      menu_state = MENU_NORMAL;
      menuIndex = 0;
    }
  }

  if (coilOn && (millis() - last_sec > 1000)) {
    last_sec = millis();
    if (++secondsCounter >= maxSeconds || readObjectTempC() >= maxTemperature) {
      coilOn = false;
      digitalWrite(COIL_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
    }
  }

  updateLCD();
  delay(150);
}
