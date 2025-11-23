
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

#define INPUT_PIN 16               // pin to monitor
const char *FILE_NAME = "/log.csv";

volatile bool eventPending = false;
volatile uint8_t eventValue = 0;
volatile uint32_t eventTime = 0;

void IRAM_ATTR pinChangeISR() {
  eventValue = digitalRead(INPUT_PIN);
  eventTime  = millis();
  eventPending = true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(INPUT_PIN, INPUT_PULLUP);   // adjust if needed

  // --- SD init ---
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  Serial.println("Mounting SD...");
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    while (1) delay(1000);
  }
  Serial.println("SD ready.");

  // --- Create file + header if needed ---
  if (!SD.exists(FILE_NAME)) {
    Serial.println("Log file does not exist, creating with header...");
    File f = SD.open(FILE_NAME, FILE_WRITE);  // FILE_WRITE is fine here (create/truncate once)
    if (f) {
      f.println("millis,pin_value");
      f.close();
      Serial.println("Header written.");
    } else {
      Serial.println("Failed to create log file!");
    }
  } else {
    Serial.println("Log file exists, will append.");
  }

  // --- Attach interrupt on pin change ---
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), pinChangeISR, CHANGE);

  Serial.println("Interrupt-driven logger ready.");
}

void loop() {
  if (eventPending) {
    noInterrupts();
    uint32_t t = eventTime;
    uint8_t  v = eventValue;
    eventPending = false;
    interrupts();

    // Open with FILE_APPEND so we *add* lines, never overwrite
    File f = SD.open(FILE_NAME, FILE_APPEND);
    if (f) {
      f.print(t);
      f.print(',');
      f.println(v);
      f.close();   // close after each write to ensure it’s saved

      Serial.print("Logged: ");
      Serial.print(t);
      Serial.print(", ");
      Serial.println(v);
    } else {
      Serial.println("Failed to open log file for append!");
    }
  }

  delay(1);  // tiny yield, no busy wait
}

