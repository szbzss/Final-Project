/*
 * Appendix A: ESP32 Hardware-Timer-Driven Polling Loop for 12-TFmini Array
 * Author: Zhanbo Sun
 * Description: Uses ESP32 Hardware Timer 0 to trigger a strict 10ms (100Hz) 
 * sequential polling loop via a 16-channel UART Multiplexer (CD74HC4067).
 */

#include <Arduino.h>
#include <TFMPlus.h>

// --- UART Multiplexer Pins (CD74HC4067) ---
// These pins control which TFmini is currently connected to Serial2
const int muxS0 = 13;
const int muxS1 = 12;
const int muxS2 = 14;
const int muxS3 = 27;

#define RXD2 16
#define TXD2 17

TFMPlus tfm;

// --- Hardware Timer Configuration ---
hw_timer_t * timer = NULL;
volatile bool triggerScan = false; // Flag set by the Interrupt Service Routine (ISR)

// Array to store the 12 distance measurements (in cm)
int16_t lidarDistances[12] = {0};

// --- Interrupt Service Routine (ISR) ---
// This function fires exactly every 10ms (100Hz)
void IRAM_ATTR onTimer() {
  triggerScan = true; 
}

// --- Function to Switch Multiplexer Channel ---
void setMuxChannel(int channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
  // Brief delay to allow the UART signal path to settle
  delayMicroseconds(50); 
}

void setup() {
  // Serial0: High-speed communication with ROS 2 Micro-ROS / Serial Node
  Serial.begin(115200); 
  
  // Initialize Multiplexer control pins
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);

  // Serial2: Shared communication line for all 12 TFmini sensors
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  tfm.begin(&Serial2);

  /* * Configure Hardware Timer 0
   * ESP32 base clock is 80MHz. Prescaler of 80 means 1 tick = 1 microsecond.
   */
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  
  // Set alarm to trigger every 10,000 microseconds (10 milliseconds = 100 Hz)
  timerAlarmWrite(timer, 10000, true); 
  timerAlarmEnable(timer);
  
  Serial.println("SYSTEM_READY");
}

void loop() {
  // Check if the hardware timer has triggered the 10ms flag
  if (triggerScan) {
    triggerScan = false; // Reset the flag immediately
    
    // Sequential polling loop for sensors 0 through 11
    for (int i = 0; i < 12; i++) {
      setMuxChannel(i);
      
      int16_t dist = 0, flux = 0, temp = 0;
      
      // Attempt to read data from the currently selected TFmini
      if (tfm.getData(dist, flux, temp)) {
        lidarDistances[i] = dist;
      } else {
        lidarDistances[i] = -1; // -1 indicates a sensor timeout or read error
      }
    }
    
    // --- Data Serialization for ROS 2 ---
    // Pack the 12 distance points into a single string frame
    Serial.print("SCAN:");
    for(int i = 0; i < 12; i++) {
      Serial.print(lidarDistances[i]);
      if(i < 11) {
        Serial.print(","); // Comma separator
      }
    }
    Serial.println(); // End of frame marker (\n)
  }
}