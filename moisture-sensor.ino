/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 */

// Enable debug prints
//#define MY_DEBUG

// Enable and select radio type attached 
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
//#define MY_RS485

// Pins CE, CSN for ARDUINO
#define MY_RF24_CE_PIN    9
#define MY_RF24_CS_PIN   10

// Pin for moisture
#define MOISTURE_PIN A0

// Pin for reset
#define RESET_PIN 2

// Device powered by battery
#define BATTERY_ON

#include <Wire.h>
#include <SPI.h>
#include <MySensors.h>  
#include "SparkFunMAX17043.h"

//
// Cycle
//
// Average cycle duration / sleep time duration in reality (in milliseconds)
// For each cycle, if probe value change, she's send to gateway
#define CYCLE_DURATION 1200000 // 20 min
// Force sending an update of the probe value after n cycle ; even if probe value not changed since N cycle
// So, gateway receive minimum of every CYCLE_DURATION * FORCE_SEND_AFTER_N_CYCLE ms a probe updated value
#define FORCE_SEND_AFTER_N_CYCLE 3 // 20 min * 3 = 1h
// Reset software after number of cycle (1 cycle = CYCLE_DURATION)
#define RESET_AFTER_N_CYCLE 36 // 1h/20min * 12 = 12h

//
// Battery
//
// Number of consecutive "low battery" values after which to consider this information as reliable.
#define SEND_BATTERY_LOW_AFTER_N_CYCLE 3
// Limit (in %) to consider battery as "low power"
// Under this value, node goind to sleep permanently
#define BATTERY_LOW_LIMIT 10 // 10%

//
// Probe value
//
// Setting up format for reading soil sensors
#define NUM_READS (int)10    // Number of sensor reads for filtering

//
// Node
//
// Node childs id (local to node)
#define CHILD_VALUE_ID 0
// Message which contain probe value
MyMessage messageValue(CHILD_VALUE_ID, V_LEVEL);
// Battery gauge
MAX17043 batteryGauge;


int lastProbeValue;
uint16_t cycleCpt;
uint16_t batteryLowCpt;
uint16_t cycleCptReset;
bool probeValueReceived = true;

/**
 * Init node to gateway
 */
void presentation()  
{ 
  // Send the sketch version information to the gateway
  sendSketchInfo("Moisture", "1.4.2");

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_VALUE_ID, S_MOISTURE); // Probe value
}

/**
 * Setup node
 */
void setup()
{
  // Keep reset pin HIGH (need to be the first command to prevent infinite reset!)
  digitalWrite(RESET_PIN, HIGH);

  // Configure pin
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(RESET_PIN, OUTPUT);

  // Battery
  initBattery();

  // Force send at startup
  cycleCpt = FORCE_SEND_AFTER_N_CYCLE;

  #ifdef MY_DEBUG
  Serial.println("Setup end.");
  #endif
}

/**
 * Loop
 */
void loop()      
{  
  // Probe value: If value send to server, allow to send others values
  bool allowSend = processMoisture();
  
  // Battery
  processBattery(allowSend);

  // Reset
  processReset();

  #ifdef MY_DEBUG
  Serial.println("Go to sleep!");
  #endif
  // Sleep for a while to save energy
  sleep(CYCLE_DURATION); 
}

/**
 * Receive message
 */
void receive(const MyMessage &message)
{
  // ACK to confirm that probe value right received
  if (message.sensor == CHILD_VALUE_ID && message.isAck()) {
    // Ok, remove flag, value has been successfully received by server
    probeValueReceived = true;
  }
}

/**
 * Restart
 */
void restart()
{
  doRestart(true);
}
/**
 * Do restart
 * 
 * @param bool software Use software method. Hardware method is used by default
 *                      Note that to use hardware restart, you need to link RESET_PIN to RST
 */
void doRestart(bool software)
{
  #ifdef MY_DEBUG
  Serial.println("Restart node");
  #endif

  cycleCptReset = 0;

  // Restart software
  if (software) {
    /*
    wdt_enable(WDTO_15MS);
    while(1) {}
    */
    asm volatile ("  jmp 0");
  }
  // Restart hardware
  else {
    digitalWrite(RESET_PIN, LOW);
  }
}

/**
 * Read moisture and send data
 * 
 * @return bool True if probe value has been send
 */
bool processMoisture()
{
  bool r = false;
  
  // Get probe value (in %, multiplicated by 100)
  int probeValueGross = analogRead(MOISTURE_PIN);
  int probeValue = (int)round(100.0 * (float)probeValueGross / 1024.0);
  delay(1000);
  #ifdef MY_DEBUG
  Serial.print("Probe value gross: ");
  Serial.println(probeValueGross);
  Serial.print("Probe value: ");
  Serial.print(probeValue);
  Serial.println("%");
  #endif
  if (probeValue != lastProbeValue || cycleCpt == FORCE_SEND_AFTER_N_CYCLE || !probeValueReceived) {
    // Send probe value if it changed since the last measurement or if we didn't send an update for n times
    lastProbeValue = probeValue;
    // Reset no updates counter
    cycleCpt = 0;
    r = true;
    // Before send, flag to indicate that server confirmation need to be received
    probeValueReceived = false;
    // Send value to server
    #ifdef MY_DEBUG
    Serial.println("Send value to server");
    #endif
    send(messageValue.set(probeValue), true);
    // Wait for server response
    wait(1000); // 1s
  } else {
    // Increase no update counter if the probe value stayed the same
    cycleCpt++;
  }

  return r;
}

/**
 * Init battery
 */
void initBattery()
{
  #ifdef BATTERY_ON
  //
  // Battery gauge
  //
  #ifdef MY_DEBUG
  Serial.println("Battery gauge begin");
  #endif
  batteryGauge.begin();
  #ifdef MY_DEBUG
  Serial.println("Battery gauge quick start");
  #endif
  batteryGauge.quickStart();
  #endif
}

/**
 * Process to read battery and send level
 * 
 * If battery level lower than limit, inter into deep sleep mode!
 * 
 * @param bool allowSend Allow to send battery level to server
 */
void processBattery(bool allowSend)
{
  #ifdef BATTERY_ON
  
  float batteryLevel = batteryGauge.getSOC();
  #ifdef MY_DEBUG
  Serial.print("Battery level: ");
  Serial.print(batteryLevel);
  Serial.println("%");
  #endif

  // Send battery level to server
  if (allowSend) {
    sendBatteryLevel(batteryLevel);
  }
  
  // If lower that battery low limit, waiting for n consecutive check
  if (batteryLevel < BATTERY_LOW_LIMIT) {
    batteryLowCpt++;
  // Else, reset cpt (wrong check)
  } else {
    batteryLowCpt = 0;
  }
  
  // If battery low confirmed
  if (batteryLowCpt >= SEND_BATTERY_LOW_AFTER_N_CYCLE) {
    batteryLowCpt = 0;
    // Consider battery level as empty!
    sendBatteryLevel(0);
    // Sleep infinity
    #ifdef MY_DEBUG
    Serial.println("Low battery level! Enter into deep sleep mode!");
    #endif
    sleep(1, CHANGE, 0); // Interrupt with pin D3
  }

  #endif
}

/**
 * Process to restart node after n cycle
 */
void processReset()
{
  // Restart every N cycle
  cycleCptReset++;
  if (cycleCptReset >= RESET_AFTER_N_CYCLE) {    
    restart();
    // Below restart(): Never executed
  }
}

