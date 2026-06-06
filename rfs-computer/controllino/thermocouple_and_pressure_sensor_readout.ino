// This is a TEST script for the Controllino

#include <Wire.h>
#include "Seeed_MCP9600.h"
#include <Controllino.h>

// --- Thermocouple Setup ---
MCP9600 sensor_1;       // Default 0x60 address
MCP9600 sensor_2(0x67); // Soldered alternate address

// --- Pressure Sensor Setup ---
// Updated to the 40 Bar limit from your uploaded file
const float MAX_PRESSURE_BAR = 40.0; 

void setup() {
    Serial.begin(9600);
    delay(100); 
    Serial.println("==================================================");
    Serial.println("  Booting Unified Test Bench (2x TC + 1x Pressure) ");
    Serial.println("==================================================");

    // 1. Initialize the I2C bus for Thermocouples
    Wire.begin(); 
    
    if (sensor_1.init(THER_TYPE_K) == NO_ERROR) {
        Serial.println(">> TC Sensor 1 (0x60) initialized.");
    } else {
        Serial.println("SYSTEM HALT: TC Sensor 1 failed. Check Hub wiring.");
        while(1); 
    }

    if (sensor_2.init(THER_TYPE_K) == NO_ERROR) {
        Serial.println(">> TC Sensor 2 (0x67) initialized.");
    } else {
        Serial.println("SYSTEM HALT: TC Sensor 2 failed. Check 0x67 address.");
        while(1); 
    }

    // 2. Initialize the 24V loop for the Pressure Sensor
    pinMode(CONTROLLINO_D0, OUTPUT);
    digitalWrite(CONTROLLINO_D0, HIGH); 
    Serial.println(">> 24V Pressure Sensor Power (DO0) ENABLED.");
    
    // Give the 4-20mA loop 50ms to energize and stabilize
    delay(50);
}

void loop() {
    float temp1 = 0;
    float temp2 = 0;
    bool data_ready_1 = false;
    bool data_ready_2 = false;
    
    // 1. Check if both thermocouples have fresh data ready
    sensor_1.check_data_update(&data_ready_1);
    sensor_2.check_data_update(&data_ready_2);
    
    // 2. Only pull data when the I2C sensors are fully synchronized
    if (data_ready_1 && data_ready_2) {
        
        // Read thermocouples
        sensor_1.read_hot_junc(&temp1);
        sensor_2.read_hot_junc(&temp2);
        
        // Read pressure sensor
        int raw_adc = analogRead(CONTROLLINO_A12);
        float voltage = raw_adc * (10.0 / 1023.0);
        
        // 3. Print everything out on a single line
        Serial.print("T1: ");
        Serial.print(temp1, 2);
        Serial.print(" °C  |  T2: ");
        Serial.print(temp2, 2);
        Serial.print(" °C  |  ");

        // Handle pressure calculations and faults inline
        if (voltage < 1.65) {
            Serial.print("Pressure: FAULT (");
            Serial.print(voltage, 2);
            Serial.println(" V)");
        } else {
            // Subtract the 1.812V baseline, divide by 7.248V span, multiply by 40 Bar
            float pressure = ((voltage - 1.812) / 7.248) * MAX_PRESSURE_BAR;
            if (pressure < 0) pressure = 0; // Clean up micro-fluctuations
            
            Serial.print("Pressure: ");
            Serial.print(pressure, 2);
            Serial.print(voltage, 2);
            Serial.println(" V)");
            Serial.println(" Bar");
        }
    }
    
    // Short polling delay so we don't spam the I2C bus too rapidly
    delay(250); 
}