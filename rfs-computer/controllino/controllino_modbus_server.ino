// Controllino Modbus TCP Server
// Implements the register map from the Instrumentation Naming specification.
//
// Coils (actuators)       0x100–0x107  written by client, applied to outputs
// Discrete Inputs         0x200–0x203  read from digital inputs, served to client
// Holding Registers       0x300–0x305  analog sensors as big-endian IEEE 754 floats

// SETUP:
// - follow https://www.controllino.com/board-library-setup-in-arduino-ide/
//   (choose MAXI Automation)
// - install also the Seeed_MCP9600 library

#include <Controllino.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>  // required dependency of ArduinoModbus
#include <ArduinoModbus.h>
#include <Wire.h>
#include "Seeed_MCP9600.h"

// --- Network ---
byte mac[] = { 0x71, 0x14, 0x89, 0x7d, 0xc1, 0xbf };
IPAddress ip(192, 168, 0, 227);
EthernetServer ethServer(502);
ModbusTCPServer modbusTCPServer;

// --- Thermocouples ---
MCP9600 tc_igniter;        // I2C 0x60 → ign_cc_t        → holding regs 0x302–0x303
MCP9600 tc_cabinet(0x67);  // I2C 0x67 → smart_cabinent_t → holding regs 0x304–0x305

// --- Modbus address map ---
#define COIL_BASE         0x100   // actuators
#define COIL_COUNT        8
#define DISCRETE_IN_BASE  0x200   // digital sensors
#define DISCRETE_IN_COUNT 4
#define HOLDING_REG_BASE  0x300   // analog sensors (3 floats × 2 registers each)
#define HOLDING_REG_COUNT 6

// Actuator output pins, indexed by coil offset from COIL_BASE
const int ACTUATOR_PINS[COIL_COUNT] = {
    CONTROLLINO_D0,  // 0x100: rocket_power    (DO0) – disconnects rocket power when HIGH
    CONTROLLINO_R0,  // 0x101: lox_dec_ext     (R0)
    CONTROLLINO_R1,  // 0x102: lox_dec_ret     (R1)
    CONTROLLINO_R2,  // 0x103: n2_dec_ext      (R2)
    CONTROLLINO_R3,  // 0x104: n2_dec_ret      (R3)
    CONTROLLINO_R4,  // 0x105: n2_shw_vlv      (R4)
    CONTROLLINO_R5,  // 0x106: eng_prg_vlv     (R5)
    CONTROLLINO_R6,  // 0x107: rocket_heating  (R6)
};

// Digital input pins, indexed by offset from DISCRETE_IN_BASE
const int DIGITAL_IN_PINS[DISCRETE_IN_COUNT] = {
    CONTROLLINO_DI0,  // 0x200: smart_valves_armed
    CONTROLLINO_DI1,  // 0x201: rocket_con_detect
    CONTROLLINO_DI2,  // 0x202: lox_dec_prox
    CONTROLLINO_DI3,  // 0x203: n2_dec_prox
};

// Decoupler Extend/retract valves
enum ValvePairState { VP_IDLE, VP_EXT_ON, VP_RET_ON, VP_COOLING_TO_EXT, VP_COOLING_TO_RET };

// A pair of valves with their coil addresses and pins, along with a state machine,
// such that they can enforce mutual exclusion
struct ValvePair {
    uint16_t ext_coil, ret_coil;
    int ext_pin, ret_pin;
    ValvePairState state;
    unsigned long cool_start;
};

ValvePair lox_valves = { 0x101, 0x102, CONTROLLINO_R0, CONTROLLINO_R1, VP_IDLE, 0 };
ValvePair n2_valves  = { 0x103, 0x104, CONTROLLINO_R2, CONTROLLINO_R3, VP_IDLE, 0 };

// Interlock rules:
//   1. All interlocked valves (0x101–0x106) are forced off when valves are not armed (DI0).
//   2. Extend and retract must never both be active simultaneously.
//   3. Writing the opposite side while one is active triggers a toggle:
//      the active side turns off, both pins are held LOW for VALVE_DEADTIME_MS,
//      then the new side turns on.
//   4. During cooloff: canceling (clearing the target coil) → go IDLE immediately.
//      Re-toggling (setting the original coil) → switch cooloff direction without
//      restarting the timer (the valve has already been closed since cool_start).
void updateValvePair(ValvePair& v) {
    const unsigned long VALVE_DEADTIME_MS = 30;

    bool want_ext = modbusTCPServer.coilRead(v.ext_coil);
    bool want_ret = modbusTCPServer.coilRead(v.ret_coil);

    switch (v.state) {

        case VP_IDLE:
            if (want_ext && !want_ret) {
                v.state = VP_EXT_ON;
                digitalWrite(v.ext_pin, HIGH);
            } else if (want_ret && !want_ext) {
                v.state = VP_RET_ON;
                digitalWrite(v.ret_pin, HIGH);
            } else if (want_ext && want_ret) {
                // Both on with no prior context: ignore both
                modbusTCPServer.coilWrite(v.ext_coil, 0);
                modbusTCPServer.coilWrite(v.ret_coil, 0);
            }
            break;

        case VP_EXT_ON:
            if (want_ret) {
                // Toggle: ret requested while ext is on — turn ext off, cooloff, then ret
                modbusTCPServer.coilWrite(v.ext_coil, 0);
                digitalWrite(v.ext_pin, LOW);
                v.state = VP_COOLING_TO_RET;
                v.cool_start = millis();
            } else if (!want_ext) {
                // Simple off: no cooloff needed
                digitalWrite(v.ext_pin, LOW);
                v.state = VP_IDLE;
            }
            break;

        case VP_RET_ON:
            if (want_ext) {
                modbusTCPServer.coilWrite(v.ret_coil, 0);
                digitalWrite(v.ret_pin, LOW);
                v.state = VP_COOLING_TO_EXT;
                v.cool_start = millis();
            } else if (!want_ret) {
                digitalWrite(v.ret_pin, LOW);
                v.state = VP_IDLE;
            }
            break;

        case VP_COOLING_TO_RET:
            // Both pins stay LOW during dead time regardless of coil state
            digitalWrite(v.ext_pin, LOW);
            digitalWrite(v.ret_pin, LOW);

            if (!want_ret) {
                // Target canceled: go idle immediately
                v.state = VP_IDLE;
            } else if (want_ext) {
                // Re-toggle: switch target; timer keeps running from original cool_start
                modbusTCPServer.coilWrite(v.ret_coil, 0);
                v.state = VP_COOLING_TO_EXT;
            } else if (millis() - v.cool_start >= VALVE_DEADTIME_MS) {
                v.state = VP_RET_ON;
                digitalWrite(v.ret_pin, HIGH);
            }
            break;

        case VP_COOLING_TO_EXT:
            digitalWrite(v.ext_pin, LOW);
            digitalWrite(v.ret_pin, LOW);

            if (!want_ext) {
                v.state = VP_IDLE;
            } else if (want_ret) {
                modbusTCPServer.coilWrite(v.ext_coil, 0);
                v.state = VP_COOLING_TO_RET;
            } else if (millis() - v.cool_start >= VALVE_DEADTIME_MS) {
                v.state = VP_EXT_ON;
                digitalWrite(v.ext_pin, HIGH);
            }
            break;
    }
}

// Deactive both valves
void resetValvePair(ValvePair& v) {
    modbusTCPServer.coilWrite(v.ext_coil, 0);
    modbusTCPServer.coilWrite(v.ret_coil, 0);
    digitalWrite(v.ext_pin, LOW);
    digitalWrite(v.ret_pin, LOW);
    v.state = VP_IDLE;
}


// Apply all coil states to hardware outputs.
// Valve pairs are managed by updateValvePair(); remaining coils are applied directly.
void applyActuators() {
    // Non-interlocked outputs always follow their coil
    digitalWrite(CONTROLLINO_D0, modbusTCPServer.coilRead(0x100) ? HIGH : LOW);
    digitalWrite(CONTROLLINO_R6, modbusTCPServer.coilRead(0x107) ? HIGH : LOW);

    // Read armed state from the pin, then either update or reset the interlocked actuators
    bool armed = digitalRead(CONTROLLINO_DI0);
    if (armed) {
        updateValvePair(lox_valves);
        updateValvePair(n2_valves);
        // Single interlocked valves: no ext/ret conflict, just require armed
        digitalWrite(CONTROLLINO_R4, modbusTCPServer.coilRead(0x105) ? HIGH : LOW);
        digitalWrite(CONTROLLINO_R5, modbusTCPServer.coilRead(0x106) ? HIGH : LOW);
    } else {
        resetValvePair(lox_valves);
        resetValvePair(n2_valves);
        modbusTCPServer.coilWrite(0x105, 0);
        digitalWrite(CONTROLLINO_R4, LOW);
        modbusTCPServer.coilWrite(0x106, 0);
        digitalWrite(CONTROLLINO_R5, LOW);
    }
}

// --- Sensors ---

// Read digital inputs into the Modbus discrete input table
void updateDiscreteInputs() {
    for (int i = 0; i < DISCRETE_IN_COUNT; i++) {
        modbusTCPServer.discreteInputWrite(DISCRETE_IN_BASE + i, digitalRead(DIGITAL_IN_PINS[i]));
    }
}

// Pressure: 4–20 mA via 453 Ω resistor → 1.812–9.060 V → 0–40 barg
// Controllino analog inputs: 0–10 V maps to ADC 0–1023
const float PRESSURE_V_ZERO  = 1.812f;  // 4 mA × 453 Ω
const float PRESSURE_V_SPAN  = 7.248f;  // 16 mA × 453 Ω
const float PRESSURE_MAX_BAR = 40.0f;

// Encode a float as two holding registers: high word at baseAddr, low word at baseAddr+1.
// Matches the big-endian decoding in driver.rs: f32::from_bits((hi << 16) | lo)
void writeFloat(int baseAddr, float value) {
    uint32_t bits;
    memcpy(&bits, &value, 4);
    modbusTCPServer.holdingRegisterWrite(baseAddr,     (uint16_t)(bits >> 16));
    modbusTCPServer.holdingRegisterWrite(baseAddr + 1, (uint16_t)(bits & 0xFFFF));
}

// Read the pressure sensor (AI12) and write to holding registers 0x300–0x301
void updatePressure() {
    float voltage = analogRead(CONTROLLINO_A12) * (10.0f / 1023.0f);
    float pressure = 0.0f;
    if (voltage >= PRESSURE_V_ZERO) {
        pressure = ((voltage - PRESSURE_V_ZERO) / PRESSURE_V_SPAN) * PRESSURE_MAX_BAR;
    }
    writeFloat(0x300, pressure);
}

// Read both thermocouples and write to holding registers 0x302–0x305
void updateTemperatures() {
    float temp;
    tc_igniter.read_hot_junc(&temp);
    writeFloat(0x302, temp);
    tc_cabinet.read_hot_junc(&temp);
    writeFloat(0x304, temp);
}

// --- Setup ---

void setup() {
    Serial.begin(9600);

    // Thermocouples
    Wire.begin();
    if (tc_igniter.init(THER_TYPE_K) != NO_ERROR)
        Serial.println("ERROR: TC igniter (0x60) init failed");
    if (tc_cabinet.init(THER_TYPE_K) != NO_ERROR)
        Serial.println("ERROR: TC cabinet (0x67) init failed");

    // Actuator pins – all off at boot
    for (int i = 0; i < COIL_COUNT; i++) {
        pinMode(ACTUATOR_PINS[i], OUTPUT);
        digitalWrite(ACTUATOR_PINS[i], LOW);
    }

    // Digital input pins
    for (int i = 0; i < DISCRETE_IN_COUNT; i++) {
        pinMode(DIGITAL_IN_PINS[i], INPUT);
    }

    // Ethernet
    Ethernet.begin(mac, ip);
    delay(1000);
    Serial.print("IP: ");
    Serial.println(Ethernet.localIP());

    // Modbus TCP server
    ethServer.begin();
    if (!modbusTCPServer.begin()) {
        Serial.println("ERROR: Modbus server init failed");
        while (1);
    }
    modbusTCPServer.configureCoils(COIL_BASE, COIL_COUNT);
    modbusTCPServer.configureDiscreteInputs(DISCRETE_IN_BASE, DISCRETE_IN_COUNT);
    modbusTCPServer.configureHoldingRegisters(HOLDING_REG_BASE, HOLDING_REG_COUNT);

    Serial.println("Modbus TCP server ready on port 502.");
}

// --- Loop ---

void loop() {
    EthernetClient client = ethServer.available();
    if (!client) return;

    Serial.println("client connected");
    modbusTCPServer.accept(client);

    while (client.connected()) {
        modbusTCPServer.poll();
        applyActuators();
        updateDiscreteInputs();
        updatePressure();
        updateTemperatures();
    }

    Serial.println("client disconnected");
}
