// Controllino Modbus TCP Server
// Implements the register map from the Instrumentation Naming specification.
//
// Coils (actuators)       0x100–0x107  written by client, applied to outputs
// Discrete Inputs         0x200–0x203  read from digital inputs, served to client
// Holding Registers       0x300–0x305  analog sensors as big-endian IEEE 754 floats

#include <Controllino.h>
#include <Ethernet.h>
#include <ArduinoRS485.h>  // required dependency of ArduinoModbus
#include <ArduinoModbus.h>
#include <Wire.h>
#include "Seeed_MCP9600.h"

// --- Network ---
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 227);
EthernetServer ethServer(502);
ModbusTCPServer modbusTCPServer;

// --- Thermocouples ---
MCP9600 tc_igniter;        // I2C 0x60 → ign_cc_t   → holding regs 0x302–0x303
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

// Pressure: 4–20 mA via 453 Ω resistor → 1.812–9.060 V → 0–40 barg
// Controllino analog inputs: 0–10 V maps to ADC 0–1023
const float PRESSURE_V_ZERO  = 1.812f;  // 4 mA × 453 Ω
const float PRESSURE_V_SPAN  = 7.248f;  // 16 mA × 453 Ω
const float PRESSURE_MAX_BAR = 40.0f;

// --- Helpers ---

// Encode a float as two holding registers: high word at baseAddr, low word at baseAddr+1.
// Matches the big-endian decoding in driver.rs: f32::from_bits((hi << 16) | lo)
void writeFloat(int baseAddr, float value) {
    uint32_t bits;
    memcpy(&bits, &value, 4);
    modbusTCPServer.holdingRegisterWrite(baseAddr,     (uint16_t)(bits >> 16));
    modbusTCPServer.holdingRegisterWrite(baseAddr + 1, (uint16_t)(bits & 0xFFFF));
}

// Interlock rules:
// 1. All interlocked valves (0x101–0x106) are forced off when valves are not armed (DI0).
// 2. Extend and retract on the same coupler must never both be active.
//    When both are commanded, the one that was off last tick (the "last on") stays on.
void enforceInterlocks() {
    static bool prev_lox_ext = false, prev_lox_ret = false;
    static bool prev_n2_ext  = false, prev_n2_ret  = false;

    // Read valve-armed interlock directly from the pin (not from the stale Modbus table)
    bool armed = digitalRead(CONTROLLINO_DI0);
    if (!armed) {
        for (int coil = 0x101; coil <= 0x106; coil++) {
            modbusTCPServer.coilWrite(coil, 0);
        }
        prev_lox_ext = prev_lox_ret = prev_n2_ext = prev_n2_ret = false;
        return;
    }

    // LOX extend/retract: if both on, the newly-set one (was off last tick) wins
    bool lox_ext = modbusTCPServer.coilRead(0x101);
    bool lox_ret = modbusTCPServer.coilRead(0x102);
    if (lox_ext && lox_ret) {
        if (!prev_lox_ext) {  // ext just turned on → ext wins
            modbusTCPServer.coilWrite(0x102, 0);
            lox_ret = false;
        } else {              // ret just turned on (or both new simultaneously: ret wins)
            modbusTCPServer.coilWrite(0x101, 0);
            lox_ext = false;
        }
    }

    // N2 extend/retract: same logic
    bool n2_ext = modbusTCPServer.coilRead(0x103);
    bool n2_ret = modbusTCPServer.coilRead(0x104);
    if (n2_ext && n2_ret) {
        if (!prev_n2_ext) {
            modbusTCPServer.coilWrite(0x104, 0);
            n2_ret = false;
        } else {
            modbusTCPServer.coilWrite(0x103, 0);
            n2_ext = false;
        }
    }

    prev_lox_ext = lox_ext;
    prev_lox_ret = lox_ret;
    prev_n2_ext  = n2_ext;
    prev_n2_ret  = n2_ret;
}

// Apply coil states to hardware outputs
void applyActuators() {
    enforceInterlocks();
    for (int i = 0; i < COIL_COUNT; i++) {
        digitalWrite(ACTUATOR_PINS[i], modbusTCPServer.coilRead(COIL_BASE + i) ? HIGH : LOW);
    }
}

// Read digital inputs into the Modbus discrete input table
void updateDiscreteInputs() {
    for (int i = 0; i < DISCRETE_IN_COUNT; i++) {
        modbusTCPServer.discreteInputWrite(DISCRETE_IN_BASE + i, digitalRead(DIGITAL_IN_PINS[i]));
    }
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
    if (tc_igniter.init(THER_TYPE_K) != NO_ERROR) {
        Serial.println("ERROR: TC igniter (0x60) init failed");
    }
    if (tc_cabinet.init(THER_TYPE_K) != NO_ERROR) {
        Serial.println("ERROR: TC cabinet (0x67) init failed");
    }

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
