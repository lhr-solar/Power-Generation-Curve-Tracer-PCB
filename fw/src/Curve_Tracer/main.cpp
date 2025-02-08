/**
 * @file        main.cpp
 * @author      Matthew Yu (matthewjkyu@gmail.com), Roy Mor (roymor.102@gmail.com), Connor Shen (connor.lishen@gmail.com)
 * @brief       Controls the IV Curve Tracer and performs PV measurements, able to communicate with Eclipse Simulator for custom parameters.
 * @version     0.4.0
 * @date        2025-01-25
 * @copyright   Copyright (c) 2022
 * @note        Use _DEBUG_TUNING flag to perform manual tuning.
 *              Default baud rate is 115200 baud.
 * @todo        - TSL2591 light sensor support through CAN
 *              - RTD temperature sensor support through CAN
 */

#include "mbed.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ArduinoJson.h>

// #define _DEBUG_TUNING_ 1
#define MAJOR_VERSION 2
#define MINOR_VERSION 0
#define PATCH_VERSION 0
#define BAUD_RATE 115200 // 57600

/** Peripherals. */
static DigitalOut led_heartbeat(D1);
static DigitalOut led_scan(D0);
static AnalogIn sen_voltage(A6);
static AnalogIn sen_current(A0);
static AnalogOut dac_control(A3);
//static CAN can(D10, D2);
//static CANMessage msg;
static BufferedSerial serial_port(USBTX, USBRX, BAUD_RATE);

/** Global Variables **/
float GATE_OFF = 0.00;
float GATE_ON = 1.00;
float GATE_STEP = 0.001;
uint8_t ITERATIONS = 25;
uint32_t SETTLING_TIME_US = 1000;
enum Mode { CELL, MODULE, ARRAY } mode;
enum Operations { DEBUG, MEASUREMENT } operation;
bool system_reset = false

/** Tickers. */
static LowPowerTicker tick_heartbeat;
void heartbeat(void) { led_heartbeat = !led_heartbeat; }

FileHandle *mbed::mbed_override_console(int fd){ return &serial_port; }

void okHandshake(char *buffer, std::size_t BUFFER_SIZE) {
    printf("READY_FOR_HANDSHAKE\r\n");
    while (true) {
        memset(buffer, '\0', BUFFER_SIZE);
        if (!readBuffer(buffer, BUFFER_SIZE, 2000)) { 
            printf("ERROR: Handshake timeout\r\n"); 
            continue; 
        }
        if (strncmp(buffer, "OK\r\n", 4) == 0) { 
            printf("OK_RECEIVED\r\n"); 
            break; 
        } else { 
            printf("ERROR: Invalid handshake response\r\n"); 
        }
    }
}

float cal_dac_control(float in)
{
    // TODO: 10/25/22 recheck dac control calibration.
    const float slope = 9.9539; // const makes it so that slope value is always 9.9539 throughout the program and can't be changed
    const float intercept = 0.0583;
    return in; // * slope + intercept;
}

float cal_sen_volt(float in, int num_iterations, enum Mode mode)
{
    // TODO: 10/25/22 recheck voltage sensor calibration.
    // + calibration offset for internal PCB resistances causing a voltage drop prior to the voltage sensor.
    switch (mode)
    {
    case CELL:
        return 1.1047 * in / num_iterations;
    case MODULE:
        return 5.4591 * in / num_iterations;
    case ARRAY:
        return 111.8247 * in / num_iterations;
    default:
        tick_heartbeat.detach();
        led_heartbeat = 1;
        while (1)
            ;
    }
}

float cal_sen_curr(float in, int num_iterations)
{
    // TODO: 10/25/22 recheck current sensor calibration.
    return 8.1169 * in / num_iterations + 0.014;
}

bool gatesValid(float lowGate, float highGate, float gateStep)
{
    bool lowValid = lowGate >= 0 && lowGate <= 1 && lowGate < highGate;
    bool highValid = highGate >= 0 && highGate <= 1 && lowGate < highGate;
    bool stepValid = gateStep <= .1 && gateStep >= 0.001;
    return lowValid && highValid && stepValid;
}

bool iterationsValid(std::uint8_t iterations)
{
    return iterations >= 1 && iterations <= 100;
}

bool settleTimeValid(std::uint32_t settlingTime)
{
    return settlingTime >= 1000 && settlingTime <= 100000;
}

bool readBuffer(char *buffer, std::size_t BUFFER_SIZE, uint32_t timeout_ms)
{
    uint8_t idx = 0;
    uint32_t elapsed_time = 0;
    const uint32_t poll_interval = 10; // Check every 10ms

    while (elapsed_time < timeout_ms)
    {
        if (serial_port.readable())
        {
            uint8_t bytes_read = serial_port.read(buffer + idx, BUFFER_SIZE - idx - 1); // Reserve space for null terminator
            idx += bytes_read;

            if (idx >= BUFFER_SIZE - 1)
            {
                printf("ERROR: Buffer overflow\r\n");
                buffer[BUFFER_SIZE - 1] = '\0'; // Null terminate to prevent undefined behavior
                return false;
            }

            if (buffer[idx - 1] == '\n')
            {
                buffer[idx] = '\0'; // Null terminate the string
                return true;        // Successfully received data
            }
        }
        wait_us(poll_interval * 1000);
        elapsed_time += poll_interval;
    }
    printf("ERROR: Serial read timeout\r\n");
    return false; // Timeout occurred
}

void parseJSONConfig(char *buffer)
{
    StaticJsonDocument<256> doc;

    // Deserialize the JSON string into a document
    DeserializationError error = deserializeJson(doc, buffer);
    if (error){ printf("ERROR: Invalid JSON Config\r\n"); return; }
    
    const char* mode_str = doc["mode"];
    if (strcmp(mode_str, "DEBUG") == 0) {
        operation = DEBUG;
    } 
    else if (strcmp(mode_str, "MEASUREMENT") == 0) {
        operation = MEASUREMENT;
    }
    else {
        printf("ERROR: Invalid mode selection \r\n");
        return
    }

    if (!gatesValid(sr_low, sr_high, step))
    {
        printf("ERROR: Invalid gate range\r\n");
        return;
    }

    if (!iterationsValid(num_iters))
    {
        printf("ERROR: Invalid iterations\r\n");
        return;
    }

    if (!settleTimeValid(settling_time))
    {
        printf("ERROR: Invalid settling time\r\n");
        return;
    }

    mode = static_cast<Mode>(doc["type"]);
    GATE_OFF = doc["sr"][0];
    GATE_ON = doc["sr"][1];
    GATE_STEP = doc["sr"][2];
    ITERATIONS = doc["num_iters"];
    SETTLING_TIME_US = doc["settling_time"] * 1000;

    printf("valid config\r\n");
}

void systemReset() {
    printf("SYSTEM RESET INITIATED...\r\n");

    // Reset global variables to default values
    GATE_OFF = 0.00;
    GATE_ON = 1.00;
    GATE_STEP = 0.001;
    ITERATIONS = 25;
    SETTLING_TIME_US = 1000;
    system_reset_requested = false;

    // Restart the handshake process
    char buffer[256] = {0};
    printf("Waiting for connection...\r\n");
    okHandshake(buffer, sizeof(buffer));

    printf("BEGIN_TRANSMISSION\r\n");
    if (!readBuffer(buffer, sizeof(buffer), 5000)) { 
        printf("ERROR: Failed to receive config\r\n"); 
        return; 
    }
    parseJSONConfig(buffer);
}

void monitorReset() {
    char buffer[10];
    while (true) {
        if (readBuffer(buffer, sizeof(buffer), 1000)) {
            if (strncmp(buffer, "RESET", 5) == 0) {
                system_reset_requested = true;
                break;
            }
        }
    }
}

void sendMeasurementResults(float gate, float voltage, float current) {
    StaticJsonDocument<128> json;
    json["gate"] = gate;
    json["voltage"] = voltage;
    json["current"] = current;
    json["power"] = voltage * current;

    // Compute simple checksum (XOR of float values converted to int)
    uint32_t checksum = (uint32_t)(gate * 1000) ^ (uint32_t)(voltage * 1000) ^ 
                        (uint32_t)(current * 1000) ^ (uint32_t)(json["power"].as<float>() * 1000);
    json["checksum"] = checksum;

    char json_output[128];
    serializeJson(json, json_output);
    printf("%s\r\n", json_output);
}

int main() {
    serial_port.set_format(8, BufferedSerial::Even, 1);
    tick_heartbeat.attach(&heartbeat, 500ms);
    dac_control = 0.0;
    printf("Curve Tracer Firmware v%d.%d.%d\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION);

    char buffer[256] = {0};
    printf("Waiting for connection...\r\n");
    okHandshake(buffer, sizeof(buffer));

    printf("BEGIN_TRANSMISSION\r\n");
    if (!readBuffer(buffer, sizeof(buffer), 5000)) { 
        printf("ERROR: Failed to receive config\r\n"); 
        return 1; 
    }
    parseJSONConfig(buffer);

    // Start monitoring for RESET command in a separate thread
    Thread resetThread;
    resetThread.start(monitorReset);

    while (true) {
        if (system_reset_requested) {
            systemReset();
        }

        if (operation == MEASUREMENT) {
            printf("MEASUREMENT MODE STARTING...\r\n");

            printf("BEGIN SCAN\r\n");
            for (float currentGate = GATE_OFF; currentGate <= GATE_ON; currentGate += GATE_STEP) {
                dac_control = currentGate;
                float meas_volt = 0.00, meas_curr = 0.00;

                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US + 100);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);
                sendMeasurementResults(cal_dac_control(currentGate), meas_volt, meas_curr);
            }

            for (float currentGate = GATE_ON; currentGate >= GATE_OFF; currentGate -= GATE_STEP) {
                dac_control = currentGate;
                float meas_volt = 0.00, meas_curr = 0.00;

                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US + 100);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);
                sendMeasurementResults(cal_dac_control(currentGate), meas_volt, meas_curr);
            }

            printf("END SCAN\r\n");

        } else if (operation == DEBUG) {
            printf("DEBUG MODE ACTIVATED\r\n");
            printf("Commands: 'INC' (increase voltage), 'DEC' (decrease voltage), 'EXIT' (exit debug mode)\r\n");

            float currentGate = GATE_OFF;
            dac_control = currentGate;
            char debugBuffer[20];

            while (true) {
                printf("Current Gate Voltage: %.3fV | Reading Sensors...\r\n", currentGate);
                
                float meas_volt = 0.00, meas_curr = 0.00;
                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US + 100);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);
                sendMeasurementResults(cal_dac_control(currentGate), meas_volt, meas_curr);

                if (!readBuffer(debugBuffer, sizeof(debugBuffer), 5000)) continue;

                if (strncmp(debugBuffer, "INC", 3) == 0 && currentGate + GATE_STEP <= GATE_ON) {
                    currentGate += GATE_STEP;
                    dac_control = currentGate;
                    printf("Voltage Increased to %.3fV\r\n", currentGate);
                } else if (strncmp(debugBuffer, "DEC", 3) == 0 && currentGate - GATE_STEP >= GATE_OFF) {
                    currentGate -= GATE_STEP;
                    dac_control = currentGate;
                    printf("Voltage Decreased to %.3fV\r\n", currentGate);
                } else if (strncmp(debugBuffer, "EXIT", 4) == 0) {
                    printf("Exiting Debug Mode...\r\n");
                    break;
                } else {
                    printf("Invalid Command. Use 'INC', 'DEC', or 'EXIT'\r\n");
                }
            }
        }

        // Keep checking for RESET command
        wait_us(1000000);
    }

    return 0;
}