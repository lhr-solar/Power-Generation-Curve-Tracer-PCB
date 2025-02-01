/**
 * @file        main.cpp
 * @author      Matthew Yu (matthewjkyu@gmail.com), Roy Mor (roymor.102@gmail.com), Connor Shen (connor.lishen@gmail.com)
 * @brief       Controls the IV Curve Tracer and performs PV measurements, able to communicate with Eclipse Simulator for custom parameters.
 * @version     0.4.0
 * @date        2025-01-25
 * @copyright   Copyright (c) 2022
 * @note        Use _DEBUG_TUNING flag to perform manual tuning.
 *              Default baud rate is 115200 baud.
 * @todo        - Communication with IV Curve Tracer
 *              - TSL2591 light sensor support through CAN
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
#define MAJOR_VERSION 0
#define MINOR_VERSION 2
#define PATCH_VERSION 0
#define BAUD_RATE 115200 // 57600

/** Peripherals. */
static DigitalOut led_heartbeat(D1);
static DigitalOut led_scan(D0);
static AnalogIn sen_voltage(A6);
static AnalogIn sen_current(A0);
static AnalogOut dac_control(A3);
static CAN can(D10, D2);
static CANMessage msg;
static BufferedSerial serial_port(USBTX, USBRX, BAUD_RATE);

float GATE_OFF = 0.00;
float GATE_ON = 1.00;
float GATE_STEP = 0.001;
uint8_t ITERATIONS = 25;
uint32_t SETTLING_TIME_US = 0;

/** Tickers. */
static LowPowerTicker tick_heartbeat;
void heartbeat(void) { led_heartbeat = !led_heartbeat; }

FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}

enum Mode
{
    CELL,
    MODULE,
    ARRAY
};

enum EncodingScheme
{
    VERBOSE,
};

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

bool modeValid(enum Mode mode)
{
    return mode == 0 || mode == 1 || mode == 2; // checks if the "mode" is in either the set 0, 1, or 2
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

    // Check for JSON parsing errors
    if (error)
    {
        printf("ERROR: Invalid JSON Config\r\n");
        return;
    }

    // Extract configuration parameters
    int type = doc["type"];
    float sr_low = doc["sr"][0];
    float sr_high = doc["sr"][1];
    float step = doc["sr"][2];
    int num_iters = doc["ni"];
    int settling_time = doc["st_ms"];
    int enc_scheme = doc["enc"];

    //Extract PV mode (0 = CELL, 1 = MODULE, 2 = ARRAY)
    int received_mode = doc["type"];
    if (!modeValid(received_mode))
    {
        printf("ERROR: Invalid mode received\r\n");
        return;
    }
    mode = static_cast<Mode>(received_mode); // Update the global variable

    // Extract other parameters
    GATE_OFF = doc["sr"][0];
    GATE_ON = doc["sr"][1];
    GATE_STEP = doc["sr"][2];
    ITERATIONS = doc["num_iters"];
    SETTLING_TIME_US = doc["settling_time"] * 1000;

    printf("Valid configuration received. Mode set to %d\r\n", mode);

    // Validate parameters
    if (!modeValid(static_cast<Mode>(type)))
    {
        printf("ERROR: Invalid mode\r\n");
        return;
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

    // Apply validated configuration
    GATE_OFF = sr_low;
    GATE_ON = sr_high;
    GATE_STEP = step;
    ITERATIONS = num_iters;
    SETTLING_TIME_US = settling_time * 1000;

    printf("valid config\r\n");
}

void okHandshake(char *buffer, std::size_t BUFFER_SIZE)
{
    bool ok = false;
    int retries = 5;
    while (!ok && retries > 0)
    {
        memset(buffer, '\0', BUFFER_SIZE);
        if (!readBuffer(buffer, BUFFER_SIZE, 2000))
        { // 2 sec timeout
            printf("ERROR: Handshake timeout\r\n");
            retries--;
            continue;
        }
        if (strncmp(buffer, "OK\r\n", 4) == 0)
        {
            ok = true;
            printf("OK_RECEIVED\r\n");
        }
        else
        {
            ok = false;
            printf("ERROR: Invalid handshake response\r\n");
            retries--;
        }

        if (!ok)
        {
            printf("ERROR: Handshake failed after retries\r\n");
            return;
        }
    }
}

int main()
{
    // Set up serial port with appropriate format
    serial_port.set_format(8, BufferedSerial::Even, 1);
    tick_heartbeat.attach(&heartbeat, 500ms); // Heartbeat LED for activity indication
    dac_control = 0.0;                        // Initialize DAC output
    printf("v%d.%d.%d\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION);

    // Mode selection
    printf("Select mode:\n");
    printf("1: Debug Mode\n");
    printf("2: Measurement Mode\n");
    printf("Enter choice: ");

    char mode_buffer[10];
    memset(mode_buffer, '\0', sizeof(mode_buffer));
    readBuffer(mode_buffer, sizeof(mode_buffer), 5000); // 5-sec timeout for mode selection

    int mode_selection = atoi(mode_buffer);
    if (mode_selection != 1 && mode_selection != 2)
    {
        printf("ERROR: Invalid mode selected. Defaulting to Measurement Mode.\n");
        mode_selection = 2; // Default Measurement Mode
    }

    if (mode_selection == 1)
    {
        /**
         * Debug mode: Manual tuning for calibration
         * Run forwards and backwards along the range where the MOSFET R_ds_on
         * experiences 95% variance.
         *
         * User calibrates sensors for measurement accuracy during the test.
         */
        printf("DEBUG MODE\r\n");
        enum Mode mode = CELL;
        int end_count = 0;

        while (1)
        {
            if (end_count >= 3)
                break;
            printf("*************************************** END_COUNT: %d ***************************************\n", end_count);

            /* Forward sweep */
            for (dac_control = GATE_OFF; dac_control <= GATE_ON; dac_control += GATE_STEP)
            {
                float meas_volt = 0.00;
                float meas_curr = 0.00;
                float meas_control = cal_dac_control(dac_control);

                /* SW based sample averaging control. */
                for (uint8_t i = 0; i < ITERATIONS; ++i)
                {
                    wait_us(SETTLING_TIME_US + 100);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                printf(
                    "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                    meas_control,
                    meas_volt,
                    meas_curr,
                    meas_volt * meas_curr);
            }

            printf("*************************************** Backwards ***************************************\n");
            /* Backward sweep. */
            for (dac_control = GATE_ON; dac_control >= GATE_OFF; dac_control -= GATE_STEP)
            {
                float meas_volt = 0.00;
                float meas_curr = 0.00;
                float meas_control = cal_dac_control(dac_control);

                /* SW based sample averaging control. */
                for (uint8_t i = 0; i < ITERATIONS; ++i)
                {
                    wait_us(SETTLING_TIME_US + 100);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                printf(
                    "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                    meas_control,
                    meas_volt,
                    meas_curr,
                    meas_volt * meas_curr);
            }

            end_count++;
        }
        printf("TERMINATE SCAN MODE\n");
    }
    else if (mode_selection == 2)
    {
        /* Measurement mode: Automated IV and PV curve scanning */
        printf("MEASUREMENT MODE\n");

/* Setup buffer and serial port. */
#define BUFFER_SIZE 256
        char buffer[BUFFER_SIZE] = {0};
        bool valid_conf = false;

        while (1)
        {
            enum Mode mode = CELL;
            bool ready = false;

            printf("READY_FOR_TRANSMISSION\r\n");

            // Perform handshake with the controller
            okHandshake(buffer, BUFFER_SIZE);
            printf("BEGIN_TRANSMISSION\r\n");

            // Read and Parse the JSON config
            memset(buffer, '\0', BUFFER_SIZE);
            if (!readBuffer(buffer, BUFFER_SIZE, 5000))
            { // Timeout of 5000ms
                printf("ERROR: Failed to receive valid data\r\n");
                continue;
            }

            printf("Received Config: %s\r\n", buffer);
            parseJSONConfig(buffer);

            // Validate the parsed configs
            valid_conf = modeValid(mode) && gatesValid(GATE_OFF, GATE_ON, GATE_STEP) && iterationsValid(ITERATIONS) && settleTimeValid(SETTLING_TIME_US);

            if (valid_conf)
            {
                printf("valid config\r\n");
                ready = true;
            }
            else
            {
                printf("invalid config\r\n");
                continue; // Skip if configuration is invalid
            }
            if (valid_conf && ready)
            {
                printf("BEGIN SCAN\r\n");

                // Forward Sweep
                for (float currentGate = GATE_OFF; currentGate <= GATE_ON; currentGate += GATE_STEP)
                {
                    dac_control = currentGate;
                    float meas_volt = 0.00;
                    float meas_curr = 0.00;

                    for (uint8_t i = 0; i < ITERATIONS; ++i)
                    {
                        wait_us(SETTLING_TIME_US + 100);
                        meas_volt += sen_voltage.read();
                        meas_curr += sen_current.read();
                    }

                    meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                    meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                    printf(
                        "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                        cal_dac_control(currentGate),
                        meas_volt,
                        meas_curr,
                        meas_volt * meas_curr);
                }

                // Backward Sweep
                for (float currentGate = GATE_ON; currentGate >= GATE_OFF; currentGate -= GATE_STEP)
                {
                    dac_control = currentGate;
                    float meas_volt = 0.00;
                    float meas_curr = 0.00;

                    for (uint8_t i = 0; i < ITERATIONS; ++i)
                    {
                        wait_us(SETTLING_TIME_US + 100);
                        meas_volt += sen_voltage.read();
                        meas_curr += sen_current.read();
                    }

                    meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                    meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                    printf(
                        "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                        cal_dac_control(currentGate),
                        meas_volt,
                        meas_curr,
                        meas_volt * meas_curr);
                }

                printf("END SCAN\r\n");
            }
        }
    }
}