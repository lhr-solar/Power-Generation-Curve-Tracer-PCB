/**
 * @file        main.cpp
 * @author      Matthew Yu (matthewjkyu@gmail.com) & Roy Mor (roymor.102@gmail.com)
 * @brief       Controls the IV Curve Tracer and performs PV measurements, able to communicate with Eclipse Simulator for custom parameters.
 * @version     0.3.0
 * @date        2022-12-13
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

#define _DEBUG_TUNING_ 1
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
void heartbeat(void) {led_heartbeat = !led_heartbeat;}

FileHandle *mbed::mbed_override_console(int fd) {
    return &serial_port;
}

enum Mode {
    CELL,
    MODULE,
    ARRAY
};

enum EncodingScheme {
    VERBOSE,
};

float cal_dac_control(float in) {
    // TODO: 10/25/22 recheck dac control calibration.
    const float slope = 9.9539; // const makes it so that slope value is always 9.9539 throughout the program and can't be changed
    const float intercept = 0.0583;
    return in;// * slope + intercept;
}

float cal_sen_volt(float in, int num_iterations, enum Mode mode) {
    // TODO: 10/25/22 recheck voltage sensor calibration.
    // + calibration offset for internal PCB resistances causing a voltage drop prior to the voltage sensor. 
    switch (mode) {
        case CELL:
            return 1.1047 * in / num_iterations;
        case MODULE:
            return 5.4591 * in / num_iterations;
        case ARRAY:
            return 111.8247 * in / num_iterations;
        default:
            tick_heartbeat.detach();
            led_heartbeat = 1;
            while (1);
    }
}

float cal_sen_curr(float in, int num_iterations) {
    // TODO: 10/25/22 recheck current sensor calibration.
    return 8.1169 * in / num_iterations + 0.014;
}

bool modeValid(enum Mode mode) {
    return mode==0||mode==1||mode==2; //checks if the "mode" is in either the set 0, 1, or 2
}

bool gatesValid(float lowGate, float highGate, float gateStep){
    bool lowValid = lowGate >=0 && lowGate <=1 && lowGate < highGate;
    bool highValid = highGate >=0 && highGate <=1 && lowGate < highGate;
    bool stepValid = gateStep <= .1 && gateStep >= 0.001;
    return lowValid && highValid && stepValid;
}

bool iterationsValid(std::uint8_t iterations) {
    return iterations >= 1 && iterations <= 100;
}

bool settleTimeValid(std::uint32_t settlingTime) {
    return settlingTime >= 1000 && settlingTime <= 100000;
}

void readBuffer(char * buffer, std::size_t BUFFER_SIZE) {
    uint8_t idx = 0;
    do {
        //printf("\r\n");
        uint8_t prev_idx = idx;
        idx += serial_port.read(buffer + idx, BUFFER_SIZE);
        //printf("%s", &buffer[prev_idx]);
    } while (buffer[idx - 1] != '\n');
}

void okHandshake(char * buffer, std::size_t BUFFER_SIZE) {
    bool ok = false;
    while (!ok){
        memset(buffer, '\0', BUFFER_SIZE);
        readBuffer(buffer, BUFFER_SIZE);
        if(strncmp(buffer, "OK\r\n", 4)==0) {
            ok = true;
            printf("OK_RECEIVED\r\n");
        } else {
            ok = false;
        }
    }
}

int main() {
    serial_port.set_format(8, BufferedSerial::Even, 1);

    tick_heartbeat.attach(&heartbeat, 500ms);
    dac_control = 0.0;

    printf("v%d.%d.%d\n", MAJOR_VERSION, MINOR_VERSION, PATCH_VERSION);

#ifdef _DEBUG_TUNING_
    /**
     * Debug mode.
     * Run forwards and backwards along the range where the MOSFET R_ds_on
     * experiences 95% variance.
     *
     * User calibrates sensors for measurement accuracy during the test.
     */
    printf("DEBUG MODE\r\n");
    enum Mode mode = CELL;

    int end_count = 0;

    while (1) {
        // Test duration one way: 6.275 seconds
        // 251 steps
        // 25 ms per step
        // 25 substeps per step
        // 1 ms per substep
        #define GATE_OFF                0.001
        #define GATE_ON                 0.999
        #define GATE_STEP               0.001
        #define SETTLING_TIME_US        1000 // us
        #define ITERATIONS              25

        if (end_count >= 3) break;
        printf("*************************************** END_COUNT: %d ***************************************\n", end_count);
        /* Forwards. */
        for (dac_control = GATE_OFF; dac_control <= GATE_ON; dac_control = dac_control + GATE_STEP) {
            float meas_volt = 0.00;
            float meas_curr = 0.00;
            float meas_control = cal_dac_control(dac_control);

            /* SW based sample averaging control. */
            for (uint8_t i = 0; i < ITERATIONS; ++i) {
                wait_us(SETTLING_TIME_US);
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
                meas_volt * meas_curr
            );
        }
        
        printf("*************************************** Backwards ***************************************\n");
        /* Backwards. */
        for (dac_control = GATE_ON; dac_control >= GATE_OFF; dac_control = dac_control - GATE_STEP) {
            float meas_volt = 0.00;
            float meas_curr = 0.00;
            float meas_control = cal_dac_control(dac_control);

            /* SW based sample averaging control. */
            for (uint8_t i = 0; i < ITERATIONS; ++i) {
                wait_us(SETTLING_TIME_US);
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
                meas_volt * meas_curr
            );
        }

        end_count++;

        #undef GATE_OFF
        #undef GATE_ON
        #undef GATE_STEP
        #undef SETTLING_TIME_US
        #undef ITERATIONS
    }
    printf("TERMINATE SCAN MODE\n");
    
#else
    /* Measurement mode. */
    printf("MEASUREMENT MODE\n");

    /* Setup buffer and serial port. */
    #define BUFFER_SIZE 100
    char buffer[BUFFER_SIZE] = { 0 };

    // char buffer2[BUFFER_SIZE] = { 0 };

    while (1) {
        enum Mode mode = CELL;
        enum EncodingScheme scheme = VERBOSE;
        bool valid_conf = false;
        bool ready = false;

        printf("READY_FOR_TRANSMISSION\r\n");

        bool baudSet = false;
        
        while (!baudSet) {

            okHandshake(buffer, BUFFER_SIZE);

            printf("BEGIN_TRANSMISSION\r\n");
            memset(buffer, '\0', BUFFER_SIZE);
            ready = true;
            baudSet = true;
            break;

            // memset(buffer, '\0', BUFFER_SIZE);
            // printf("READY_FOR_BAUDRATE\r\n");
            // readBuffer(buffer, BUFFER_SIZE);
            // int newBaudRate = atoi(buffer); 
            // printf("%d\r\n", newBaudRate);
            // bool baudRateMismatch = newBaudRate!=BAUD_RATE;
            // printf("%d\r\n", baudRateMismatch);

            // if (baudRateMismatch) {
            //     printf("SWAPPING_BAUDRATE\r\n");
            //     wait_us(100000);
            //     serial_port.close();
            //     static BufferedSerial serial_port(USBTX, USBRX, newBaudRate);
            //     // serial_port.set_baud(newBaudRate);  
                
            // } else if(!baudRateMismatch) {
            
            // }


        
        }

        /**
         * Expect pv config string in format:
         *   type=$PV_TYPE,sr=[$LOW,$HIGH,$STEP],ni=$NUM_ITER,st_ms=$SETTLE_TIME,enc=$ENC_SCHEME
         */

        /* Get PV type */
    
        readBuffer(buffer, BUFFER_SIZE);    
        printf("%s", buffer);
        sscanf(buffer,
            "type=%i,sr=[%f,%f,%f],ni=%i,st_ms=%i,enc=%i\r\n",
            &mode, &GATE_OFF, &GATE_ON, &GATE_STEP,
            &ITERATIONS, &SETTLING_TIME_US, &scheme);
        memset(buffer, '\0', BUFFER_SIZE);

        printf("config received\r\n");
        SETTLING_TIME_US*=1000;
        /* TODO: check for valid results */
        if(modeValid(mode)) printf("mode valid\r\n");
        if(gatesValid(GATE_OFF,GATE_ON,GATE_STEP)) printf("gates valid\r\n");
        if(iterationsValid(ITERATIONS)) printf("iterations valid\r\n");
        if(settleTimeValid(SETTLING_TIME_US)) printf("Settle Time valid\r\n");
        valid_conf = modeValid(mode) && gatesValid(GATE_OFF, GATE_ON, GATE_STEP) && iterationsValid(ITERATIONS) && settleTimeValid(SETTLING_TIME_US);
        if(valid_conf){
            printf("valid config\r\n");
        } else {
            printf("invalid config\r\n");
        }

        if (valid_conf && ready) {
            printf("BEGIN_SCAN\r\n");
            /* Forwards. */
            printf("forwards\r\n");
            printf("%f %f %f\r\n", GATE_OFF,GATE_ON,GATE_STEP);
            float currentGate;
            for (currentGate = GATE_OFF; currentGate <= GATE_ON; currentGate = currentGate + GATE_STEP) {
                dac_control = currentGate;
                float meas_volt = 0.00;
                float meas_curr = 0.00;
                float meas_control = cal_dac_control(dac_control);

                /* SW based sample averaging control. */
                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                switch (scheme) {
                    case VERBOSE:
                        printf(
                            "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                            meas_control,
                            meas_volt,
                            meas_curr,
                            meas_volt * meas_curr
                        );
                        break;
                }

            }

            /* Backwards. */
            printf("backwards\r\n");
            for (currentGate = GATE_ON; currentGate >= GATE_OFF; currentGate = currentGate - GATE_STEP) {
                dac_control = currentGate;
                float meas_volt = 0.00;
                float meas_curr = 0.00;
                float meas_control = cal_dac_control(dac_control);

                /* SW based sample averaging control. */
                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                switch (scheme) {
                    case VERBOSE:
                        printf(
                            "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                            meas_control,
                            meas_volt,
                            meas_curr,
                            meas_volt * meas_curr
                        );
                        break;
                }
            }

            /* Forwards. */
            printf("forwards\r\n");
            for (currentGate = GATE_OFF; currentGate <= GATE_ON; currentGate = currentGate + GATE_STEP) {
                dac_control = currentGate;
                float meas_volt = 0.00;
                float meas_curr = 0.00;
                float meas_control = cal_dac_control(dac_control);

                /* SW based sample averaging control. */
                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, 25, mode);
                meas_curr = cal_sen_curr(meas_curr, 25);

                switch (scheme) {
                    case VERBOSE:
                        printf(
                            "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                            meas_control,
                            meas_volt,
                            meas_curr,
                            meas_volt * meas_curr
                        );
                        break;
                }

            }

            /* Backwards. */
            printf("backwards\r\n");
            for (currentGate = GATE_ON; currentGate >= GATE_OFF; currentGate = currentGate - GATE_STEP) {
                dac_control = currentGate;
                float meas_volt = 0.00;
                float meas_curr = 0.00;
                float meas_control = cal_dac_control(dac_control);

                /* SW based sample averaging control. */
                for (uint8_t i = 0; i < ITERATIONS; ++i) {
                    wait_us(SETTLING_TIME_US);
                    meas_volt += sen_voltage.read();
                    meas_curr += sen_current.read();
                }

                meas_volt = cal_sen_volt(meas_volt, ITERATIONS, mode);
                meas_curr = cal_sen_curr(meas_curr, ITERATIONS);

                switch (scheme) {
                    case VERBOSE:
                        printf(
                            "Gate (V): %f, VSense (V): %f, ISense (A): %f, V*I (W): %f\n",
                            meas_control,
                            meas_volt,
                            meas_curr,
                            meas_volt * meas_curr
                        );
                        break;
                }
            }

            printf("END_SCAN\r\n");
        }
    }
#endif
}