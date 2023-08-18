/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>                                                                                                       // Include standard integer types
#include <stddef.h>                                                                                                       // Include standard definition types
#include <stdio.h>                                                                                                        // Include standard input/output library

/* Driver Header files */
#include <ti/drivers/GPIO.h>                                                                                              // Include GPIO driver
#include <ti/drivers/I2C.h>                                                                                               // Include I2C driver
#include <ti/drivers/UART2.h>                                                                                             // Include UART2 driver
#include <ti/drivers/Timer.h>                                                                                             // Include Timer driver
#include <ti/display/Display.h>                                                                                           // Include Display driver
#include <ti/display/DisplayUart2.h>                                                                                      // Include DisplayUart2 driver

/* Driver configuration */
#include "ti_drivers_config.h"                                                                                            // Include driver configuration


/* Init buttons before callback */
int leftButton = 0;                                                                                                       // Initialize left button for decrement
int rightButton = 0;                                                                                                      // Initialize right button for increment


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index) {

    /* Button function */
    leftButton = 1;                                                                                                       // Set left button flag to 1

}


/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index) {

    /* Button function */
    rightButton = 1;                                                                                                      // Set right button flag to 1

}


/*
 *  ======== UART code begin ========
 */
#define DISPLAY(x) UART2_write(uart2, &output, x, bytesToSend);


/* UART Global Variables */
char output[64];                                                                                                          // Output buffer for UART
size_t *bytesToSend;                                                                                                      // Pointer to bytes to send


UART2_Handle uart2;                                                                                                       // UART2 handle


void initUART(void) {

    /* Configure the driver */
    UART2_Params uart2Params;                                                                                             // UART2 parameters
    UART2_Params_init(&uart2Params);                                                                                      // Initialize UART2 parameters
    uart2Params.writeMode = UART2_Mode_BLOCKING;                                                                          // Set write mode to blocking
    uart2Params.readMode = UART2_Mode_BLOCKING;                                                                           // Set read mode to blocking
    uart2Params.readReturnMode = UART2_ReadReturnMode_PARTIAL;                                                            // Set read return mode to partial
    uart2Params.baudRate = 115200;                                                                                        // Set baud rate to 115200

    /* Open the driver */
    uart2 = UART2_open(CONFIG_UART2_0, &uart2Params);                                                                     // Open UART2 with parameters

    if (uart2 == NULL) {

        /* UART_open() failed */
        while (1);                                                                                                        // Infinite loop if UART2 open failed

    }

}


/*
 *  ======== I2C code begin ========
 */
/* I2C Global Variables */
static const struct {

    uint8_t address;                                                                                                      // I2C address
    uint8_t resultReg;                                                                                                    // Result register
    char *id;                                                                                                             // Sensor ID

}

sensors[3] = {

              { 0x48, 0x0000, "11X" },                                                                                    // Sensor 1 configuration
              { 0x49, 0x0000, "116" },                                                                                    // Sensor 2 configuration
              { 0x41, 0x0001, "006" }                                                                                     // Sensor 3 configuration

};


uint8_t txBuffer[1];                                                                                                      // Transmit buffer
uint8_t rxBuffer[2];                                                                                                      // Receive buffer
I2C_Transaction i2cTransaction;                                                                                           // I2C transaction


/* Driver Handles - Global variables */
I2C_Handle i2c;                                                                                                           // I2C handle


/* Make sure you call initUART() before calling this function. */
void initI2C(void) {

    int8_t i, found;                                                                                                      // Variables for loop and found flag
    I2C_Params i2cParams;                                                                                                 // I2C parameters
    DISPLAY(UART2_write(uart2, output, 64, bytesToSend))                                                                  // Display UART2 write

    /* Init the driver */
    I2C_init();                                                                                                           // Initialize I2C

    /* Configure the driver */
    I2C_Params_init(&i2cParams);                                                                                          // Initialize I2C parameters
    i2cParams.bitRate = I2C_400kHz;                                                                                       // Set bit rate to 400kHz

    /* Open the driver */
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);                                                                             // Open I2C with parameters

    if (i2c == NULL) {

        DISPLAY(snprintf(output, 64, "Failed\n\r"))                                                                       // Display failure message
        while (1);                                                                                                        // Infinite loop if I2C open failed

    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))                                                                           // Display success message

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;                                                                                   // Set write buffer
    i2cTransaction.writeCount = 1;                                                                                        // Set write count
    i2cTransaction.readBuf = rxBuffer;                                                                                    // Set read buffer
    i2cTransaction.readCount = 0;                                                                                         // Set read count

    found = false;                                                                                                        // Initialize found flag to false

    for (i = 0; i < 3; ++i) {

        i2cTransaction.targetAddress = sensors[i].address;                                                                // Set target address
        txBuffer[0] = sensors[i].resultReg;                                                                               // Set result register
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))                                                      // Display sensor ID

        if (I2C_transfer(i2c, &i2cTransaction)) {

            DISPLAY(snprintf(output, 64, "Found\n\r"))                                                                    // Display found message
            found = true;                                                                                                 // Set found flag to true

            break;                                                                                                        // Break loop if found

        }

        DISPLAY(snprintf(output, 64, "No\n\r"))                                                                           // Display not found message

    };

    if (found) {

        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))  // Display detected sensor
    }

    else {

        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))                              // Display sensor not found message

    }


}


int16_t readTemp(void) {

    int16_t temperature = 0;                                                                                              // Initialize temperature variable to 0
    i2cTransaction.readCount = 2;                                                                                         // Set the read count for the I2C transaction to 2 bytes

    if (I2C_transfer(i2c, &i2cTransaction)) {                                                                             // If the I2C transfer is successful

        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */

        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);                                                                 // Combine the two received bytes into a single 16-bit temperature value
        temperature *= 0.0078125;                                                                                         // Convert the raw temperature value to degrees Celsius


        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */

        if (rxBuffer[0] & 0x80) {                                                                                         // If the most significant bit is set

            temperature |= 0xF000;                                                                                        // Sign extend the temperature value

        }


    }

    else {                                                                                                                // If the I2C transfer failed

        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r ,i2cTransaction.status"))                 // Display an error message
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))        // Suggest a solution

    }

    return temperature;                                                                                                   // Return the temperature value

}


/*
 *  ======== Timer code begin ========
 */
/* Driver Handles - Global variables */

Timer_Handle timer0;                                                                                                      // Timer handle
volatile unsigned char TimerFlag = 0;                                                                                     // Timer flag to indicate timer interrupts

void timerCallback(Timer_Handle myHandle, int_fast16_t status) {

    TimerFlag = 1;                                                                                                        // Set the TimerFlag when the timer interrupt occurs
}


void initTimer(void) {

    Timer_Params params;                                                                                                  // Timer parameters

    /* Init the driver */
    Timer_init();                                                                                                         // Initialize the Timer driver

   /*  Configure the Timer parameters */
    Timer_Params_init(&params);
    params.period = 1000000;                                                                                              // Set the timer period to 1 second
    params.timerMode = Timer_CONTINUOUS_CALLBACK;                                                                         // Set the timer mode to continuous with callbacks
    params.periodUnits = Timer_PERIOD_US;                                                                                 // Set the period units to microseconds
    params.timerCallback = timerCallback;                                                                                 // Set the timer callback function

    /* Open the Timer driver */
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {                                                                                                 // If the Timer initialization failed

        /* Failed to initialized timer */
        while (1){}                                                                                                       // Trap in an infinite loop

    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {                                                                      // If starting the Timer failed

        /* Failed to start timer */
        while (1){}                                                                                                       // Trap in an infinite loop

    }

}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {

    /* Call driver init functions */
    GPIO_init();                                                                                                          // Initialize the GPIO driver

    /* Configure the red LED */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);                                               // Configure the red LED as an output and set it low

    /* Configure BUTTON0 pin */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);                                       // Configure BUTTON0 pin as an input with pull-up and falling edge interrupt

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);                                                                    // Turn on the user LED

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);                                                               // Install the callback function for BUTTON0

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);                                                                                 // Enable interrupts for BUTTON0


    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {                                                                   // If there is more than one button, configure and enable interrupts for BUTTON1

        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);                                   // Configure BUTTON1 pin

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);                                                           // Install Button callback for BUTTON1

        /* Enable interrupts */
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);                                                                             // Enable interrupts for BUTTON1

    }


    /*
     *  ======== Main code ========
     */

    /*
     * Add flags (similar to timer flag) to button handlers
     * Refer to Zybooks - "Converting different-period tasks to C"
     * Remember to configure timer period (currently 100ms, or 100000us)
     */

    /* Initialize UART, I2C, and Timer drivers */
    initUART();
    initI2C();
    initTimer();

    /*  Initialize variables for button checking, temperature checking, and display updating */
    unsigned long buttonCheckTime = 0;                                                                                    // Init elapsed time for button
    unsigned long tempCheckTime = 0;                                                                                      // Init elapsed time for temp
    unsigned long displayCheckTime = 0;                                                                                   // Init elapsed time for display
    const unsigned long buttonCheckPeriod = 200;                                                                          // Init constant button period (200ms)
    const unsigned long tempCheckPeriod = 500;                                                                            // Init constant temp period (500ms)
    const unsigned long displayCheckPeriod = 1000;                                                                        // Init constant display period (1000ms)
    const unsigned long timerPeriod = 100;                                                                                // Init constant timer period/GCD (100ms)
    int setpoint = 25;                                                                                                    // Init setpoint (default thermostat value)
    int heat = 0;                                                                                                         // Init heat (0 is off, 1 is on)
    int seconds = 0;                                                                                                      // Init seconds
    int temperature = 0;                                                                                                  // Init temperature (temperature will equal ambient board temp)


    /*
     *  ======== Main while loop ========
     */

    while (1) {

        readTemp();                                                                                                       // Read the temperature

        /* Every 200ms, check button presses */
        if (buttonCheckTime >= buttonCheckPeriod)  {                                                                      // Check button presses every 200ms (check time equals or exceeds period)

            if (rightButton == 1) {                                                                                       // Button on right side raises setpoint (thermostat setting) by 1

                setpoint += 1;                                                                                            // Increment the setpoint if the right button is pressed
                rightButton = 0;                                                                                          // Reset the right button flag

            }


            if (leftButton == 1) {                                                                                        // Button on left side lowers setpoint (thermostat setting) by 1

                setpoint -= 1;                                                                                            // Decrement the setpoint if the left button is pressed
                leftButton = 0;                                                                                           // Reset the left button flag

            }

        }


        /* Every 500ms check temperature and update LED/heat */
        if (tempCheckTime >= tempCheckPeriod) {                                                                           // Temp check time equals or exceeds period


            temperature = readTemp();                                                                                     // Read the temperature

            if (temperature < setpoint) {                                                                                 // Temp lower than thermostat setting, heat/LED turns on


                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);                                                        // Turn on the LED if the temperature is below the setpoint
                heat = 1;                                                                                                 // Turn off the heat

            }

            else {                                                                                                        // Temp higher than thermostat setting, heat/LED turns off



                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);                                                       // Turn off the LED if the temperature is above the setpoint
                heat = 0;                                                                                                 // Turn off the heat

            }

        }


        /* Every 1000ms output to UART to display temperature */
        if (displayCheckTime >= displayCheckPeriod) {                                                                     // Display check time equals or exceeds period


            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))                // Display the temperature, setpoint, heat status, and seconds
            ++seconds;                                                                                                    // Increment the seconds counter

        }

        while (!TimerFlag){}                                                                                              // Wait for the timer period

        TimerFlag = 0;                                                                                                    // Reset the TimerFlag
        buttonCheckTime += timerPeriod;                                                                                   // Increment the button check time by 100ms
        tempCheckTime += timerPeriod;                                                                                     // Increment the temperature check time 100ms
        displayCheckTime += timerPeriod;                                                                                  // Increment the display check time by 100ms

    }

}
