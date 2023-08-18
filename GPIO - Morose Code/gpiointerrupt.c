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
#include <stdint.h>                                                                      // Include standard integer types
#include <stddef.h>                                                                      // Include standard definition types

#include <stdio.h>                                                                       // Include standard input/output library

/* Driver Header files */
#include <ti/drivers/GPIO.h>                                                             // Include Texas Instruments' GPIO driver
#include <ti/drivers/Timer.h>                                                            // Include Texas Instruments' Timer driver

/* Driver configuration */
#include "ti_drivers_config.h"                                                           // Include specific configuration for TI drivers

#include <unistd.h>                                                                      // Include standard symbolic constants and types



void sleep_ms(int milliseconds) {                                                        // Define a function to sleep for a given number of milliseconds

    usleep(milliseconds * 1000);                                                         // Convert milliseconds to microseconds and call usleep function

}



void dot() {                                                                             // Define a function to represent a Morse code "dot"

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);                                   // Turn ON LED 0
    usleep(500000);                                                                      // Sleep for 500 milliseconds
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);                                  // Turn OFF LED 0
    usleep(500000);                                                                      // Sleep for 500 milliseconds

}

void dash() {                                                                            // Define a function to represent a Morse code "dash"

    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);                                   // Turn ON LED 1
    usleep(1500000);                                                                     // Sleep for 1500 milliseconds
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);                                  // Turn OFF LED 1
    usleep(500000);                                                                      // Sleep for 500 milliseconds

}


static bool pending_sos_state = true;                                                    // Define a static variable for pending SOS state
static bool sos_state = true;                                                            // Define a static variable for current SOS state
static int sos_char_count = 0;                                                           // Define a static variable for SOS character count


void timerCallback(Timer_Handle myHandle, int_fast16_t status) {                         // Define a timer callback function

    printf("timerCallback called\n");                                                    // Print a message indicating the callback was called

    /* Toggle between SOS and OK */
    if (GPIO_read(CONFIG_GPIO_BUTTON_0) || GPIO_read(CONFIG_GPIO_BUTTON_1)) {            // Check if either button is pressed

        printf("Button pressed\n");                                                      // Print a message indicating a button press
        pending_sos_state = !sos_state;                                                  // Toggle the pending SOS state
        sos_char_count = 0;                                                              // Reset the SOS character count

    }

    /* Check if the current Morse code sequence has finished */
    if (sos_char_count == 0) {                                                           // Check if the SOS character count is zero

        /* If so, update sos_state with the value of pending_sos_state */
        sos_state = pending_sos_state;                                                   // Update the current SOS state
    
    }

    if (sos_state) {                                                                     // Check if the current state is SOS

        /* SOS */
        if (sos_char_count == 0) {                                                       // Check if the SOS character count is zero (first character)

            /* S */
            dot(); dot(); dot();                                                         // Call the dot function three times for "S"
            sleep_ms(1500);                                                              // Wait for 1500 milliseconds between letters

        } 
        
        else if (sos_char_count == 1) {                                                  // Check if the SOS character count is one (second character)

            /* O */
            dash(); dash(); dash();                                                      // Call the dash function three times for "O"
            sleep_ms(1500);                                                              // Wait for 1500 milliseconds between letters

        } 
        
        else if (sos_char_count == 2) {                                                  // Check if the SOS character count is two (third character)

            /* S */
            dot(); dot(); dot();                                                         // Call the dot function three times for "S"

        }

    } 
    
    else {                                                                               // If the current state is not SOS (it's OK)

        /* OK */
        if (sos_char_count == 0) {                                                       // Check if the SOS character count is zero (first character)
           
            /* O */
            dash(); dash(); dash();                                                      // Call the dash function three times for "O"
            sleep_ms(1500);                                                              // Wait for 1500 milliseconds between letters

        } 
        
        else if (sos_char_count == 1) {                                                  // Check if the SOS character count is one (second character)

            /* K */
            dash(); dot(); dash();                                                       // Call the dash-dot-dash sequence for "K"

        }

    }

    sos_char_count++;                                                                    // Increment the SOS character count

    if (sos_char_count == 3) {                                                           // Check if the SOS character count reaches three

        sos_char_count = 0;                                                              // Reset the SOS character count
        sleep_ms(3500);                                                                  // Wait for 3500 milliseconds between words

    }

    /* Restart the timer */
    Timer_start(myHandle);                                                               // Restart the timer

}


void initTimer(void) {                                                                   // Define a function to initialize the timer

    Timer_Handle timer0;                                                                 // Declare a handle for the timer
    Timer_Params params;                                                                 // Declare parameters for the timer
    Timer_init();                                                                        // Initialize the timer
    Timer_Params_init(&params);                                                          // Initialize the timer parameters
    params.period = 500000;                                                              // Set the timer period to 500,000 microseconds
    params.periodUnits = Timer_PERIOD_US;                                                // Set the period units to microseconds
    params.timerMode = Timer_CONTINUOUS_CALLBACK;                                        // Set the timer mode to continuous callback
    params.timerCallback = timerCallback;                                                // Set the timer callback function

    timer0 = Timer_open(CONFIG_TIMER_0, &params);                                        // Open the timer with the specified parameters

    if (timer0 == NULL) {                                                                // Check if the timer initialization failed

        /* Failed to initialize timer */
        while (1) {}                                                                     // Infinite loop if initialization fails

    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {                                     // Check if starting the timer fails

        /* Failed to start timer */
        while (1) {}                                                                     // Infinite loop if start fails

    }

}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index) {                                               // Define callback function for GPIO button 0

    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0); // Code commented out to toggle an LED
    
    /* Set the pending sos_state variable */
    pending_sos_state = !sos_state;                                                      // Toggle the pending SOS state

}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index) {                                               // Define callback function for GPIO button 1

    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1); // Code commented out to toggle an LED
    
    /* Set the pending sos_state variable */
    pending_sos_state = !sos_state;                                                      // Toggle the pending SOS state

}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {                                                           // Define the main thread function

    /* Call driver init functions */
    GPIO_init();                                                                         // Initialize the GPIO driver

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);              // Configure LED 0 as output, initially low
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);              // Configure LED 1 as output, initially low
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);      // Configure button 0 as input with pull-up and falling edge interrupt

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);                                   // Turn ON LED 0

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);                              // Set the callback function for button 0

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);                                                // Enable interrupts for button 0

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {                                  // Check if there is more than one input pin
    
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);  // Configure button 1 as input with pull-up and falling edge interrupt

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);                          // Set the callback function for button 1
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);                                            // Enable interrupts for button 1
    
    }

    /* Initialize and start the timer */
    initTimer();                                                                         // Call the function to initialize and start the timer

    /* Main loop */
    while (1) {                                                                          // Infinite loop
    
        if (GPIO_read(CONFIG_GPIO_BUTTON_0) || GPIO_read(CONFIG_GPIO_BUTTON_1)) {        // Check if either button is pressed
    
            printf("Button pressed\n");                                                  // Print a message indicating a button press
            sos_state = !sos_state;                                                      // Toggle the current SOS state
            sos_char_count = 0;                                                          // Reset the SOS character count
    
        }
    
    }

    return (NULL);                                                                       // Return NULL from the main thread function

}
