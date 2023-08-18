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
#include <stdint.h>
#include <stddef.h>

#include <stdio.h>


/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"


#include <unistd.h>


void sleep_ms(int milliseconds)
{
    usleep(milliseconds * 1000); // Convert to microseconds
}


void dot() {
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    usleep(500000); // 500 milliseconds = 500,000 microseconds
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    usleep(500000);
}

void dash() {
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
    usleep(1500000); // 1500 milliseconds = 1,500,000 microseconds
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    usleep(500000);
}

static bool pending_sos_state = true;
static bool sos_state = true;
static int sos_char_count = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    printf("timerCallback called\n");

    // Toggle between SOS and OK
    if (GPIO_read(CONFIG_GPIO_BUTTON_0) || GPIO_read(CONFIG_GPIO_BUTTON_1)) {
        printf("Button pressed\n");
        pending_sos_state = !sos_state; // Change this line
        sos_char_count = 0;
    }

    // Check if the current Morse code sequence has finished
    if (sos_char_count == 0) {
        // If so, update sos_state with the value of pending_sos_state
        sos_state = pending_sos_state;
    }

    if (sos_state) {
        // SOS
        if (sos_char_count == 0) {
            // S
            dot(); dot(); dot();
            sleep_ms(1500); // Wait for 3*500ms between letters
        } else if (sos_char_count == 1) {
            // O
            dash(); dash(); dash();
            sleep_ms(1500); // Wait for 3*500ms between letters
        } else if (sos_char_count == 2) {
            // S
            dot(); dot(); dot();
        }
    } else {
        // OK
        if (sos_char_count == 0) {
            // O
            dash(); dash(); dash();
            sleep_ms(1500); // Wait for 3*500ms between letters
        } else if (sos_char_count == 1) {
            // K
            dash(); dot(); dash();
        }
    }

    sos_char_count++;
    if (sos_char_count == 3) {
        sos_char_count = 0;
        sleep_ms(3500); // Wait for 7*500ms between words
    }

    // Restart the timer
    Timer_start(myHandle);
}




void initTimer(void) {

    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {

        /* Failed to initialized timer */
        while (1) {}

    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {

        /* Failed to start timer */
        while (1) {}

    }

}



/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);

    /* Set the pending sos_state variable */
    pending_sos_state = !sos_state;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);

    /* Set the pending sos_state variable */
    pending_sos_state = !sos_state;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    /* Initialize and start the timer */
    initTimer();

    /* Main loop */
    while (1) {
        if (GPIO_read(CONFIG_GPIO_BUTTON_0) || GPIO_read(CONFIG_GPIO_BUTTON_1)) {
            printf("Button pressed\n");
            sos_state = !sos_state;
            sos_char_count = 0;
        }
    }

    return (NULL);
}
