
#include <ti/devices/msp/msp.h>
#include "lab6_helper.h"
#include <stdio.h>

/*
 * Servo code
 */
#define PERIOD 20000
#define EXTEND_FACTOR 1.6
#define FULL_EXTENDED_FACTOR 1.5
#define RETRACT_FACTOR 2.5
void extendServo(){
    TIMA1->COUNTERREGS.LOAD = PERIOD;
    TIMA1->COUNTERREGS.CC_01[1] = (TIMA1->COUNTERREGS.LOAD + 1) / EXTEND_FACTOR;
    TIMA1->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);

}
void captureExtendServo(){
    TIMA1->COUNTERREGS.LOAD = PERIOD;
        TIMA1->COUNTERREGS.CC_01[1] = (TIMA1->COUNTERREGS.LOAD + 1) / FULL_EXTENDED_FACTOR;
        TIMA1->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);
}

void retractServo(){
    TIMA1->COUNTERREGS.LOAD = PERIOD;
    TIMA1->COUNTERREGS.CC_01[1] = (TIMA1->COUNTERREGS.LOAD + 1) / RETRACT_FACTOR;
    TIMA1->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);
}


//anti debounce for limit switch and start switch
const int max_debounce_duration = 1000;
int curr_debounce_duration = 1;

const int servo_wait_time = 1500;
int curr_servo_wait_time = 1;
enum current_state_enum {
    PRE_START = 0,
    PRE_TAKEOFF = 1,
    IN_FLIGHT = 2,
    DELIVERED = 3,
    CAPTURED = 4
};



/*
 * defining Neopixel LED Packet
 *
 * 24 LEDs
 * --> 24 * 24 'bytes' = uint16_t must be 288 elements long
 */
uint32_t ledValue = 0x0a0011;
uint16_t rValue = 0xFF;
uint16_t gValue = 0x00;
uint16_t bValue = 0x00;

uint16_t packet[288];


uint16_t *txPacket;

int transmissionComplete = 0; // flag for SPI ISR wakeup
int timerTicked = 0; // flag for timer ISR wakeup
int idx = 0;

int message_len = sizeof(packet) / sizeof(packet[0]);



int main(void)
{
    InitializeProcessor();
    InitializeGPIO();
    InitializeSPI();

    InitializeTimerG0();
    InitializeTimerA1_PWM();

    // let the buzzer run for 0.1 s just so we know it's there!
    delay_cycles(1600000);
    TIMA1->COUNTERREGS.CTRCTL &= ~(GPTIMER_CTRCTL_EN_ENABLED); // Disable the buzzer

    NVIC_EnableIRQ(TIMG0_INT_IRQn); // Enable the timer interrupt

    TIMG0->COUNTERREGS.LOAD = 327; // 10 milliseconds
    TIMG0->COUNTERREGS.CTRCTL |= (GPTIMER_CTRCTL_EN_ENABLED);

    uint8_t lightState = 0;

    enum current_state_enum next_state;
    enum current_state_enum prev_state;
    next_state = PRE_START;
    prev_state = PRE_START;


    while (1) { // this loop will execute once per timer interrupt
        switch (next_state) {
            case PRE_START:
                /*
                 * Update the color of the LEDs
                 */
                switch (lightState) {
                    case 0:
                        gValue+=2;
                        rValue-=2;
                        if (gValue >= 0xFA) {
                            lightState = 1;
                            rValue = 0x00;
                            bValue = 0x00;
                        }
                        break;
                    case 1:
                        gValue-=2;
                        bValue+=2;
                        if (bValue >= 0xFA) {
                            lightState = 2;
                            gValue = 0x00;
                            rValue = 0x00;
                        }
                        break;
                    case 2:
                        bValue-=2;
                        rValue+=2;
                        if (rValue >= 0xFA) {
                            lightState = 0;
                            bValue = 0x00;
                            gValue = 0x00;
                        }
                }
                ledValue = gValue << 16 | rValue << 8 | bValue;


                //extends the servo
                extendServo();

                //checks if the arming button has been pressed
                if ((GPIOA->DIN31_0 & START_SW) != START_SW){
                    next_state = PRE_TAKEOFF;
                }

                else{

                    next_state = PRE_START;
                }

                break;

            case PRE_TAKEOFF:
                //if the limit switch is released, we are airbone
                if ((GPIOA->DIN31_0 & LIMIT_SW) != LIMIT_SW){
                    next_state = PRE_TAKEOFF;
                }

                else{

                    next_state = IN_FLIGHT;
                }
                ledValue = 0xA00000; // set the LEDs to green
                break;

            case IN_FLIGHT:
                ledValue = 0x00A000; // set the LEDs to red

                //If the limit switch has been pressed, we have landed
                if ((GPIOA->DIN31_0 & LIMIT_SW) != LIMIT_SW){
                    next_state = DELIVERED;
                    ledValue = 0x000000; // turn the LEDs off
                   }
                else{
                    next_state = IN_FLIGHT;
                }
                break;

            case DELIVERED:
                //if the limit switch is released, we have been captured
                if ((GPIOA->DIN31_0 & LIMIT_SW) != LIMIT_SW){
                    next_state = DELIVERED;
                    if (curr_servo_wait_time < servo_wait_time){
                        curr_servo_wait_time++;
                        retractServo(); // retract the servo to deploy
                    }
                    else{

                        captureExtendServo();
                    }
                }
                else{
                    next_state = CAPTURED;
                }
                break;

            case CAPTURED:
                default:
                    // retract the servo back to the start position
                    extendServo();
                    ledValue = 0x0000A0; // set the LEDs to blue
                break;
        }


        /*
         * Write to packet to set the values for LEDs
         */
        for (int i = 0; i < message_len/12; i++) {
            for (int j = 0; j < 12; j++) {
                if ((ledValue & 0x1 << (j*2)) >> j*2 == 0x1) {
                    packet[i*12+(11-j)] = 0b11111100;
                } else {
                    packet[i*12+(11-j)] = 0b10000000;
                }
                if ((ledValue & 0x1 << (j*2+1)) >> (j*2+1) == 0x1) {
                    packet[i*12+(11-j)] = packet[i*12+(11-j)] | 0b11111100 << 8;
                } else {
                    packet[i*12+(11-j)] = packet[i*12+(11-j)] | 0b10000000 << 8;
                }
            }
        }

        txPacket = packet;

        // Clear pending SPI interrupts
        NVIC_ClearPendingIRQ(SPI0_INT_IRQn);
        NVIC_EnableIRQ(SPI0_INT_IRQn);
        transmissionComplete = 0; // reset flag
        idx = 1; // reset pointer to point at the second element of the SPI message
        SPI0->TXDATA = *txPacket; // This will start TX ISR running.
        // It will stop itself at the end of the message, and disable SPI interrupts.



        while (!timerTicked) // Wait for timer wake up
            __WFI();

        timerTicked = 0; // reset timer interupt flag
    }
}

void SPI0_IRQHandler(void)
{
    switch (SPI0->CPU_INT.IIDX) {
        case SPI_CPU_INT_IIDX_STAT_TX_EVT: // SPI interrupt index for transmit FIFO
            SPI0->TXDATA = txPacket[idx];
            idx++;
            if (idx == message_len) {
               transmissionComplete = 1; 
               NVIC_DisableIRQ(SPI0_INT_IRQn);
            }
            break;
        default:
            break;
    }
}


void TIMG0_IRQHandler(void)
{
    // This wakes up the processor!

    switch (TIMG0->CPU_INT.IIDX) {
        case GPTIMER_CPU_INT_IIDX_STAT_Z: // Counted down to zero event.
            timerTicked = 1; // set a flag so we can know what woke us up.
            break;
        default:
            break;
    }
}

/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
