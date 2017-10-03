/*
    ChibiOS - Copyright (C) 2015 RedoX https://github.com/RedoXyde

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config =
{
    .ports = {
        {
            /*
             * PORTA setup.
             */
            .port = IOPORT1,
            .pads = {
                PAL_MODE_ALTERNATIVE_7,   //*< PTA0 - SW D
                PAL_MODE_UNCONNECTED,     //*< PTA1 - SW D
                PAL_MODE_UNCONNECTED,     //*< PTA2 - SW D
                PAL_MODE_ALTERNATIVE_7,   //*< PTA3 - SW D
                PAL_MODE_UNCONNECTED,     //*< PTA4 - SW D
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA5 - TEENSY_PIN25
                PAL_MODE_UNCONNECTED,     //*< PTA6
                PAL_MODE_UNCONNECTED,     //*< PTA7
                PAL_MODE_UNCONNECTED,     //*< PTA8
                PAL_MODE_UNCONNECTED,     //*< PTA9
                PAL_MODE_UNCONNECTED,     //*< PTA10
                PAL_MODE_UNCONNECTED,     //*< PTA11
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA12 - TEENSY_PIN3
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA13 - TEENSY_PIN4
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA14 - TEENSY_PIN26
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA15 - TEENSY_PIN27
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA16 - TEENSY_PIN28
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA17 - TEENSY_PIN39
                PAL_MODE_INPUT_ANALOG,    //*< PTA18 - XTAL
                PAL_MODE_INPUT_ANALOG,    //*< PTA19 - XTAL
                PAL_MODE_UNCONNECTED,     //*< PTA20
                PAL_MODE_UNCONNECTED,     //*< PTA21
                PAL_MODE_UNCONNECTED,     //*< PTA22
                PAL_MODE_UNCONNECTED,     //*< PTA23
                PAL_MODE_UNCONNECTED,     //*< PTA24
                PAL_MODE_UNCONNECTED,     //*< PTA25
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA26 - TEENSY_PIN42
                PAL_MODE_UNCONNECTED,     //*< PTA27
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA28 - TEENSY_PIN40
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTA29 - TEENSY_PIN41
                PAL_MODE_UNCONNECTED,     //*< PTA30
                PAL_MODE_UNCONNECTED,     //*< PTA31
            },
        },
        {
            /*
             * PORTB setup.
             */
            .port = IOPORT2,
            .pads = {
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB0 - TEENSY_PIN16
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB1 - TEENSY_PIN17
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB2 - TEENSY_PIN19
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB3 - TEENSY_PIN18
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB4 - TEENSY_PIN49
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB5 - TEENSY_PIN50
                PAL_MODE_UNCONNECTED,     //*< PTB6
                PAL_MODE_UNCONNECTED,     //*< PTB7
                PAL_MODE_UNCONNECTED,     //*< PTB8
                PAL_MODE_UNCONNECTED,     //*< PTB9
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB10 - TEENSY_PIN31
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB11 - TEENSY_PIN32
                PAL_MODE_UNCONNECTED,     //*< PTB12
                PAL_MODE_UNCONNECTED,     //*< PTB13
                PAL_MODE_UNCONNECTED,     //*< PTB14
                PAL_MODE_UNCONNECTED,     //*< PTB15
                PAL_MODE_ALTERNATIVE_3,   //*< PTB16 - TEENSY_PIN0 - UART RX1
                PAL_MODE_ALTERNATIVE_3,   //*< PTB17 - TEENSY_PIN1 - UART TX1
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB18 - TEENSY_PIN29
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB19 - TEENSY_PIN30
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB20 - TEENSY_PIN43
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB21 - TEENSY_PIN46
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB22 - TEENSY_PIN44
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTB23 - TEENSY_PIN45
                PAL_MODE_UNCONNECTED,     //*< PTB24
                PAL_MODE_UNCONNECTED,     //*< PTB25
                PAL_MODE_UNCONNECTED,     //*< PTB26
                PAL_MODE_UNCONNECTED,     //*< PTB27
                PAL_MODE_UNCONNECTED,     //*< PTB28
                PAL_MODE_UNCONNECTED,     //*< PTB29
                PAL_MODE_UNCONNECTED,     //*< PTB30
                PAL_MODE_UNCONNECTED,     //*< PTB31
            },
        },
        {
            /*
             * PORTC setup.
             */
            .port = IOPORT3,
            .pads = {
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC0 - TEENSY_PIN15
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC1 - TEENSY_PIN22
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC2 - TEENSY_PIN23
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC3 - TEENSY_PIN9
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC4 - TEENSY_PIN10
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC5 - TEENSY_PIN13
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC6 - TEENSY_PIN11
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC7 - TEENSY_PIN12
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC8 - TEENSY_PIN35
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC9 - TEENSY_PIN36
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC10 - TEENSY_PIN37
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTC11 - TEENSY_PIN38
                PAL_MODE_UNCONNECTED,     //*< PTC12
                PAL_MODE_UNCONNECTED,     //*< PTC13
                PAL_MODE_UNCONNECTED,     //*< PTC14
                PAL_MODE_UNCONNECTED,     //*< PTC15
                PAL_MODE_UNCONNECTED,     //*< PTC16
                PAL_MODE_UNCONNECTED,     //*< PTC17
                PAL_MODE_UNCONNECTED,     //*< PTC18
                PAL_MODE_UNCONNECTED,     //*< PTC19
                PAL_MODE_UNCONNECTED,     //*< PTC20
                PAL_MODE_UNCONNECTED,     //*< PTC21
                PAL_MODE_UNCONNECTED,     //*< PTC22
                PAL_MODE_UNCONNECTED,     //*< PTC23
                PAL_MODE_UNCONNECTED,     //*< PTC24
                PAL_MODE_UNCONNECTED,     //*< PTC25
                PAL_MODE_UNCONNECTED,     //*< PTC26
                PAL_MODE_UNCONNECTED,     //*< PTC27
                PAL_MODE_UNCONNECTED,     //*< PTC28
                PAL_MODE_UNCONNECTED,     //*< PTC29
                PAL_MODE_UNCONNECTED,     //*< PTC30
                PAL_MODE_UNCONNECTED,     //*< PTC31
            },
        },
        {
            /*
             * PORTD setup.
             */
            .port = IOPORT4,
            .pads = {
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD0 - TEENSY_PIN2
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD1 - TEENSY_PIN14
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD2 - TEENSY_PIN7
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD3 - TEENSY_PIN8
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD4 - TEENSY_PIN6
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD5 - TEENSY_PIN20
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD6 - TEENSY_PIN21
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD7 - TEENSY_PIN5
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD8 - TEENSY_PIN47
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD9 - TEENSY_PIN48
                PAL_MODE_UNCONNECTED,     //*< PTD10
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD11 - TEENSY_PIN55
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD12 - TEENSY_PIN53
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD13 - TEENSY_PIN52
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD14 - TEENSY_PIN51
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTD15 - TEENSY_PIN54
                PAL_MODE_UNCONNECTED,     //*< PTD16
                PAL_MODE_UNCONNECTED,     //*< PTD17
                PAL_MODE_UNCONNECTED,     //*< PTD18
                PAL_MODE_UNCONNECTED,     //*< PTD19
                PAL_MODE_UNCONNECTED,     //*< PTD20
                PAL_MODE_UNCONNECTED,     //*< PTD21
                PAL_MODE_UNCONNECTED,     //*< PTD22
                PAL_MODE_UNCONNECTED,     //*< PTD23
                PAL_MODE_UNCONNECTED,     //*< PTD24
                PAL_MODE_UNCONNECTED,     //*< PTD25
                PAL_MODE_UNCONNECTED,     //*< PTD26
                PAL_MODE_UNCONNECTED,     //*< PTD27
                PAL_MODE_UNCONNECTED,     //*< PTD28
                PAL_MODE_UNCONNECTED,     //*< PTD29
                PAL_MODE_UNCONNECTED,     //*< PTD30
                PAL_MODE_UNCONNECTED,     //*< PTD31
            },
        },
        {
            /*
             * PORTE setup.
             */
            .port = IOPORT5,
            .pads = {
                PAL_MODE_UNCONNECTED,     //*< PTE0 - SD8 - DAT1
                PAL_MODE_UNCONNECTED,     //*< PTE1 - SD7 - DAT0
                PAL_MODE_UNCONNECTED,     //*< PTE2 - SD5 - CLK
                PAL_MODE_UNCONNECTED,     //*< PTE3 - SD3 - CMD
                PAL_MODE_UNCONNECTED,     //*< PTE4 - SD2 - CD/DAT3
                PAL_MODE_UNCONNECTED,     //*< PTE5 - SD1 - DAT2
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTE6 - USB ENABLE
                PAL_MODE_UNCONNECTED,     //*< PTE7
                PAL_MODE_UNCONNECTED,     //*< PTE8
                PAL_MODE_UNCONNECTED,     //*< PTE9
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTE10 - TEENSY_PIN56
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTE11 - TEENSY_PIN57
                PAL_MODE_UNCONNECTED,     //*< PTE12
                PAL_MODE_UNCONNECTED,	  //*< PTE13
                PAL_MODE_UNCONNECTED,	  //*< PTE14
                PAL_MODE_UNCONNECTED,	  //*< PTE15
                PAL_MODE_UNCONNECTED,	  //*< PTE16
                PAL_MODE_UNCONNECTED,	  //*< PTE17
                PAL_MODE_UNCONNECTED,	  //*< PTE18
                PAL_MODE_UNCONNECTED,	  //*< PTE19
                PAL_MODE_UNCONNECTED,	  //*< PTE20
                PAL_MODE_UNCONNECTED,	  //*< PTE21
                PAL_MODE_UNCONNECTED,	  //*< PTE22
                PAL_MODE_UNCONNECTED,	  //*< PTE23
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTE24 - TEENSY_PIN33
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTE25 - TEENSY_PIN34
                PAL_MODE_OUTPUT_PUSHPULL, //*< PTE26 - TEESNY_PIN24
                PAL_MODE_UNCONNECTED,	  //*< PTE27
                PAL_MODE_UNCONNECTED,	  //*< PTE28
                PAL_MODE_UNCONNECTED,	  //*< PTE29
                PAL_MODE_UNCONNECTED,	  //*< PTE30
                PAL_MODE_UNCONNECTED,	  //*< PTE31
            },
        },
    },
};
#endif

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {

  k64_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
}
