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
#include "ch.h"
#include "hal.h"

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * the LED integrated into the board.
 */
static THD_WORKING_AREA(waBlinker, 64);
static THD_FUNCTION(thBlinker, arg) {
  (void)arg;
  chRegSetThreadName("Blinker");
  while (true) {
    palTogglePad(TEENSY_PIN13_IOPORT, TEENSY_PIN13);
    chThdSleepMilliseconds(500);
  }
}

static msg_t sdPutHexTimeout(SerialDriver *ip, uint8_t b, systime_t time)
{
  const uint8_t hex[] = "0123456789ABCDEF";
  msg_t res = chnPutTimeout(ip, hex[b>>4], time);
  if (res != MSG_OK)
    return res;

  return chnPutTimeout(ip, hex[b&0xF], time);
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  chThdCreateStatic(waBlinker, sizeof(waBlinker), NORMALPRIO, thBlinker, NULL);

  /*
   * Activates serial 1 (UART0) using the s0cfg configuration
   */
  const SerialConfig s0cfg = {
    19200                       /* baud rate */
  };
  palSetPadMode(TEENSY_PIN0_IOPORT, TEENSY_PIN0, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN1_IOPORT, TEENSY_PIN1, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD1, &s0cfg);
  {
    const uint8_t msg[] = "Hello!\r\n";
    sdWrite(&SD1, msg, sizeof(msg)-1);
  }

  /*
   * Start the I2C driver 0.
   */
  const I2CConfig i2cconfig = {
    .clock = 400000UL,          /* 400Khz clock */
  };

#if KINETIS_I2C_USE_I2C0
#define I2CD I2CD1
  palSetPadMode(TEENSY_PIN18_IOPORT, TEENSY_PIN18, PAL_MODE_ALTERNATIVE_2);
  palSetPadMode(TEENSY_PIN19_IOPORT, TEENSY_PIN19, PAL_MODE_ALTERNATIVE_2);
  /* alternative configurations */
  /* palSetPadMode(TEENSY_PIN16_IOPORT, TEENSY_PIN16, PAL_MODE_ALTERNATIVE_2); */
  /* palSetPadMode(TEENSY_PIN17_IOPORT, TEENSY_PIN17, PAL_MODE_ALTERNATIVE_2); */
  /* palSetPadMode(TEENSY_PIN33_IOPORT, TEENSY_PIN33, PAL_MODE_ALTERNATIVE_5); */
  /* palSetPadMode(TEENSY_PIN34_IOPORT, TEENSY_PIN34, PAL_MODE_ALTERNATIVE_5); */
  /* palSetPadMode(TEENSY_PIN7_IOPORT, TEENSY_PIN7, PAL_MODE_ALTERNATIVE_7); */
  /* palSetPadMode(TEENSY_PIN8_IOPORT, TEENSY_PIN8, PAL_MODE_ALTERNATIVE_7); */
#endif

#if KINETIS_I2C_USE_I2C1
#define I2CD I2CD2
  palSetPadMode(TEENSY_PIN37_IOPORT, TEENSY_PIN37, PAL_MODE_ALTERNATIVE_2);
  palSetPadMode(TEENSY_PIN38_IOPORT, TEENSY_PIN38, PAL_MODE_ALTERNATIVE_2);
#endif

#if KINETIS_I2C_USE_I2C2
#define I2CD I2CD3
  palSetPadMode(TEENSY_PIN3_IOPORT, TEENSY_PIN3, PAL_MODE_ALTERNATIVE_5);
  palSetPadMode(TEENSY_PIN4_IOPORT, TEENSY_PIN4, PAL_MODE_ALTERNATIVE_5);
#endif

  i2cStart(&I2CD, &i2cconfig);
  {
    const uint8_t msg[] = "I2C port configured\r\n";
    sdWrite(&SD1, msg, sizeof(msg)-1);
  }

  /* set the read address */
  uint16_t addr = 0;

  /* the two bytes are inverted but here it doesn't matter */
  msg_t res;
  while ((res = i2cMasterTransmitTimeout(
            &I2CD, 0x50, (uint8_t*)&addr, sizeof(addr), NULL, 0, 1000)) != MSG_OK) {
    const uint8_t error[] = "cannot send the address\r\n";
    sdWrite(&SD1, error, sizeof(error)-1);
    chThdSleepMilliseconds(1000);
  }

  uint8_t data[16];
  while (!chThdShouldTerminateX()) {
    res = i2cMasterReceive(&I2CD, 0x50, data, sizeof(data));
    if (res != MSG_OK) {
      const uint8_t error[] = "cannot read the data\r\n";
      sdWrite(&SD1, error, sizeof(error)-1);
    } else {
      /* address */
      sdPutHexTimeout(&SD1, addr >> 8, TIME_INFINITE);
      sdPutHexTimeout(&SD1, addr, TIME_INFINITE);
      sdWrite(&SD1, (const uint8_t *)": ", 2);

      /* content */
      for (unsigned i = 0; i < sizeof(data); i++) {
        sdPutHexTimeout(&SD1, data[i], TIME_INFINITE);
        chnPutTimeout(&SD1, ' ', TIME_INFINITE);
      }

      /* newline */
      sdWrite(&SD1, (const uint8_t *)"\r\n", 2);
    }

    addr += sizeof(data);
    chThdSleepMilliseconds(1000);
  }

  return 0;
}
