/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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
#include "ch_test.h"

/*
 * LED blinker thread.
 */
static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {
    (void)arg;
    chRegSetThreadName("LEDBlinker");
    while (true) {
#if KINETIS_USB_USE_USB0
      extern SerialUSBConfig serusbcfg;
      systime_t time = serusbcfg.usbp->state == USB_ACTIVE ? 150 : 500;
#else
      systime_t time = 500;
#endif
      palTogglePad(TEENSY_PIN13_IOPORT, TEENSY_PIN13);
      chThdSleepMilliseconds(time);
    }
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

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Activates serial 1 (UART0) using the driver default configuration.
   */
#if KINETIS_SERIAL_USE_UART0
  palSetPadMode(TEENSY_PIN0_IOPORT, TEENSY_PIN0, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN1_IOPORT, TEENSY_PIN1, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD1, NULL);
  test_execute((BaseSequentialStream *)&SD1);
#endif

#if KINETIS_SERIAL_USE_UART1
  palSetPadMode(TEENSY_PIN9_IOPORT, TEENSY_PIN9, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN10_IOPORT, TEENSY_PIN10, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD2, NULL);
  test_execute((BaseSequentialStream *)&SD2);
#endif

#if KINETIS_SERIAL_USE_UART2
  palSetPadMode(TEENSY_PIN7_IOPORT, TEENSY_PIN7, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN8_IOPORT, TEENSY_PIN8, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD3, NULL);
  test_execute((BaseSequentialStream *)&SD3);
#endif

#if KINETIS_SERIAL_USE_UART3
  palSetPadMode(TEENSY_PIN31_IOPORT, TEENSY_PIN31, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN32_IOPORT, TEENSY_PIN32, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD4, NULL);
  test_execute((BaseSequentialStream *)&SD4);
#endif

#if KINETIS_SERIAL_USE_UART4
  palSetPadMode(TEENSY_PIN33_IOPORT, TEENSY_PIN33, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN34_IOPORT, TEENSY_PIN34, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD5, NULL);
  test_execute((BaseSequentialStream *)&SD5);
#endif

#if KINETIS_SERIAL_USE_UART5
  palSetPadMode(TEENSY_PIN47_IOPORT, TEENSY_PIN47, PAL_MODE_ALTERNATIVE_3);
  palSetPadMode(TEENSY_PIN48_IOPORT, TEENSY_PIN48, PAL_MODE_ALTERNATIVE_3);
  sdStart(&SD6, NULL);
  test_execute((BaseSequentialStream *)&SD6);
#endif

#if KINETIS_USB_USE_USB0
  extern const USBConfig usbcfg;
  extern SerialUSBConfig serusbcfg;
  extern SerialUSBDriver SDU1;
  /*
   * Initialize and Activate SDU1 (USB CDC ACM).
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
  test_execute((BaseSequentialStream *)&SDU1);
#endif

  while (true) {
      chThdSleepMilliseconds(1000);
  }

  return 0;
}
