/*
    ChibiOS - (C) 2015-2016 flabbergast <s3+flabbergast@sdfeu.org>

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

#ifndef _MCUCONF_H_
#define _MCUCONF_H_

#define K64_MCUCONF

/*
 * HAL driver system settings.
 */
/* PEE mode - 48MHz system clock driven by external crystal. */
#define KINETIS_MCG_MODE            KINETIS_MCG_MODE_PEE
#define KINETIS_SYSCLK_FREQUENCY    120000000UL
#define KINETIS_BUSCLK_FREQUENCY    60000000UL
#define KINETIS_FLASHCLK_FREQUENCY  24000000UL

/*
 * SERIAL driver system settings.
 */
#define KINETIS_SERIAL_USE_UART0    FALSE

/*
 * USB driver settings
 */
#define KINETIS_USB_USE_USB0        TRUE

#endif /* _MCUCONF_H_ */
