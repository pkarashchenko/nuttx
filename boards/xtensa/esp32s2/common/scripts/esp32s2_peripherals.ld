/****************************************************************************
 * boards/xtensa/esp32s2/common/scripts/esp32s2_peripherals.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

PROVIDE ( UART0 = 0x3f400000 );
PROVIDE ( SPIMEM1 = 0x3f402000 );
PROVIDE ( SPIMEM0 = 0x3f403000 );
PROVIDE ( GPIO = 0x3f404000 );
PROVIDE ( SIGMADELTA = 0x3f404f00 );
PROVIDE ( RTCCNTL = 0x3f408000 );
PROVIDE ( RTCIO = 0x3f408400 );
PROVIDE ( SENS = 0x3f408800 );
PROVIDE ( HINF = 0x3f40b000 );
PROVIDE ( I2S0 = 0x3f40F000 );
PROVIDE ( UART1 = 0x3f410000 );
PROVIDE ( I2C0 = 0x3f413000 );
PROVIDE ( UHCI0 = 0x3f414000 );
PROVIDE ( HOST = 0x3f415000 );
PROVIDE ( RMT = 0x3f416000 );
PROVIDE ( RMTMEM = 0x3f416400 );
PROVIDE ( PCNT = 0x3f417000 );
PROVIDE ( SLC = 0x3f418000 );
PROVIDE ( LEDC = 0x3f419000 );
PROVIDE ( CP_DMA = 0x3f4c3000 );
PROVIDE ( TIMERG0 = 0x3f41f000 );
PROVIDE ( TIMERG1 = 0x3f420000 );
PROVIDE ( SPI2 = 0x3f424000 );
PROVIDE ( SPI3 = 0x3f425000 );
PROVIDE ( SYSCON = 0x3f426000 );
PROVIDE ( I2C1 = 0x3f427000 );
PROVIDE ( APB_SARADC = 0x3f440000 );
PROVIDE ( DEDIC_GPIO = 0x3f4cf000 );
PROVIDE ( TWAI = 0x6002B000 );
PROVIDE ( USB0     = 0x60080000 );
PROVIDE ( USBH     = 0x60080000 );
PROVIDE ( USB_WRAP = 0x3f439000 );
