// #################################################################################################
// # << NEORV32 - PWM Demo Program >>                                                              #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file demo_pwm/main.c
 * @author Stephan Nolting
 * @brief Simple PWM demo program.
 **************************************************************************/

#include <neorv32.h>


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/** Maximum PWM output intensity (0..255) */
#define PWM_MAX 128
/**@}*/


/**********************************************************************//**
 * Helper function to count the number of implemented PWM channels.
 *
 * @return Number of available PWM channels (0..60)
 **************************************************************************/
int aux_get_pwm_num_channels(void) {

  if (neorv32_pwm_available() == 0) {
    return 0;
  }

  int i = 0;
  uint32_t tmp = 0;

  NEORV32_PWM.CTRL = 0; // disable PWM controller

  for (i=0; i<15; i++) {
    NEORV32_PWM.DUTY[i] = 0x01010101; // try to set all possible DUTY cycle values
  }

  for (i=0; i<15; i++) {
    tmp += NEORV32_PWM.DUTY[i]; // read back
  }

  // add four byte subwords
  tmp = ((tmp >>  0) & 0xff) + ((tmp >>  8) & 0xff) + ((tmp >> 16) & 0xff) + ((tmp >> 24) & 0xff);

  return (int)(tmp & 0xff);
}


/**********************************************************************//**
 * This program generates a simple dimming sequence for PWM channels 0 to 3.
 *
 * @note This program requires the PWM controller to be synthesized (the UART is optional).
 *
 * @return !=0 if error.
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // use UART0 if implemented
  if (neorv32_uart0_available()) {
    // init UART at default baud rate, no parity bits, no hw flow control
    neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

    // say hello
    neorv32_uart0_printf("PWM demo program\n");
  }

  // check if PWM unit is implemented at all
  if (neorv32_pwm_available() == 0) {
    if (neorv32_uart0_available()) {
      neorv32_uart0_printf("ERROR: PWM module not implemented!\n");
    }
    return 1;
  }

  int num_pwm_channels = aux_get_pwm_num_channels();

  // check number of PWM channels
  if (neorv32_uart0_available()) {
    neorv32_uart0_printf("Implemented PWM channels: %i\n\n", num_pwm_channels);
  }


  // deactivate all PWM channels
  int i;
  for (i=0; i<num_pwm_channels; i++) {
    neorv32_pwm_set(i, 0);
  }

  // configure and enable PWM
  neorv32_pwm_setup(CLK_PRSC_64);


  uint8_t pwm = 0;
  uint8_t up = 1;
  uint8_t ch = 0;

  // animate!
  while(1) {
  
    // update duty cycle
    if (up) {
      if (pwm == (PWM_MAX & 0xFF)) {
        up = 0;
      }
      else {
        pwm++;
      }
    }
    else {
      if (pwm == 0) {
        ch = (ch + 1) & 3; // goto next channel
        up = 1;
      }
      else {
        pwm--;
      }
    }

    neorv32_pwm_set(ch, pwm); // output new duty cycle

    neorv32_cpu_delay_ms(5); // wait ~5ms
  }

  return 0;
}
