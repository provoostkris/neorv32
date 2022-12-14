// #################################################################################################
// # << NEORV32: neorv32_slink.c - Stream Link Interface HW Driver >>                              #
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
 * @file neorv32_slink.c
 * @brief Stream Link Interface HW driver source file.
 **************************************************************************/
#include "neorv32.h"
#include "neorv32_slink.h"


/**********************************************************************//**
 * Check if stream link interface was synthesized.
 *
 * @return 0 if SLINK was not synthesized, 1 if SLINK is available.
 **************************************************************************/
int neorv32_slink_available(void) {

  if (NEORV32_SYSINFO.SOC & (1 << SYSINFO_SOC_IO_SLINK)) {
    return 1;
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Reset and enable SLINK.
 *
 * @param[in] irq_config Configure RX and TX interrupts per link (#NEORV32_SLINK_IRQ_enum).
 **************************************************************************/
void neorv32_slink_setup(uint32_t irq_config) {

  NEORV32_SLINK.CTRL = 0;
  NEORV32_SLINK.TX_STATUS = 0;
  NEORV32_SLINK.IRQ = irq_config;
  NEORV32_SLINK.CTRL = (1 << SLINK_CTRL_EN);
}


/**********************************************************************//**
 * Activate stream link interface.
 **************************************************************************/
void neorv32_slink_enable(void) {

  NEORV32_SLINK.CTRL |= (uint32_t)(1 << SLINK_CTRL_EN);
}


/**********************************************************************//**
 * Deactivate stream link interface.
 *
 * @note This will also clear all link FIFOs.
 **************************************************************************/
void neorv32_slink_disable(void) {

  NEORV32_SLINK.CTRL &= ~(uint32_t)(1 << SLINK_CTRL_EN);
}


/**********************************************************************//**
 * Get number of implemented RX/TX links
 *
 * @param[in] sel =0 -> RX, =!0 -> TX
 * @return Number of implemented RX/TX links (0..8).
 **************************************************************************/
int neorv32_slink_get_link_num(int sel) {

  if (neorv32_slink_available()) {
    uint32_t tmp = NEORV32_SLINK.CTRL;
    if (sel) {
      return (int)((tmp >> SLINK_CTRL_TX_NUM_LSB) & 0x0f);
    }
    else {
      return (int)((tmp >> SLINK_CTRL_RX_NUM_LSB) & 0x0f);
    }
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Get FIFO depth of RX/TX links
 *
 * @param[in] sel =0 -> RX, =!0 -> TX
 * @return FIFO depth of RX/TX links (1..32768); 0 if no RX/TX links implemented.
 **************************************************************************/
int neorv32_slink_get_fifo_depth(int sel) {

  if (neorv32_slink_available()) {
    uint32_t tmp = NEORV32_SLINK.CTRL;
    if (sel) {
      tmp = (tmp >> SLINK_CTRL_TX_FIFO_LSB) & 0x0f;
    }
    else {
      tmp = (tmp >> SLINK_CTRL_RX_FIFO_LSB) & 0x0f;
    }
    return (int)(1 << tmp);
  }
  else {
    return 0;
  }
}


/**********************************************************************//**
 * Write data to TX stream link i (non-blocking)
 *
 * @param[in] link_id Link ID (0..7).
 * @param[in] tx_data Data to send to link.
 * @param[in] last Set to 1 to indicate this is the "end of packet".
 * @return 0 if data was send, -1 if link is still busy (FIFO full)
 **************************************************************************/
int neorv32_slink_tx(int link_id, uint32_t tx_data, int last) {

  int link_sel = link_id & 7;

  if ((NEORV32_SLINK.TX_STATUS & (1 << (SLINK_TX_STATUS_FULL_LSB + link_sel))) == 0) { // free entry?
    NEORV32_SLINK.TX_STATUS = last << (SLINK_TX_STATUS_LAST_LSB + link_sel); // set if LAST, clear otherwise
    NEORV32_SLINK.DATA[link_sel] = tx_data;
    return 0;
  }
  else {
    return -1;
  }
}


/**********************************************************************//**
 * Read data from RX stream link i (non-blocking)
 *
 * @param[in] link_id Link ID (0..7).
 * @param[in] rx_data Pointer for received data.
 * @return 0 if data was received, -1 if no data available, +1 if data was received and
 * data was marked as "end of packet".
 **************************************************************************/
int neorv32_slink_rx(int link_id, uint32_t *rx_data) {

  int link_sel = link_id & 7;

  uint32_t tmp = NEORV32_SLINK.RX_STATUS; // read flags before doing a RX data access

  if ((tmp & (1 << (SLINK_RX_STATUS_EMPTY_LSB + link_sel))) == 0) { // data available?
    *rx_data = NEORV32_SLINK.DATA[link_sel];
    if (tmp & (1 << (SLINK_RX_STATUS_LAST_LSB + link_sel))) { // was "end of packet"?
      return +1;
    }
    else {
      return 0;
    }
  }
  else {
  }
    return -1;
}
