-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: 2-to-1 Bus Switch                    --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_bus_switch is
  generic (
    ROUND_ROBIN_EN   : boolean := false; -- enable round-robing scheduling
    PORT_A_READ_ONLY : boolean := false; -- set if port A is read-only
    PORT_B_READ_ONLY : boolean := false  -- set if port B is read-only
  );
  port (
    clk_i    : in  std_ulogic; -- global clock, rising edge
    rstn_i   : in  std_ulogic; -- global reset, low-active, async
    a_lock_i : in  std_ulogic; -- exclusive access for port A while set
    a_req_i  : in  bus_req_t;  -- host port A request bus
    a_rsp_o  : out bus_rsp_t;  -- host port A response bus
    b_req_i  : in  bus_req_t;  -- host port B request bus
    b_rsp_o  : out bus_rsp_t;  -- host port B response bus
    x_req_o  : out bus_req_t;  -- device port request bus
    x_rsp_i  : in  bus_rsp_t   -- device port response bus
  );
end neorv32_bus_switch;

architecture neorv32_bus_switch_rtl of neorv32_bus_switch is

  -- access arbiter --
  type state_t is (S_CHECK_A, S_BUSY_A, S_CHECK_B, S_BUSY_B);
  signal state, state_nxt : state_t;
  signal a_req, b_req     : std_ulogic;
  signal sel,   stb       : std_ulogic;

begin

  -- Access Arbiter -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state <= S_CHECK_A;
      a_req <= '0';
      b_req <= '0';
    elsif rising_edge(clk_i) then
      state <= state_nxt;
      if (state = S_BUSY_A) then -- clear request
        a_req <= '0';
      else -- buffer request
        a_req <= a_req or a_req_i.stb;
      end if;
      if (state = S_BUSY_B) then -- clear request
        b_req <= '0';
      else -- buffer request
        b_req <= b_req or b_req_i.stb;
      end if;
    end if;
  end process arbiter_sync;


  -- Prioritizing Bus Switch ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_prioritized:
  if not ROUND_ROBIN_EN generate
    arbiter_fsm: process(state, a_req, b_req, a_lock_i, a_req_i, b_req_i, x_rsp_i)
    begin
      -- defaults --
      state_nxt <= state;
      sel       <= '0';
      stb       <= '0';

      -- state machine --
      case state is

        when S_BUSY_A => -- port A access in progress
        -- ------------------------------------------------------------
          sel <= '0';
          if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
            state_nxt <= S_CHECK_A;
          end if;

        when S_BUSY_B => -- port B access in progress
        -- ------------------------------------------------------------
          sel <= '1';
          if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
            state_nxt <= S_CHECK_A;
          end if;

        when others => -- wait for requests
        -- ------------------------------------------------------------
          if (a_req_i.stb = '1') or (a_req = '1') then -- request from port A (prioritized)?
            sel       <= '0';
            stb       <= '1';
            state_nxt <= S_BUSY_A;
          elsif ((b_req_i.stb = '1') or (b_req = '1')) and (a_lock_i = '0') then -- request from port B?
            sel       <= '1';
            stb       <= '1';
            state_nxt <= S_BUSY_B;
          end if;

      end case;
    end process arbiter_fsm;
  end generate;


  -- Round-Robin Arbiter --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_round_robin:
  if ROUND_ROBIN_EN generate
    arbiter_fsm: process(state, a_req, b_req, a_req_i, b_req_i, x_rsp_i)
    begin
      -- defaults --
      state_nxt <= state;
      sel       <= '0';
      stb       <= '0';

      -- state machine --
      case state is

        when S_CHECK_A => -- check if access from port A
        -- ------------------------------------------------------------
          sel <= '0';
          if (a_req_i.stb = '1') or (a_req = '1') then
            stb       <= '1';
            state_nxt <= S_BUSY_A;
          else
            state_nxt <= S_CHECK_B;
          end if;

        when S_BUSY_A => -- port B access in progress
        -- ------------------------------------------------------------
          sel <= '0';
          if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
            state_nxt <= S_CHECK_B;
          end if;

        when S_CHECK_B => -- check if access from port B
        -- ------------------------------------------------------------
          sel <= '1';
          if (b_req_i.stb = '1') or (b_req = '1') then
            stb       <= '1';
            state_nxt <= S_BUSY_B;
          else
            state_nxt <= S_CHECK_A;
          end if;

        when S_BUSY_B => -- port B access in progress
        -- ------------------------------------------------------------
          sel <= '1';
          if (x_rsp_i.err = '1') or (x_rsp_i.ack = '1') then
            state_nxt <= S_CHECK_A;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          state_nxt <= S_CHECK_A;

      end case;
    end process arbiter_fsm;
  end generate;


  -- Request Switch -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  x_req_o.addr  <= a_req_i.addr  when (sel = '0') else b_req_i.addr;
  x_req_o.amo   <= a_req_i.amo   when (sel = '0') else b_req_i.amo;
  x_req_o.amoop <= a_req_i.amoop when (sel = '0') else b_req_i.amoop;
  x_req_o.priv  <= a_req_i.priv  when (sel = '0') else b_req_i.priv;
  x_req_o.src   <= a_req_i.src   when (sel = '0') else b_req_i.src;
  x_req_o.rw    <= a_req_i.rw    when (sel = '0') else b_req_i.rw;
  x_req_o.fence <= a_req_i.fence or  b_req_i.fence; -- propagate any fence request
  x_req_o.sleep <= a_req_i.sleep and b_req_i.sleep; -- set if ALL upstream devices are in sleep mode
  x_req_o.debug <= a_req_i.debug when (sel = '0') else b_req_i.debug;

  x_req_o.data  <= b_req_i.data  when PORT_A_READ_ONLY    else
                   a_req_i.data  when PORT_B_READ_ONLY    else
                   a_req_i.data  when (sel = '0') else b_req_i.data;

  x_req_o.ben   <= b_req_i.ben   when PORT_A_READ_ONLY    else
                   a_req_i.ben   when PORT_B_READ_ONLY    else
                   a_req_i.ben   when (sel = '0') else b_req_i.ben;

  x_req_o.stb   <= stb;


  -- Response Switch ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  a_rsp_o.data <= x_rsp_i.data;
  a_rsp_o.ack  <= x_rsp_i.ack when (sel = '0') else '0';
  a_rsp_o.err  <= x_rsp_i.err when (sel = '0') else '0';

  b_rsp_o.data <= x_rsp_i.data;
  b_rsp_o.ack  <= x_rsp_i.ack when (sel = '1') else '0';
  b_rsp_o.err  <= x_rsp_i.err when (sel = '1') else '0';


end neorv32_bus_switch_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: Bus Register Stage                   --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_bus_reg is
  generic (
    REQ_REG_EN : boolean := false; -- enable request bus register stage
    RSP_REG_EN : boolean := false  -- enable response bus register stage
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    rstn_i       : in  std_ulogic; -- global reset, low-active, async
    -- bus ports --
    host_req_i   : in  bus_req_t; -- host request
    host_rsp_o   : out bus_rsp_t; -- host response
    device_req_o : out bus_req_t; -- device request
    device_rsp_i : in  bus_rsp_t  -- device response
  );
end neorv32_bus_reg;

architecture neorv32_bus_reg_rtl of neorv32_bus_reg is

begin

  -- Request Register -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  request_reg_enabled:
  if REQ_REG_EN generate
    request_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        device_req_o <= req_terminate_c;
      elsif rising_edge(clk_i) then
        if (host_req_i.stb = '1') then -- reduce switching activity on device bus system
          device_req_o <= host_req_i;
        end if;
        device_req_o.stb <= host_req_i.stb;
      end if;
    end process request_reg;
  end generate;

  request_reg_disabled:
  if not REQ_REG_EN generate
    device_req_o <= host_req_i;
  end generate;


  -- Response Register ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  response_reg_enabled:
  if RSP_REG_EN generate
    response_reg: process(rstn_i, clk_i)
    begin
      if (rstn_i = '0') then
        host_rsp_o <= rsp_terminate_c;
      elsif rising_edge(clk_i) then
        host_rsp_o <= device_rsp_i;
      end if;
    end process response_reg;
  end generate;

  response_reg_disabled:
  if not RSP_REG_EN generate
    host_rsp_o <= device_rsp_i;
  end generate;


end neorv32_bus_reg_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: Section Gateway                      --
-- -------------------------------------------------------------------------------- --
-- Bus gateway to distribute accesses to 4 non-overlapping address sub-spaces       --
-- (A to D). Note that the sub-spaces have to be aligned to their individual sizes. --
-- All accesses that do not match any of these sections are redirected to the "X"   --
-- port. The gateway-internal bus monitor ensures that all accesses are completed   --
-- within a bound time window (if port's *_TMO_EN is true). Otherwise, a bus error  --
-- exception is raised.                                                             --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_bus_gateway is
  generic (
    TIMEOUT  : natural; -- internal bus timeout cycles
    -- port A --
    A_ENABLE : boolean; -- port enable
    A_BASE   : std_ulogic_vector(31 downto 0); -- port address space base address
    A_SIZE   : natural; -- port address space size in bytes (power of two), aligned to size
    A_TMO_EN : boolean; -- port access timeout enable
    -- port B --
    B_ENABLE : boolean;
    B_BASE   : std_ulogic_vector(31 downto 0);
    B_SIZE   : natural;
    B_TMO_EN : boolean;
    -- port C --
    C_ENABLE : boolean;
    C_BASE   : std_ulogic_vector(31 downto 0);
    C_SIZE   : natural;
    C_TMO_EN : boolean;
    -- port D --
    D_ENABLE : boolean;
    D_BASE   : std_ulogic_vector(31 downto 0);
    D_SIZE   : natural;
    D_TMO_EN : boolean;
    -- port X (the void) --
    X_ENABLE : boolean;
    X_TMO_EN : boolean
  );
  port (
    -- global control --
    clk_i   : in  std_ulogic; -- global clock, rising edge
    rstn_i  : in  std_ulogic; -- global reset, low-active, async
    -- host port --
    req_i   : in  bus_req_t;  -- host request
    rsp_o   : out bus_rsp_t;  -- host response
    -- section ports --
    a_req_o : out bus_req_t;
    a_rsp_i : in  bus_rsp_t;
    b_req_o : out bus_req_t;
    b_rsp_i : in  bus_rsp_t;
    c_req_o : out bus_req_t;
    c_rsp_i : in  bus_rsp_t;
    d_req_o : out bus_req_t;
    d_rsp_i : in  bus_rsp_t;
    x_req_o : out bus_req_t;
    x_rsp_i : in  bus_rsp_t
  );
end neorv32_bus_gateway;

architecture neorv32_bus_gateway_rtl of neorv32_bus_gateway is

  -- port select --
  signal port_sel : std_ulogic_vector(4 downto 0);

  -- port enable list --
  type port_bool_list_t is array (0 to 4) of boolean;
  constant port_en_list_c : port_bool_list_t := (A_ENABLE, B_ENABLE, C_ENABLE, D_ENABLE, X_ENABLE);

  -- port timeout enable list --
  constant tmo_en_list_c : std_ulogic_vector(4 downto 0) := (
    bool_to_ulogic_f(X_TMO_EN), bool_to_ulogic_f(D_TMO_EN), bool_to_ulogic_f(C_TMO_EN), bool_to_ulogic_f(B_TMO_EN), bool_to_ulogic_f(A_TMO_EN)
  );

  -- gateway ports combined as arrays --
  type port_req_t is array (0 to 4) of bus_req_t;
  type port_rsp_t is array (0 to 4) of bus_rsp_t;
  signal port_req : port_req_t;
  signal port_rsp : port_rsp_t;

  -- summarized response --
  signal int_rsp : bus_rsp_t;

  -- bus monitor --
  type keeper_t is record
    busy : std_ulogic;
    cnt  : std_ulogic_vector(index_size_f(TIMEOUT) downto 0);
    err  : std_ulogic;
    halt : std_ulogic;
  end record;
  signal keeper : keeper_t;

begin

  -- Address Section Decoder ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  port_sel(0) <= '1' when A_ENABLE and (req_i.addr(31 downto index_size_f(A_SIZE)) = A_BASE(31 downto index_size_f(A_SIZE))) else '0';
  port_sel(1) <= '1' when B_ENABLE and (req_i.addr(31 downto index_size_f(B_SIZE)) = B_BASE(31 downto index_size_f(B_SIZE))) else '0';
  port_sel(2) <= '1' when C_ENABLE and (req_i.addr(31 downto index_size_f(C_SIZE)) = C_BASE(31 downto index_size_f(C_SIZE))) else '0';
  port_sel(3) <= '1' when D_ENABLE and (req_i.addr(31 downto index_size_f(D_SIZE)) = D_BASE(31 downto index_size_f(D_SIZE))) else '0';

  -- accesses to the "void" are redirected to the X port --
  port_sel(4) <= '1' when X_ENABLE and (port_sel(3 downto 0) = "0000") else '0';


  -- Gateway Ports --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  a_req_o <= port_req(0); port_rsp(0) <= a_rsp_i;
  b_req_o <= port_req(1); port_rsp(1) <= b_rsp_i;
  c_req_o <= port_req(2); port_rsp(2) <= c_rsp_i;
  d_req_o <= port_req(3); port_rsp(3) <= d_rsp_i;
  x_req_o <= port_req(4); port_rsp(4) <= x_rsp_i;

  -- bus request --
  request: process(req_i, port_sel)
  begin
    for i in 0 to 4 loop
      port_req(i) <= req_terminate_c;
      if port_en_list_c(i) then -- port enabled
        port_req(i) <= req_i;
        port_req(i).stb <= port_sel(i) and req_i.stb;
      end if;
    end loop;
  end process request;

  -- bus response --
  response: process(port_rsp)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start with all-zero
    for i in 0 to 4 loop -- OR all response signals
      if port_en_list_c(i) then -- port enabled
        tmp_v.data := tmp_v.data or port_rsp(i).data;
        tmp_v.ack  := tmp_v.ack  or port_rsp(i).ack;
        tmp_v.err  := tmp_v.err  or port_rsp(i).err;
      end if;
    end loop;
    int_rsp <= tmp_v;
  end process response;

  -- host response --
  rsp_o.data <= int_rsp.data;
  rsp_o.ack  <= int_rsp.ack;
  rsp_o.err  <= keeper.err;


  -- Bus Monitor (aka "the KEEPER") ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_monitor: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      keeper.busy <= '0';
      keeper.cnt  <= (others => '0');
      keeper.err  <= '0';
      keeper.halt <= '0';
    elsif rising_edge(clk_i) then
      keeper.err  <= '0'; -- default
      keeper.halt <= or_reduce_f(port_sel and (not tmo_en_list_c)); -- no timeout if *_TMO_EN = false
      if (keeper.busy = '0') then -- bus idle
        keeper.cnt  <= std_ulogic_vector(to_unsigned(TIMEOUT, keeper.cnt'length));
        keeper.busy <= req_i.stb;
      else -- bus access in progress
        keeper.cnt <= std_ulogic_vector(unsigned(keeper.cnt) - 1);
        if (int_rsp.err = '1') or ((or_reduce_f(keeper.cnt) = '0') and (keeper.halt = '0')) then -- bus error or timeout
          keeper.err  <= '1';
          keeper.busy <= '0';
        elsif (int_rsp.ack = '1') then -- normal access termination
          keeper.busy <= '0';
        end if;
      end if;
    end if;
  end process bus_monitor;


end neorv32_bus_gateway_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: IO Switch                            --
-- -------------------------------------------------------------------------------- --
-- Simple switch for accessing one out of several (IO) devices. The main request    --
-- input bus provides a partial register stage to relax timing. Thus, accesses      --
-- require an additional clock cycle.                                               --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_bus_io_switch is
  generic (
    INREG_EN  : boolean := false; -- enable main_req_i register stage
    OUTREG_EN : boolean := false; -- enable main_rsp_o register stage
    DEV_SIZE  : natural := 256; -- size of each single IO device, has to be a power of two
    -- device port enable and base address; enabled ports do not have to be contiguous --
    DEV_00_EN : boolean := false; DEV_00_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_01_EN : boolean := false; DEV_01_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_02_EN : boolean := false; DEV_02_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_03_EN : boolean := false; DEV_03_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_04_EN : boolean := false; DEV_04_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_05_EN : boolean := false; DEV_05_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_06_EN : boolean := false; DEV_06_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_07_EN : boolean := false; DEV_07_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_08_EN : boolean := false; DEV_08_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_09_EN : boolean := false; DEV_09_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_10_EN : boolean := false; DEV_10_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_11_EN : boolean := false; DEV_11_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_12_EN : boolean := false; DEV_12_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_13_EN : boolean := false; DEV_13_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_14_EN : boolean := false; DEV_14_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_15_EN : boolean := false; DEV_15_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_16_EN : boolean := false; DEV_16_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_17_EN : boolean := false; DEV_17_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_18_EN : boolean := false; DEV_18_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_19_EN : boolean := false; DEV_19_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_20_EN : boolean := false; DEV_20_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_21_EN : boolean := false; DEV_21_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_22_EN : boolean := false; DEV_22_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_23_EN : boolean := false; DEV_23_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_24_EN : boolean := false; DEV_24_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_25_EN : boolean := false; DEV_25_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_26_EN : boolean := false; DEV_26_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_27_EN : boolean := false; DEV_27_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_28_EN : boolean := false; DEV_28_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_29_EN : boolean := false; DEV_29_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_30_EN : boolean := false; DEV_30_BASE : std_ulogic_vector(31 downto 0) := (others => '0');
    DEV_31_EN : boolean := false; DEV_31_BASE : std_ulogic_vector(31 downto 0) := (others => '0')
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    rstn_i       : in  std_ulogic; -- global reset, low-active, async
    -- host port --
    main_req_i   : in  bus_req_t; -- host request
    main_rsp_o   : out bus_rsp_t; -- host response
    -- device ports --
    dev_00_req_o : out bus_req_t; dev_00_rsp_i : in bus_rsp_t;
    dev_01_req_o : out bus_req_t; dev_01_rsp_i : in bus_rsp_t;
    dev_02_req_o : out bus_req_t; dev_02_rsp_i : in bus_rsp_t;
    dev_03_req_o : out bus_req_t; dev_03_rsp_i : in bus_rsp_t;
    dev_04_req_o : out bus_req_t; dev_04_rsp_i : in bus_rsp_t;
    dev_05_req_o : out bus_req_t; dev_05_rsp_i : in bus_rsp_t;
    dev_06_req_o : out bus_req_t; dev_06_rsp_i : in bus_rsp_t;
    dev_07_req_o : out bus_req_t; dev_07_rsp_i : in bus_rsp_t;
    dev_08_req_o : out bus_req_t; dev_08_rsp_i : in bus_rsp_t;
    dev_09_req_o : out bus_req_t; dev_09_rsp_i : in bus_rsp_t;
    dev_10_req_o : out bus_req_t; dev_10_rsp_i : in bus_rsp_t;
    dev_11_req_o : out bus_req_t; dev_11_rsp_i : in bus_rsp_t;
    dev_12_req_o : out bus_req_t; dev_12_rsp_i : in bus_rsp_t;
    dev_13_req_o : out bus_req_t; dev_13_rsp_i : in bus_rsp_t;
    dev_14_req_o : out bus_req_t; dev_14_rsp_i : in bus_rsp_t;
    dev_15_req_o : out bus_req_t; dev_15_rsp_i : in bus_rsp_t;
    dev_16_req_o : out bus_req_t; dev_16_rsp_i : in bus_rsp_t;
    dev_17_req_o : out bus_req_t; dev_17_rsp_i : in bus_rsp_t;
    dev_18_req_o : out bus_req_t; dev_18_rsp_i : in bus_rsp_t;
    dev_19_req_o : out bus_req_t; dev_19_rsp_i : in bus_rsp_t;
    dev_20_req_o : out bus_req_t; dev_20_rsp_i : in bus_rsp_t;
    dev_21_req_o : out bus_req_t; dev_21_rsp_i : in bus_rsp_t;
    dev_22_req_o : out bus_req_t; dev_22_rsp_i : in bus_rsp_t;
    dev_23_req_o : out bus_req_t; dev_23_rsp_i : in bus_rsp_t;
    dev_24_req_o : out bus_req_t; dev_24_rsp_i : in bus_rsp_t;
    dev_25_req_o : out bus_req_t; dev_25_rsp_i : in bus_rsp_t;
    dev_26_req_o : out bus_req_t; dev_26_rsp_i : in bus_rsp_t;
    dev_27_req_o : out bus_req_t; dev_27_rsp_i : in bus_rsp_t;
    dev_28_req_o : out bus_req_t; dev_28_rsp_i : in bus_rsp_t;
    dev_29_req_o : out bus_req_t; dev_29_rsp_i : in bus_rsp_t;
    dev_30_req_o : out bus_req_t; dev_30_rsp_i : in bus_rsp_t;
    dev_31_req_o : out bus_req_t; dev_31_rsp_i : in bus_rsp_t
  );
end neorv32_bus_io_switch;

architecture neorv32_bus_io_switch_rtl of neorv32_bus_io_switch is

  -- bus register --
  component neorv32_bus_reg
  generic (
    REQ_REG_EN : boolean := false;
    RSP_REG_EN : boolean := false
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic;
    rstn_i       : in  std_ulogic;
    -- bus ports --
    host_req_i   : in  bus_req_t;
    host_rsp_o   : out bus_rsp_t;
    device_req_o : out bus_req_t;
    device_rsp_i : in  bus_rsp_t
  );
  end component;

  -- module configuration --
  constant num_devs_c : natural := 32; -- number of device ports

  -- address bit boundaries for access decoding --
  constant addr_lo_c : natural := index_size_f(DEV_SIZE); -- low address boundary bit
  constant addr_hi_c : natural := (index_size_f(DEV_SIZE) + index_size_f(num_devs_c)) - 1; -- high address boundary bit

  -- list of enabled device ports --
  type dev_en_list_t is array (0 to num_devs_c-1) of boolean;
  constant dev_en_list_c : dev_en_list_t := (
    DEV_00_EN, DEV_01_EN, DEV_02_EN, DEV_03_EN, DEV_04_EN, DEV_05_EN, DEV_06_EN, DEV_07_EN,
    DEV_08_EN, DEV_09_EN, DEV_10_EN, DEV_11_EN, DEV_12_EN, DEV_13_EN, DEV_14_EN, DEV_15_EN,
    DEV_16_EN, DEV_17_EN, DEV_18_EN, DEV_19_EN, DEV_20_EN, DEV_21_EN, DEV_22_EN, DEV_23_EN,
    DEV_24_EN, DEV_25_EN, DEV_26_EN, DEV_27_EN, DEV_28_EN, DEV_29_EN, DEV_30_EN, DEV_31_EN
  );

  -- list of device base addresses --
  type dev_base_list_t is array (0 to num_devs_c-1) of std_ulogic_vector(31 downto 0);
  constant dev_base_list_c : dev_base_list_t := (
    DEV_00_BASE, DEV_01_BASE, DEV_02_BASE, DEV_03_BASE, DEV_04_BASE, DEV_05_BASE, DEV_06_BASE, DEV_07_BASE,
    DEV_08_BASE, DEV_09_BASE, DEV_10_BASE, DEV_11_BASE, DEV_12_BASE, DEV_13_BASE, DEV_14_BASE, DEV_15_BASE,
    DEV_16_BASE, DEV_17_BASE, DEV_18_BASE, DEV_19_BASE, DEV_20_BASE, DEV_21_BASE, DEV_22_BASE, DEV_23_BASE,
    DEV_24_BASE, DEV_25_BASE, DEV_26_BASE, DEV_27_BASE, DEV_28_BASE, DEV_29_BASE, DEV_30_BASE, DEV_31_BASE
  );

  -- device ports combined as arrays --
  type dev_req_t is array (0 to num_devs_c-1) of bus_req_t;
  type dev_rsp_t is array (0 to num_devs_c-1) of bus_rsp_t;
  signal dev_req : dev_req_t;
  signal dev_rsp : dev_rsp_t;

  -- register stages --
  signal main_req : bus_req_t;
  signal main_rsp : bus_rsp_t;

begin

  -- Register Stages ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_bus_reg_inst: neorv32_bus_reg
  generic map (
    REQ_REG_EN => INREG_EN,
    RSP_REG_EN => OUTREG_EN
  )
  port map (
    -- global control --
    clk_i        => clk_i,
    rstn_i       => rstn_i,
    -- bus ports --
    host_req_i   => main_req_i,
    host_rsp_o   => main_rsp_o,
    device_req_o => main_req,
    device_rsp_i => main_rsp
  );


  -- Combine Device Ports -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dev_00_req_o <= dev_req(0);  dev_rsp(0)  <= dev_00_rsp_i;
  dev_01_req_o <= dev_req(1);  dev_rsp(1)  <= dev_01_rsp_i;
  dev_02_req_o <= dev_req(2);  dev_rsp(2)  <= dev_02_rsp_i;
  dev_03_req_o <= dev_req(3);  dev_rsp(3)  <= dev_03_rsp_i;
  dev_04_req_o <= dev_req(4);  dev_rsp(4)  <= dev_04_rsp_i;
  dev_05_req_o <= dev_req(5);  dev_rsp(5)  <= dev_05_rsp_i;
  dev_06_req_o <= dev_req(6);  dev_rsp(6)  <= dev_06_rsp_i;
  dev_07_req_o <= dev_req(7);  dev_rsp(7)  <= dev_07_rsp_i;
  dev_08_req_o <= dev_req(8);  dev_rsp(8)  <= dev_08_rsp_i;
  dev_09_req_o <= dev_req(9);  dev_rsp(9)  <= dev_09_rsp_i;
  dev_10_req_o <= dev_req(10); dev_rsp(10) <= dev_10_rsp_i;
  dev_11_req_o <= dev_req(11); dev_rsp(11) <= dev_11_rsp_i;
  dev_12_req_o <= dev_req(12); dev_rsp(12) <= dev_12_rsp_i;
  dev_13_req_o <= dev_req(13); dev_rsp(13) <= dev_13_rsp_i;
  dev_14_req_o <= dev_req(14); dev_rsp(14) <= dev_14_rsp_i;
  dev_15_req_o <= dev_req(15); dev_rsp(15) <= dev_15_rsp_i;
  dev_16_req_o <= dev_req(16); dev_rsp(16) <= dev_16_rsp_i;
  dev_17_req_o <= dev_req(17); dev_rsp(17) <= dev_17_rsp_i;
  dev_18_req_o <= dev_req(18); dev_rsp(18) <= dev_18_rsp_i;
  dev_19_req_o <= dev_req(19); dev_rsp(19) <= dev_19_rsp_i;
  dev_20_req_o <= dev_req(20); dev_rsp(20) <= dev_20_rsp_i;
  dev_21_req_o <= dev_req(21); dev_rsp(21) <= dev_21_rsp_i;
  dev_22_req_o <= dev_req(22); dev_rsp(22) <= dev_22_rsp_i;
  dev_23_req_o <= dev_req(23); dev_rsp(23) <= dev_23_rsp_i;
  dev_24_req_o <= dev_req(24); dev_rsp(24) <= dev_24_rsp_i;
  dev_25_req_o <= dev_req(25); dev_rsp(25) <= dev_25_rsp_i;
  dev_26_req_o <= dev_req(26); dev_rsp(26) <= dev_26_rsp_i;
  dev_27_req_o <= dev_req(27); dev_rsp(27) <= dev_27_rsp_i;
  dev_28_req_o <= dev_req(28); dev_rsp(28) <= dev_28_rsp_i;
  dev_29_req_o <= dev_req(29); dev_rsp(29) <= dev_29_rsp_i;
  dev_30_req_o <= dev_req(30); dev_rsp(30) <= dev_30_rsp_i;
  dev_31_req_o <= dev_req(31); dev_rsp(31) <= dev_31_rsp_i;


  -- Request --------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_request_gen:
  for i in 0 to (num_devs_c-1) generate

    bus_request_port_enabled:
    if dev_en_list_c(i) generate
      bus_request: process(main_req)
      begin
        dev_req(i) <= main_req;
        if (main_req.addr(addr_hi_c downto addr_lo_c) = dev_base_list_c(i)(addr_hi_c downto addr_lo_c)) then
          dev_req(i).stb <= main_req.stb; -- propagate transaction strobe if address match
        else
          dev_req(i).stb <= '0';
        end if;
      end process bus_request;
    end generate;

    bus_request_port_disabled:
    if not dev_en_list_c(i) generate
      dev_req(i) <= req_terminate_c;
    end generate;

  end generate;


  -- Response -------------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_response: process(dev_rsp)
    variable tmp_v : bus_rsp_t;
  begin
    tmp_v := rsp_terminate_c; -- start with all-zero
    for i in 0 to (num_devs_c-1) loop -- OR all enabled response buses
      if dev_en_list_c(i) then
        tmp_v.data := tmp_v.data or dev_rsp(i).data;
        tmp_v.ack  := tmp_v.ack  or dev_rsp(i).ack;
        tmp_v.err  := tmp_v.err  or dev_rsp(i).err;
      end if;
    end loop;
    main_rsp <= tmp_v;
  end process;


end neorv32_bus_io_switch_rtl;


-- ================================================================================ --
-- NEORV32 SoC - Processor Bus Infrastructure: Atomic Memory Operations Controller  --
-- -------------------------------------------------------------------------------- --
-- Read-modify-write controller for the RISC-V A/Zaamp ISA extension.               --
-- [WARNING] Load-reservate and store-conditional operations (Zalrsc ISA extension) --
-- are NOT supported!                                                               --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_bus_amo_ctrl is
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- global clock, rising edge
    rstn_i     : in  std_ulogic; -- global reset, low-active, async
    -- core port --
    core_req_i : in  bus_req_t;
    core_rsp_o : out bus_rsp_t;
    -- system port --
    sys_req_o  : out bus_req_t;
    sys_rsp_i  : in  bus_rsp_t
  );
end neorv32_bus_amo_ctrl;

architecture neorv32_bus_amo_ctrl_rtl of neorv32_bus_amo_ctrl is

  -- arbiter --
  type state_t is (S_IDLE, S_READ_WAIT, S_EXECUTE, S_WRITE, S_WRITE_WAIT);
  type arbiter_t is record
    state : state_t;
    cmd   : std_ulogic_vector(3 downto 0);
    rdata : std_ulogic_vector(31 downto 0);
    wdata : std_ulogic_vector(31 downto 0);
    ack   : std_ulogic;
  end record;
  signal arbiter, arbiter_nxt : arbiter_t;

  -- internal data ALU --
  signal alu_res : std_ulogic_vector(31 downto 0);

  -- comparator --
  signal cmp_opa  : std_ulogic_vector(32 downto 0);
  signal cmp_opb  : std_ulogic_vector(32 downto 0);
  signal cmp_less : std_ulogic;
  signal cmp_res  : std_ulogic_vector(31 downto 0);

begin

  -- Arbiter Sync ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arbiter.state <= S_IDLE;
      arbiter.cmd   <= (others => '0');
      arbiter.rdata <= (others => '0');
      arbiter.wdata <= (others => '0');
    elsif rising_edge(clk_i) then
      arbiter <= arbiter_nxt;
    end if;
  end process arbiter_sync;


  -- Arbiter Comb ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_comb: process(arbiter, core_req_i, sys_rsp_i)
  begin
    arbiter_nxt <= arbiter; -- defaults
    case arbiter.state is

      when S_IDLE => -- wait for AMO request; pass-through current request
      -- ------------------------------------------------------------
        if (core_req_i.stb = '1') and (core_req_i.amo = '1') then
          arbiter_nxt.cmd   <= core_req_i.amoop;
          arbiter_nxt.wdata <= core_req_i.data;
          arbiter_nxt.state <= S_READ_WAIT;
        end if;

      when S_READ_WAIT => -- wait for device read-access to complete
      -- ------------------------------------------------------------
        arbiter_nxt.rdata <= sys_rsp_i.data;
        if (sys_rsp_i.ack = '1') or (sys_rsp_i.err = '1') then
          arbiter_nxt.state <= S_EXECUTE;
        end if;

      when S_EXECUTE => -- execute atomic data operation
      -- ------------------------------------------------------------
        arbiter_nxt.state <= S_WRITE;

      when S_WRITE => -- wait operation result to device
      -- ------------------------------------------------------------
        arbiter_nxt.state <= S_WRITE_WAIT;

      when S_WRITE_WAIT => -- wait for device write-access to complete
      -- ------------------------------------------------------------
        if (sys_rsp_i.ack = '1') or (sys_rsp_i.err = '1') then
          arbiter_nxt.state <= S_IDLE;
        end if;

      when others => -- undefined
      -- ------------------------------------------------------------
        arbiter_nxt.state <= S_IDLE;

    end case;
  end process arbiter_comb;

  -- request switch --
  sys_req_o.addr  <= core_req_i.addr;
  sys_req_o.data  <= alu_res when (arbiter.state = S_WRITE) or (arbiter.state = S_WRITE_WAIT) else core_req_i.data;
  sys_req_o.ben   <= core_req_i.ben;
  sys_req_o.stb   <= '1' when (arbiter.state = S_WRITE) else core_req_i.stb;
  sys_req_o.rw    <= '1' when (arbiter.state = S_WRITE) or (arbiter.state = S_WRITE_WAIT) else core_req_i.rw;
  sys_req_o.src   <= core_req_i.src;
  sys_req_o.priv  <= core_req_i.priv;
  sys_req_o.amo   <= core_req_i.amo; -- set during the entire read-modify-write operation
  sys_req_o.amoop <= (others => '0'); -- the specific AMO type should not matter after this point
  sys_req_o.fence <= core_req_i.fence;
  sys_req_o.sleep <= core_req_i.sleep;
  sys_req_o.debug <= core_req_i.debug;

  -- response switch --
  core_rsp_o.data <= sys_rsp_i.data when (arbiter.state = S_IDLE) else arbiter.rdata;
  core_rsp_o.err  <= sys_rsp_i.err  when (arbiter.state = S_IDLE) or (arbiter.state = S_WRITE_WAIT) else '0';
  core_rsp_o.ack  <= sys_rsp_i.ack  when (arbiter.state = S_IDLE) or (arbiter.state = S_WRITE_WAIT) else '0';


  -- Arbiter Sync ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  amo_alu: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      alu_res <= (others => '0');
    elsif rising_edge(clk_i) then
      case arbiter.cmd(2 downto 0) is
        when "000"  => alu_res <= arbiter.wdata; -- AMOSWAP
        when "001"  => alu_res <= std_ulogic_vector(unsigned(arbiter.rdata) + unsigned(arbiter.wdata)); -- AMOADD
        when "010"  => alu_res <= arbiter.rdata xor arbiter.wdata; -- AMOXOR
        when "011"  => alu_res <= arbiter.rdata and arbiter.wdata; -- AMOAND
        when "100"  => alu_res <= arbiter.rdata or arbiter.wdata; -- AMOOR
        when others => alu_res <= cmp_res; -- AMOMIN[U] / AMOMAX[U]
      end case;
    end if;
  end process amo_alu;

  -- comparator logic (min/max and signed/unsigned) --
  cmp_opa  <= (arbiter.rdata(arbiter.rdata'left) and arbiter.cmd(3)) & arbiter.rdata; -- sign-extend if signed operation
  cmp_opb  <= (arbiter.wdata(arbiter.wdata'left) and arbiter.cmd(3)) & arbiter.wdata; -- sign-extend if signed operation
  cmp_less <= '1' when (signed(cmp_opa) < signed(cmp_opb)) else '0';
  cmp_res  <= cmp_opa(31 downto 0) when ((cmp_less xor arbiter.cmd(0)) = '1') else cmp_opb(31 downto 0);


end neorv32_bus_amo_ctrl_rtl;
