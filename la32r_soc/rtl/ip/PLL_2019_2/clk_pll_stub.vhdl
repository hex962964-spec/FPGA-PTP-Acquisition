-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
-- Date        : Thu Oct 29 16:04:03 2026
-- Host        : xiaohe running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               d:/111.wenjian/7.bisaitijiao/04_quanbudaima/FPGA_shimintongbucaiji/FPGA_TSN_Acquisition/la32r_soc/rtl/ip/PLL_2019_2/clk_pll_stub.vhdl
-- Design      : clk_pll
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a200tfbg484-2
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity clk_pll is
  Port ( 
    cpu_clk : out STD_LOGIC;
    sys_clk : out STD_LOGIC;
    eth_clk : out STD_LOGIC;
    eth_clk_90 : out STD_LOGIC;
    resetn : in STD_LOGIC;
    locked : out STD_LOGIC;
    clk_in1 : in STD_LOGIC
  );

  attribute CORE_GENERATION_INFO : string;
  attribute CORE_GENERATION_INFO of clk_pll : entity is "clk_pll,clk_wiz_v6_0_17_0_0,{component_name=clk_pll,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,enable_axi=0,feedback_source=FDBK_AUTO,PRIMITIVE=PLL,num_out_clk=4,clkin1_period=20.000,clkin2_period=10.0,use_power_down=false,use_reset=true,use_locked=true,use_inclk_stopped=false,feedback_type=SINGLE,CLOCK_MGR_TYPE=NA,manual_override=false}";
end clk_pll;

architecture stub of clk_pll is
  attribute syn_black_box : boolean;
  attribute black_box_pad_pin : string;
  attribute syn_black_box of stub : architecture is true;
  attribute black_box_pad_pin of stub : architecture is "cpu_clk,sys_clk,eth_clk,eth_clk_90,resetn,locked,clk_in1";
begin
end;
