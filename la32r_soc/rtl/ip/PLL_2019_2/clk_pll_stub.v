// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2025.2 (win64) Build 6299465 Fri Nov 14 19:35:11 GMT 2025
// Date        : Thu Oct 29 16:04:03 2026
// Host        : xiaohe running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               d:/111.wenjian/7.bisaitijiao/04_quanbudaima/FPGA_shimintongbucaiji/FPGA_TSN_Acquisition/la32r_soc/rtl/ip/PLL_2019_2/clk_pll_stub.v
// Design      : clk_pll
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a200tfbg484-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* CORE_GENERATION_INFO = "clk_pll,clk_wiz_v6_0_17_0_0,{component_name=clk_pll,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,enable_axi=0,feedback_source=FDBK_AUTO,PRIMITIVE=PLL,num_out_clk=4,clkin1_period=20.000,clkin2_period=10.0,use_power_down=false,use_reset=true,use_locked=true,use_inclk_stopped=false,feedback_type=SINGLE,CLOCK_MGR_TYPE=NA,manual_override=false}" *) 
module clk_pll(cpu_clk, sys_clk, eth_clk, eth_clk_90, resetn, 
  locked, clk_in1)
/* synthesis syn_black_box black_box_pad_pin="resetn,locked,clk_in1" */
/* synthesis syn_force_seq_prim="cpu_clk" */
/* synthesis syn_force_seq_prim="sys_clk" */
/* synthesis syn_force_seq_prim="eth_clk" */
/* synthesis syn_force_seq_prim="eth_clk_90" */;
  output cpu_clk /* synthesis syn_isclock = 1 */;
  output sys_clk /* synthesis syn_isclock = 1 */;
  output eth_clk /* synthesis syn_isclock = 1 */;
  output eth_clk_90 /* synthesis syn_isclock = 1 */;
  input resetn;
  output locked;
  input clk_in1;
endmodule
