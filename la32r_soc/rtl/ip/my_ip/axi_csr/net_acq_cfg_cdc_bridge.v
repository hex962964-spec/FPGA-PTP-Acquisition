`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_cfg_cdc_bridge
// Description:
//   把 sys_clk 域中“已经锁存好的 active 配置”安全搬到目标时钟域。
//
//   设计方式：
//     1) sys 侧先检测外部 req_toggle 的一次新请求；
//     2) 在 sys 域内部把整组 cfg_bus 先锁存到本模块自己的 hold 寄存器；
//     3) 再由本模块内部 launch_toggle 发起一次真正的跨域请求；
//     4) 目标域同步该 launch_toggle，检测到变化后一次性采样 hold 总线；
//     5) 目标域翻转 ack_toggle 回传 ACK；
//     6) sys 侧同步 ACK toggle，向上层报告 sys_ack_toggle。
//
//   这样做的目的：
//     - 配置总线不再直接来自“外部任意寄存器簇”，而是先进入 CDC 模块内部
//       的 dedicated hold bank，结构更清晰，也更利于后续约束；
//     - launch_toggle 与 hold bank 在同一个 sys 域拍边界同时更新，避免目的域
//       过早观察到请求，而配置总线仍来自上层组合/寄存器路径的歧义。
//
//   说明：
//     - 该模块不改变外部接口合同；
//     - 依旧采用“稳定总线 + req/ack toggle”的多位 CDC 模式；
//     - 需要配合顶层工程中的 CDC 约束模板一起使用。
//////////////////////////////////////////////////////////////////////////////////

module net_acq_cfg_cdc_bridge #(
    parameter integer P_CFG_W = 235,
    parameter [P_CFG_W-1:0] P_INIT_CFG = {P_CFG_W{1'b0}}
)(
    // ------------------------------------------------------------------------
    // sys domain
    // ------------------------------------------------------------------------
    input  wire               sys_clk,
    input  wire               sys_resetn,
    input  wire [P_CFG_W-1:0] sys_cfg_bus,
    input  wire               sys_req_toggle,
    output wire               sys_ack_toggle,

    // ------------------------------------------------------------------------
    // target domain (historically eth domain, but kept generic)
    // ------------------------------------------------------------------------
    input  wire               eth_clk,
    input  wire               eth_resetn,
    output reg  [P_CFG_W-1:0] eth_cfg_bus,
    output reg                eth_cfg_update_pulse
);

    // ------------------------------------------------------------------------
    // sys domain: dedicated hold bank + locally-launched request toggle
    // ------------------------------------------------------------------------
    reg [P_CFG_W-1:0] sys_cfg_hold_r;
    reg               sys_req_toggle_seen_r;
    reg               sys_req_launch_toggle_r;

    // ------------------------------------------------------------------------
    // sys -> eth : req toggle synchronizer
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg req_toggle_sync1_eth_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg req_toggle_sync2_eth_r;
    reg                                                  req_toggle_seen_eth_r;

    // ------------------------------------------------------------------------
    // eth -> sys : ack toggle synchronizer
    // ------------------------------------------------------------------------
    reg                                                  ack_toggle_eth_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg ack_toggle_sync1_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg ack_toggle_sync2_sys_r;

    assign sys_ack_toggle = ack_toggle_sync2_sys_r;

    // ------------------------------------------------------------------------
    // sys domain:
    //   1) 检测上层 req_toggle 是否变化；
    //   2) 先锁存配置总线，再翻转本地 launch_toggle；
    //   3) 同步目标域 ACK。
    // ------------------------------------------------------------------------
    always @(posedge sys_clk or negedge sys_resetn) begin
        if (!sys_resetn) begin
            sys_cfg_hold_r         <= P_INIT_CFG;
            sys_req_toggle_seen_r  <= 1'b0;
            sys_req_launch_toggle_r<= 1'b0;
            ack_toggle_sync1_sys_r <= 1'b0;
            ack_toggle_sync2_sys_r <= 1'b0;
        end else begin
            ack_toggle_sync1_sys_r <= ack_toggle_eth_r;
            ack_toggle_sync2_sys_r <= ack_toggle_sync1_sys_r;

            if (sys_req_toggle != sys_req_toggle_seen_r) begin
                sys_req_toggle_seen_r   <= sys_req_toggle;
                sys_cfg_hold_r          <= sys_cfg_bus;
                sys_req_launch_toggle_r <= ~sys_req_launch_toggle_r;
            end
        end
    end

    // ------------------------------------------------------------------------
    // eth domain : 同步 launch toggle，检测边沿后一次性采样 hold bank
    // ------------------------------------------------------------------------
    always @(posedge eth_clk or negedge eth_resetn) begin
        if (!eth_resetn) begin
            req_toggle_sync1_eth_r <= 1'b0;
            req_toggle_sync2_eth_r <= 1'b0;
            req_toggle_seen_eth_r  <= 1'b0;
            ack_toggle_eth_r       <= 1'b0;
            eth_cfg_bus            <= P_INIT_CFG;
            eth_cfg_update_pulse   <= 1'b0;
        end else begin
            req_toggle_sync1_eth_r <= sys_req_launch_toggle_r;
            req_toggle_sync2_eth_r <= req_toggle_sync1_eth_r;
            eth_cfg_update_pulse   <= 1'b0;

            if (req_toggle_sync2_eth_r != req_toggle_seen_eth_r) begin
                req_toggle_seen_eth_r <= req_toggle_sync2_eth_r;
                eth_cfg_bus           <= sys_cfg_hold_r;
                eth_cfg_update_pulse  <= 1'b1;
                ack_toggle_eth_r      <= ~ack_toggle_eth_r;
            end
        end
    end

endmodule
