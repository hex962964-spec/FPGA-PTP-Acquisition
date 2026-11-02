`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_pulse_cdc
// Description:
//   单比特脉冲跨时钟域同步模块。
//
//   用法：
//     - src_pulse 在源时钟域拉高 1 拍即可。
//     - 模块内部先把 pulse 转成 toggle，再同步到目标时钟域。
//     - dst_pulse 在目标时钟域输出 1 拍脉冲。
//
//   适用场景：
//     - soft_reset_pulse / clear_status_pulse 等低频控制脉冲跨域。
//
//   CDC / timing 收口说明：
//     - 目标域同步链显式拆成 sync1/sync2 两级寄存器；
//     - 为同步链添加 ASYNC_REG / SHREG_EXTRACT 属性，避免工具把它们
//       当普通时序逻辑优化或抽成 SRL；
//     - 功能语义保持不变，不改变现有 pulse->toggle->pulse 合同。
//////////////////////////////////////////////////////////////////////////////////

module net_acq_pulse_cdc(
    input  wire src_clk,
    input  wire src_resetn,
    input  wire src_pulse,

    input  wire dst_clk,
    input  wire dst_resetn,
    output reg  dst_pulse
);

    // 源域：把单拍 pulse 编码成“翻转一次”的 toggle。
    reg src_toggle_r;

    // 目标域：标准 2FF 同步链。
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dst_toggle_sync1_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dst_toggle_sync2_r;
    reg dst_toggle_seen_r;

    // ------------------------------------------------------------------------
    // 源域：pulse -> toggle
    // ------------------------------------------------------------------------
    always @(posedge src_clk or negedge src_resetn) begin
        if (!src_resetn) begin
            src_toggle_r <= 1'b0;
        end else if (src_pulse) begin
            src_toggle_r <= ~src_toggle_r;
        end
    end

    // ------------------------------------------------------------------------
    // 目标域：同步 toggle 并检测边沿
    // ------------------------------------------------------------------------
    always @(posedge dst_clk or negedge dst_resetn) begin
        if (!dst_resetn) begin
            dst_toggle_sync1_r <= 1'b0;
            dst_toggle_sync2_r <= 1'b0;
            dst_toggle_seen_r  <= 1'b0;
            dst_pulse          <= 1'b0;
        end else begin
            dst_toggle_sync1_r <= src_toggle_r;
            dst_toggle_sync2_r <= dst_toggle_sync1_r;
            dst_pulse          <= 1'b0;

            if (dst_toggle_sync2_r != dst_toggle_seen_r) begin
                dst_toggle_seen_r <= dst_toggle_sync2_r;
                dst_pulse         <= 1'b1;
            end
        end
    end

endmodule
