`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: data_gen
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   内部测试数据源：
//   - mode = 0 : sawtooth（锯齿 / 计数递增）
//   - mode = 1 : triangle（三角波）
//   - mode = 2 : square（方波）
//   - mode = 3 : constant（固定测试字）
//   - 其他模式保留给后续 SIN / COS / CORDIC / LUT 扩展
//
//   说明：
//   - 当前 wave_valid 在 enable=1 时每拍有效
//   - sampler 负责进一步按 sample_period 做抽样
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module data_gen(
        clk_in,
        resetn,
        mode,
        enable,
        wave_out,
        wave_valid
    );

    input               clk_in;
    input               resetn;
    input       [3:0]   mode;
    input               enable;
    output  reg [15:0]  wave_out;
    output  reg         wave_valid;

    reg [15:0] phase_cnt;
    reg [15:0] tri_val;
    reg        tri_dir;

    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        phase_cnt  <= 16'd0;
        tri_val    <= 16'd0;
        tri_dir    <= 1'b0;
        wave_out   <= 16'd0;
        wave_valid <= 1'b0;
    end else if (!enable) begin
        phase_cnt  <= 16'd0;
        tri_val    <= 16'd0;
        tri_dir    <= 1'b0;
        wave_out   <= 16'd0;
        wave_valid <= 1'b0;
    end else begin
        phase_cnt  <= phase_cnt + 16'd1;
        wave_valid <= 1'b1;

        // 三角波内部状态更新
        if (!tri_dir) begin
            tri_val <= tri_val + 16'd256;
            if (tri_val >= 16'hFE00)
                tri_dir <= 1'b1;
        end else begin
            tri_val <= tri_val - 16'd256;
            if (tri_val <= 16'h0200)
                tri_dir <= 1'b0;
        end

        case (mode)
            4'd0: wave_out <= phase_cnt;                          // sawtooth
            4'd1: wave_out <= tri_val;                            // triangle
            4'd2: wave_out <= phase_cnt[15] ? 16'h7000 : 16'h1000;// square
            4'd3: wave_out <= 16'h55AA;                           // constant test pattern
            default: wave_out <= phase_cnt;                       // reserved -> fallback sawtooth
        endcase
    end

endmodule
