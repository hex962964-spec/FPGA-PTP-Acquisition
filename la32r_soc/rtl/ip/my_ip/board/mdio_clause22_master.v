`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: mdio_clause22_master
// Description:
//   Minimal Clause-22 MDIO master.
//
//   - Generates MDC from `clk` using an integer half-period divider.
//   - Supports one transaction at a time.
//   - Supports Clause-22 read and write operations.
//   - Drives MDIO actively for all write bits.
//   - Releases MDIO during read turnaround and read data phase.
//
// Timing model:
//   - MDIO output changes while MDC is low.
//   - Input data is sampled on MDC rising edges.
//
// Notes:
//   - Transaction format always uses 32-bit preamble.
//   - `done` and `rd_valid` pulse for one `clk` cycle when the transaction
//     completes and MDC has returned to idle-low.
//////////////////////////////////////////////////////////////////////////////////
module mdio_clause22_master #(
    parameter integer MDC_HALF_PERIOD = 25
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire        start,
    input  wire        op_wr,
    input  wire [4:0]  phy_addr,
    input  wire [4:0]  reg_addr,
    input  wire [15:0] wr_data,

    output reg         busy,
    output reg         done,
    output reg  [15:0] rd_data,
    output reg         rd_valid,

    output wire        mdc,
    output reg         mdio_o,
    output reg         mdio_oe,
    input  wire        mdio_i
);

    localparam [7:0] C_LAST_SLOT = 8'd63;

    reg        mdc_r;
    reg [31:0] div_cnt_r;
    reg [7:0]  slot_index_r;
    reg        finish_pending_r;

    reg        op_wr_r;
    reg [4:0]  phy_addr_r;
    reg [4:0]  reg_addr_r;
    reg [15:0] wr_data_r;

    assign mdc = busy ? mdc_r : 1'b0;

    function [0:0] f_drive_enable;
        input       op_wr_f;
        input [7:0] slot_f;
        begin
            if (op_wr_f) begin
                f_drive_enable = 1'b1;
            end else if (slot_f <= 8'd45) begin
                f_drive_enable = 1'b1;
            end else begin
                f_drive_enable = 1'b0;
            end
        end
    endfunction

    function [0:0] f_frame_bit;
        input        op_wr_f;
        input [4:0]  phy_addr_f;
        input [4:0]  reg_addr_f;
        input [15:0] wr_data_f;
        input [7:0]  slot_f;
        reg [63:0]   frame_v;
        begin
            if (op_wr_f) begin
                frame_v = {32'hFFFF_FFFF, 2'b01, 2'b01, phy_addr_f, reg_addr_f, 2'b10, wr_data_f};
            end else begin
                frame_v = {32'hFFFF_FFFF, 2'b01, 2'b10, phy_addr_f, reg_addr_f, 18'd0};
            end
            f_frame_bit = frame_v[63 - slot_f];
        end
    endfunction

    wire div_tick_w = (div_cnt_r == (MDC_HALF_PERIOD - 1));

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            busy             <= 1'b0;
            done             <= 1'b0;
            rd_data          <= 16'd0;
            rd_valid         <= 1'b0;
            mdc_r            <= 1'b0;
            div_cnt_r        <= 32'd0;
            slot_index_r     <= 8'd0;
            finish_pending_r <= 1'b0;
            op_wr_r          <= 1'b0;
            phy_addr_r       <= 5'd0;
            reg_addr_r       <= 5'd0;
            wr_data_r        <= 16'd0;
            mdio_o           <= 1'b1;
            mdio_oe          <= 1'b0;
        end else begin
            done     <= 1'b0;
            rd_valid <= 1'b0;

            if (!busy) begin
                mdc_r     <= 1'b0;
                div_cnt_r <= 32'd0;
                mdio_o    <= 1'b1;
                mdio_oe   <= 1'b0;

                if (start) begin
                    busy             <= 1'b1;
                    finish_pending_r <= 1'b0;
                    slot_index_r     <= 8'd0;
                    rd_data          <= 16'd0;
                    op_wr_r          <= op_wr;
                    phy_addr_r       <= phy_addr;
                    reg_addr_r       <= reg_addr;
                    wr_data_r        <= wr_data;
                    mdio_oe          <= f_drive_enable(op_wr, 8'd0);
                    mdio_o           <= f_frame_bit(op_wr, phy_addr, reg_addr, wr_data, 8'd0);
                end
            end else begin
                if (div_tick_w) begin
                    div_cnt_r <= 32'd0;

                    if (!mdc_r) begin
                        // Rising edge: sample current slot.
                        mdc_r <= 1'b1;

                        if (!op_wr_r) begin
                            if (slot_index_r >= 8'd48 && slot_index_r <= 8'd63) begin
                                rd_data <= {rd_data[14:0], mdio_i};
                            end
                        end

                        if (slot_index_r == C_LAST_SLOT) begin
                            finish_pending_r <= 1'b1;
                        end else begin
                            slot_index_r <= slot_index_r + 8'd1;
                        end
                    end else begin
                        // Falling edge: prepare next slot while MDC is low.
                        mdc_r <= 1'b0;

                        if (finish_pending_r) begin
                            busy             <= 1'b0;
                            finish_pending_r <= 1'b0;
                            done             <= 1'b1;
                            if (!op_wr_r)
                                rd_valid <= 1'b1;
                            mdio_o  <= 1'b1;
                            mdio_oe <= 1'b0;
                        end else begin
                            mdio_oe <= f_drive_enable(op_wr_r, slot_index_r);
                            mdio_o  <= f_frame_bit(op_wr_r, phy_addr_r, reg_addr_r, wr_data_r, slot_index_r);
                        end
                    end
                end else begin
                    div_cnt_r <= div_cnt_r + 32'd1;
                end
            end
        end
    end

endmodule
