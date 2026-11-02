`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_rx_mailbox
// -----------------------------------------------------------------------------
// Purpose:
//   Sys-domain single-entry mailbox for the compact PTP RX path.
//
// Placement:
//   ptp_rx_filter (eth_rx_clk) -> ptp_rx_async_fifo -> ptp_rx_mailbox (sys_clk)
//
// CPU polling model:
//   - Software polls mbox_valid in PTP_RX_MBOX_STATUS.
//   - Once valid=1, all mailbox fields remain stable until software writes POP.
//   - If POP and the upstream FIFO already has the next record, the mailbox
//     reloads immediately and mbox_valid stays asserted.
//   - overflow_sticky reflects upstream async-FIFO overflow, i.e. software did
//     not drain mailbox/FIFO quickly enough and at least one record was lost.
//
// Notes:
//   - clear_status must not accidentally consume one FIFO entry in the same
//     cycle, so s_rec_ready is explicitly gated low during clear_status.
//   - A small re-arm helper is kept for overflow_sticky so a stale synchronized
//     high level from the upstream FIFO is not treated as a fresh overflow
//     immediately after clear_status.
//
// Record format must match ptp_rx_filter.
//////////////////////////////////////////////////////////////////////////////////
module ptp_rx_mailbox #(
    parameter integer P_RECORD_W = 256,
    parameter integer P_MSG_TS_W = 80,
    parameter integer P_PORT_ID_W = 80
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     clear_status,

    input  wire                     s_rec_valid,
    output wire                     s_rec_ready,
    input  wire [P_RECORD_W-1:0]    s_rec_data,
    input  wire                     fifo_overflow_sticky,

    output reg                      mbox_valid,
    output reg  [3:0]               mbox_msg_type,
    output reg  [15:0]              mbox_sequence_id,
    output reg  [63:0]              mbox_rx_timestamp_ns,
    output reg  [P_MSG_TS_W-1:0]    mbox_msg_timestamp_80,
    output reg  [P_PORT_ID_W-1:0]   mbox_requesting_port_identity,
    output reg                      mbox_new_pulse,
    input  wire                     mbox_pop,
    output reg                      mbox_overflow_sticky
);

    localparam integer C_MSG_TYPE_MSB = 255;
    localparam integer C_MSG_TYPE_LSB = 252;
    localparam integer C_SEQ_ID_MSB   = 251;
    localparam integer C_SEQ_ID_LSB   = 236;
    localparam integer C_HAS_RX_TS    = 235;
    localparam integer C_HAS_MSG_TS   = 234;
    localparam integer C_HAS_REQ_PID  = 233;
    localparam integer C_RX_TS_MSB    = 231;
    localparam integer C_RX_TS_LSB    = 168;
    localparam integer C_MSG_TS_MSB   = 167;
    localparam integer C_MSG_TS_LSB   = 88;
    localparam integer C_REQ_PID_MSB  = 87;
    localparam integer C_REQ_PID_LSB  = 8;

    reg  overflow_rearm_r;

    wire slot_free_w;
    wire take_record_w;

    assign slot_free_w   = !mbox_valid || mbox_pop;
    assign s_rec_ready   = !clear_status && slot_free_w;
    assign take_record_w = s_rec_valid && s_rec_ready;

    task t_clear_fields;
        begin
            mbox_msg_type                 <= 4'd0;
            mbox_sequence_id              <= 16'd0;
            mbox_rx_timestamp_ns          <= 64'd0;
            mbox_msg_timestamp_80         <= {P_MSG_TS_W{1'b0}};
            mbox_requesting_port_identity <= {P_PORT_ID_W{1'b0}};
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mbox_valid           <= 1'b0;
            t_clear_fields();
            mbox_new_pulse       <= 1'b0;
            mbox_overflow_sticky <= 1'b0;
            overflow_rearm_r     <= 1'b0;
        end else begin
            mbox_new_pulse <= 1'b0;

            if (clear_status) begin
                mbox_valid           <= 1'b0;
                t_clear_fields();
                mbox_overflow_sticky <= 1'b0;
                overflow_rearm_r     <= 1'b0;
            end else begin
                if (!fifo_overflow_sticky) begin
                    overflow_rearm_r <= 1'b1;
                end else if (overflow_rearm_r) begin
                    mbox_overflow_sticky <= 1'b1;
                    overflow_rearm_r     <= 1'b0;
                end

                if (take_record_w) begin
                    mbox_valid       <= 1'b1;
                    mbox_new_pulse   <= 1'b1;
                    mbox_msg_type    <= s_rec_data[C_MSG_TYPE_MSB:C_MSG_TYPE_LSB];
                    mbox_sequence_id <= s_rec_data[C_SEQ_ID_MSB:C_SEQ_ID_LSB];

                    if (s_rec_data[C_HAS_RX_TS])
                        mbox_rx_timestamp_ns <= s_rec_data[C_RX_TS_MSB:C_RX_TS_LSB];
                    else
                        mbox_rx_timestamp_ns <= 64'd0;

                    if (s_rec_data[C_HAS_MSG_TS])
                        mbox_msg_timestamp_80 <= s_rec_data[C_MSG_TS_MSB:C_MSG_TS_LSB];
                    else
                        mbox_msg_timestamp_80 <= {P_MSG_TS_W{1'b0}};

                    if (s_rec_data[C_HAS_REQ_PID])
                        mbox_requesting_port_identity <= s_rec_data[C_REQ_PID_MSB:C_REQ_PID_LSB];
                    else
                        mbox_requesting_port_identity <= {P_PORT_ID_W{1'b0}};
                end else if (mbox_pop && mbox_valid) begin
                    mbox_valid <= 1'b0;
                    t_clear_fields();
                end
            end
        end
    end

endmodule
