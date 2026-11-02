`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_ptp_master_tx
// -----------------------------------------------------------------------------
// Purpose:
//   First-stage PTP Master TX path for the Observer FPGA.
//
// Implemented now:
//   1) Periodically emits a two-step PTPv2 Sync event frame on UDP/319.
//   2) Requests a hardware TX timestamp from observer_axis_mac_tx for Sync.
//   3) Emits a Follow_Up general frame on UDP/320 carrying the captured Sync
//      SFD timestamp as preciseOriginTimestamp.
//
// Not implemented in this step:
//   - Delay_Req RX parsing and Delay_Resp TX.  The output path is intentionally
//     structured so that a future Delay_Resp source can reuse the same mux/MAC.
//   - Announce, Management, Pdelay or full IEEE 1588 state machines.
//
// Frame model:
//   This block outputs a complete Ethernet frame body:
//     Ethernet header + IPv4 header + UDP header + PTP payload
//   The downstream MAC block adds preamble/SFD/FCS/IFG.
//
// Timestamp model:
//   Sync is sent as two-step.  Its originTimestamp field is zero.  The Follow_Up
//   frame carries the exact seconds/nanoseconds timestamp captured by MAC TX at
//   the Sync SFD.  The flat ns timestamp is also exposed for telemetry/debug.
//////////////////////////////////////////////////////////////////////////////////
module observer_ptp_master_tx #(
    parameter [47:0] LOCAL_MAC             = 48'h04_CB_01_DA_AA_DA,
    parameter [47:0] DUT_MAC               = 48'h00_0A_35_01_FE_C0,
    parameter [31:0] LOCAL_IP              = 32'hC0A8_0003, // 192.168.0.3
    parameter [31:0] DUT_IP                = 32'hC0A8_000E, // 192.168.0.14
    parameter [63:0] LOCAL_CLOCK_ID        = 64'h04CB_01FF_FEDA_AADA,
    parameter [15:0] LOCAL_PORT_NUM        = 16'd1,
    parameter [7:0]  PTP_DOMAIN            = 8'd0,
    parameter [7:0]  LOG_SYNC_INTERVAL     = 8'h00,
    parameter [31:0] SYNC_INTERVAL_CYCLES  = 32'd125000000,
    parameter [31:0] TXSTAMP_TIMEOUT_CYCLES= 32'd1000000,
    parameter [7:0]  FOLLOW_UP_GAP_CYCLES  = 8'd32
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    // Hardware TX timestamp from MAC TX.  Only Sync asks for this timestamp.
    input  wire        tx_timestamp_valid,
    input  wire [63:0] tx_timestamp_ns,
    input  wire [47:0] tx_timestamp_seconds,
    input  wire [31:0] tx_timestamp_nanoseconds,

    // Frame-body stream to TX mux / MAC.
    output wire [7:0]  m_axis_tdata,
    output wire        m_axis_tvalid,
    output wire        m_axis_tlast,
    input  wire        m_axis_tready,
    output wire [3:0]  m_axis_tuser,

    output wire        busy,
    output reg  [15:0] current_sequence_id,
    output reg  [31:0] stat_sync_count,
    output reg  [31:0] stat_follow_up_count,
    output reg  [31:0] stat_txstamp_timeout_count,
    output reg  [63:0] stat_last_sync_t1_ns,
    output reg  [47:0] stat_last_sync_t1_seconds,
    output reg  [31:0] stat_last_sync_t1_nanoseconds
);

    localparam [2:0] ST_WAIT         = 3'd0;
    localparam [2:0] ST_SEND         = 3'd1;
    localparam [2:0] ST_WAIT_STAMP   = 3'd2;
    localparam [2:0] ST_FUP_GAP      = 3'd3;

    localparam [3:0] MSG_SYNC      = 4'h0;
    localparam [3:0] MSG_FOLLOW_UP = 4'h8;

    localparam [7:0] FRAME_BYTES = 8'd86; // 14 Ethernet + 20 IPv4 + 8 UDP + 44 PTP

    localparam [15:0] UDP_PORT_EVENT   = 16'd319;
    localparam [15:0] UDP_PORT_GENERAL = 16'd320;
    localparam [15:0] IPV4_TOTAL_LEN   = 16'd72;
    localparam [15:0] UDP_LEN          = 16'd52;
    localparam [15:0] PTP_MSG_LEN      = 16'd44;

    localparam [1:0] FRAME_TYPE_PTP_EVENT   = 2'b01;
    localparam [1:0] FRAME_TYPE_PTP_GENERAL = 2'b10;

    reg [2:0]  state_r;
    reg [31:0] interval_cnt_r;
    reg [31:0] timeout_cnt_r;
    reg [7:0]  byte_idx_r;
    reg [7:0]  gap_cnt_r;
    reg [3:0]  active_msg_type_r;
    reg [15:0] active_sequence_id_r;
    reg [15:0] active_ip_ident_r;

    reg        sync_stamp_seen_r;
    reg [63:0] sync_t1_ns_r;
    reg [47:0] sync_t1_seconds_r;
    reg [31:0] sync_t1_nanoseconds_r;

    wire fire_w;
    wire send_sync_w;
    wire send_follow_up_w;
    wire last_byte_w;
    wire active_is_sync_w;
    wire active_is_follow_up_w;

    assign fire_w              = m_axis_tvalid && m_axis_tready;
    assign last_byte_w         = (byte_idx_r == (FRAME_BYTES - 8'd1));
    assign active_is_sync_w    = (active_msg_type_r == MSG_SYNC);
    assign active_is_follow_up_w = (active_msg_type_r == MSG_FOLLOW_UP);
    assign send_sync_w         = active_is_sync_w;
    assign send_follow_up_w    = active_is_follow_up_w;

    assign m_axis_tvalid = (state_r == ST_SEND);
    assign m_axis_tlast  = (state_r == ST_SEND) && last_byte_w;
    assign m_axis_tdata  = f_frame_byte(byte_idx_r,
                                        active_msg_type_r,
                                        active_sequence_id_r,
                                        active_ip_ident_r,
                                        sync_t1_seconds_r,
                                        sync_t1_nanoseconds_r);
    assign m_axis_tuser  = ((state_r == ST_SEND) && (byte_idx_r == 8'd0)) ?
                           (active_is_sync_w ? {FRAME_TYPE_PTP_EVENT, 1'b1, 1'b1} :
                                               {FRAME_TYPE_PTP_GENERAL,1'b0, 1'b1}) :
                           4'd0;
    assign busy = (state_r != ST_WAIT);

    function [15:0] f_ipv4_checksum;
        input [15:0] ident;
        reg [31:0] sum;

        begin
            sum = 32'd0;
            sum = sum + 16'h4500;
            sum = sum + IPV4_TOTAL_LEN;
            sum = sum + ident;
            sum = sum + 16'h4000; // don't fragment
            sum = sum + 16'h4011; // TTL=64, protocol=UDP
            sum = sum + LOCAL_IP[31:16];
            sum = sum + LOCAL_IP[15:0];
            sum = sum + DUT_IP[31:16];
            sum = sum + DUT_IP[15:0];
            sum = {16'd0, sum[15:0]} + {16'd0, sum[31:16]};
            sum = {16'd0, sum[15:0]} + {16'd0, sum[31:16]};
            f_ipv4_checksum = ~sum[15:0];
        end
    endfunction

    function [7:0] f_ptp_timestamp_byte;
        input [3:0]  msg_type;
        input [3:0]  pos;
        input [47:0] sec;
        input [31:0] nsec;
        begin
            if (msg_type == MSG_SYNC) begin
                f_ptp_timestamp_byte = 8'h00;
            end else begin
                case (pos)
                    4'd0: f_ptp_timestamp_byte = sec[47:40];
                    4'd1: f_ptp_timestamp_byte = sec[39:32];
                    4'd2: f_ptp_timestamp_byte = sec[31:24];
                    4'd3: f_ptp_timestamp_byte = sec[23:16];
                    4'd4: f_ptp_timestamp_byte = sec[15:8];
                    4'd5: f_ptp_timestamp_byte = sec[7:0];
                    4'd6: f_ptp_timestamp_byte = nsec[31:24];
                    4'd7: f_ptp_timestamp_byte = nsec[23:16];
                    4'd8: f_ptp_timestamp_byte = nsec[15:8];
                    default: f_ptp_timestamp_byte = nsec[7:0];
                endcase
            end
        end
    endfunction

    function [7:0] f_frame_byte;
        input [7:0]  idx;
        input [3:0]  msg_type;
        input [15:0] seq_id;
        input [15:0] ip_ident;
        input [47:0] t1_sec;
        input [31:0] t1_nsec;
        reg [15:0] ip_csum;
        reg [15:0] src_port;
        reg [15:0] dst_port;
        reg [7:0]  control_field;
        reg [15:0] flag_field;
        begin
            ip_csum      = f_ipv4_checksum(ip_ident);
            src_port     = (msg_type == MSG_SYNC) ? UDP_PORT_EVENT : UDP_PORT_GENERAL;
            dst_port     = src_port;
            control_field= (msg_type == MSG_SYNC) ? 8'h00 : 8'h02;
            flag_field   = (msg_type == MSG_SYNC) ? 16'h0200 : 16'h0000; // twoStepFlag for Sync

            case (idx)
                // Ethernet header, MSB first.
                8'd0:  f_frame_byte = DUT_MAC[47:40];
                8'd1:  f_frame_byte = DUT_MAC[39:32];
                8'd2:  f_frame_byte = DUT_MAC[31:24];
                8'd3:  f_frame_byte = DUT_MAC[23:16];
                8'd4:  f_frame_byte = DUT_MAC[15:8];
                8'd5:  f_frame_byte = DUT_MAC[7:0];
                8'd6:  f_frame_byte = LOCAL_MAC[47:40];
                8'd7:  f_frame_byte = LOCAL_MAC[39:32];
                8'd8:  f_frame_byte = LOCAL_MAC[31:24];
                8'd9:  f_frame_byte = LOCAL_MAC[23:16];
                8'd10: f_frame_byte = LOCAL_MAC[15:8];
                8'd11: f_frame_byte = LOCAL_MAC[7:0];
                8'd12: f_frame_byte = 8'h08;
                8'd13: f_frame_byte = 8'h00;

                // IPv4 header.
                8'd14: f_frame_byte = 8'h45;
                8'd15: f_frame_byte = 8'h00;
                8'd16: f_frame_byte = IPV4_TOTAL_LEN[15:8];
                8'd17: f_frame_byte = IPV4_TOTAL_LEN[7:0];
                8'd18: f_frame_byte = ip_ident[15:8];
                8'd19: f_frame_byte = ip_ident[7:0];
                8'd20: f_frame_byte = 8'h40;
                8'd21: f_frame_byte = 8'h00;
                8'd22: f_frame_byte = 8'd64;
                8'd23: f_frame_byte = 8'd17;
                8'd24: f_frame_byte = ip_csum[15:8];
                8'd25: f_frame_byte = ip_csum[7:0];
                8'd26: f_frame_byte = LOCAL_IP[31:24];
                8'd27: f_frame_byte = LOCAL_IP[23:16];
                8'd28: f_frame_byte = LOCAL_IP[15:8];
                8'd29: f_frame_byte = LOCAL_IP[7:0];
                8'd30: f_frame_byte = DUT_IP[31:24];
                8'd31: f_frame_byte = DUT_IP[23:16];
                8'd32: f_frame_byte = DUT_IP[15:8];
                8'd33: f_frame_byte = DUT_IP[7:0];

                // UDP header.
                8'd34: f_frame_byte = src_port[15:8];
                8'd35: f_frame_byte = src_port[7:0];
                8'd36: f_frame_byte = dst_port[15:8];
                8'd37: f_frame_byte = dst_port[7:0];
                8'd38: f_frame_byte = UDP_LEN[15:8];
                8'd39: f_frame_byte = UDP_LEN[7:0];
                8'd40: f_frame_byte = 8'h00; // UDP checksum disabled for IPv4
                8'd41: f_frame_byte = 8'h00;

                // PTP common header, byte offsets 42..75.
                8'd42: f_frame_byte = {4'h0, msg_type};
                8'd43: f_frame_byte = 8'h02;
                8'd44: f_frame_byte = PTP_MSG_LEN[15:8];
                8'd45: f_frame_byte = PTP_MSG_LEN[7:0];
                8'd46: f_frame_byte = PTP_DOMAIN;
                8'd47: f_frame_byte = 8'h00;
                8'd48: f_frame_byte = flag_field[15:8];
                8'd49: f_frame_byte = flag_field[7:0];
                8'd50,8'd51,8'd52,8'd53,8'd54,8'd55,8'd56,8'd57,
                8'd58,8'd59,8'd60,8'd61: f_frame_byte = 8'h00; // correction + reserved
                8'd62: f_frame_byte = LOCAL_CLOCK_ID[63:56];
                8'd63: f_frame_byte = LOCAL_CLOCK_ID[55:48];
                8'd64: f_frame_byte = LOCAL_CLOCK_ID[47:40];
                8'd65: f_frame_byte = LOCAL_CLOCK_ID[39:32];
                8'd66: f_frame_byte = LOCAL_CLOCK_ID[31:24];
                8'd67: f_frame_byte = LOCAL_CLOCK_ID[23:16];
                8'd68: f_frame_byte = LOCAL_CLOCK_ID[15:8];
                8'd69: f_frame_byte = LOCAL_CLOCK_ID[7:0];
                8'd70: f_frame_byte = LOCAL_PORT_NUM[15:8];
                8'd71: f_frame_byte = LOCAL_PORT_NUM[7:0];
                8'd72: f_frame_byte = seq_id[15:8];
                8'd73: f_frame_byte = seq_id[7:0];
                8'd74: f_frame_byte = control_field;
                8'd75: f_frame_byte = LOG_SYNC_INTERVAL;

                // originTimestamp / preciseOriginTimestamp, byte offsets 76..85.
                8'd76: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd0, t1_sec, t1_nsec);
                8'd77: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd1, t1_sec, t1_nsec);
                8'd78: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd2, t1_sec, t1_nsec);
                8'd79: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd3, t1_sec, t1_nsec);
                8'd80: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd4, t1_sec, t1_nsec);
                8'd81: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd5, t1_sec, t1_nsec);
                8'd82: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd6, t1_sec, t1_nsec);
                8'd83: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd7, t1_sec, t1_nsec);
                8'd84: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd8, t1_sec, t1_nsec);
                default: f_frame_byte = f_ptp_timestamp_byte(msg_type, 4'd9, t1_sec, t1_nsec);
            endcase
        end
    endfunction

    task t_start_sync;
        begin
            active_msg_type_r       <= MSG_SYNC;
            active_sequence_id_r    <= current_sequence_id;
            active_ip_ident_r       <= current_sequence_id;
            byte_idx_r              <= 8'd0;
            sync_stamp_seen_r       <= 1'b0;
            timeout_cnt_r           <= TXSTAMP_TIMEOUT_CYCLES;
            state_r                 <= ST_SEND;
        end
    endtask

    task t_start_follow_up;
        begin
            active_msg_type_r       <= MSG_FOLLOW_UP;
            active_sequence_id_r    <= current_sequence_id;
            active_ip_ident_r       <= current_sequence_id + 16'd1;
            byte_idx_r              <= 8'd0;
            state_r                 <= ST_SEND;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r                       <= ST_WAIT;
            interval_cnt_r                <= 32'd0;
            timeout_cnt_r                 <= 32'd0;
            byte_idx_r                    <= 8'd0;
            gap_cnt_r                     <= 8'd0;
            active_msg_type_r             <= MSG_SYNC;
            active_sequence_id_r          <= 16'd0;
            active_ip_ident_r             <= 16'd0;
            sync_stamp_seen_r             <= 1'b0;
            sync_t1_ns_r                  <= 64'd0;
            sync_t1_seconds_r             <= 48'd0;
            sync_t1_nanoseconds_r         <= 32'd0;
            current_sequence_id           <= 16'd1;
            stat_sync_count               <= 32'd0;
            stat_follow_up_count          <= 32'd0;
            stat_txstamp_timeout_count    <= 32'd0;
            stat_last_sync_t1_ns          <= 64'd0;
            stat_last_sync_t1_seconds     <= 48'd0;
            stat_last_sync_t1_nanoseconds <= 32'd0;
        end else begin
            if (!enable) begin
                state_r          <= ST_WAIT;
                interval_cnt_r   <= 32'd0;
                timeout_cnt_r    <= 32'd0;
                byte_idx_r       <= 8'd0;
                gap_cnt_r        <= 8'd0;
                sync_stamp_seen_r<= 1'b0;
            end else begin
                // Latch Sync timestamp whenever the MAC reports it.  It may
                // arrive while the Sync frame source is still streaming bytes.
                if (tx_timestamp_valid && active_is_sync_w && (state_r == ST_SEND || state_r == ST_WAIT_STAMP)) begin
                    sync_stamp_seen_r             <= 1'b1;
                    sync_t1_ns_r                  <= tx_timestamp_ns;
                    sync_t1_seconds_r             <= tx_timestamp_seconds;
                    sync_t1_nanoseconds_r         <= tx_timestamp_nanoseconds;
                    stat_last_sync_t1_ns          <= tx_timestamp_ns;
                    stat_last_sync_t1_seconds     <= tx_timestamp_seconds;
                    stat_last_sync_t1_nanoseconds <= tx_timestamp_nanoseconds;
                end

                case (state_r)
                    ST_WAIT: begin
                        if (interval_cnt_r == 32'd0) begin
                            t_start_sync();
                        end else begin
                            interval_cnt_r <= interval_cnt_r - 32'd1;
                        end
                    end

                    ST_SEND: begin
                        if (fire_w) begin
                            if (last_byte_w) begin
                                if (send_sync_w) begin
                                    stat_sync_count <= stat_sync_count + 32'd1;
                                    if (sync_stamp_seen_r || tx_timestamp_valid) begin
                                        gap_cnt_r <= FOLLOW_UP_GAP_CYCLES;
                                        state_r   <= ST_FUP_GAP;
                                    end else begin
                                        state_r <= ST_WAIT_STAMP;
                                    end
                                end else if (send_follow_up_w) begin
                                    stat_follow_up_count <= stat_follow_up_count + 32'd1;
                                    current_sequence_id  <= current_sequence_id + 16'd1;
                                    interval_cnt_r       <= SYNC_INTERVAL_CYCLES - 32'd1;
                                    state_r              <= ST_WAIT;
                                end else begin
                                    state_r <= ST_WAIT;
                                end
                            end else begin
                                byte_idx_r <= byte_idx_r + 8'd1;
                            end
                        end
                    end

                    ST_WAIT_STAMP: begin
                        if (tx_timestamp_valid) begin
                            gap_cnt_r <= FOLLOW_UP_GAP_CYCLES;
                            state_r   <= ST_FUP_GAP;
                        end else if (timeout_cnt_r == 32'd0) begin
                            stat_txstamp_timeout_count <= stat_txstamp_timeout_count + 32'd1;
                            current_sequence_id        <= current_sequence_id + 16'd1;
                            interval_cnt_r             <= SYNC_INTERVAL_CYCLES - 32'd1;
                            state_r                    <= ST_WAIT;
                        end else begin
                            timeout_cnt_r <= timeout_cnt_r - 32'd1;
                        end
                    end

                    ST_FUP_GAP: begin
                        if (gap_cnt_r == 8'd0) begin
                            t_start_follow_up();
                        end else begin
                            gap_cnt_r <= gap_cnt_r - 8'd1;
                        end
                    end

                    default: begin
                        state_r <= ST_WAIT;
                    end
                endcase
            end
        end
    end

endmodule
`default_nettype wire
