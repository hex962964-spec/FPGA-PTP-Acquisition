`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_telemetry_formatter
// -----------------------------------------------------------------------------
// Purpose:
//   Emit multi-line ASCII diagnostic telemetry for the Observer FPGA over UART.
//
// Output contract, once per start_pulse:
//   STAT,<old-compatible 16 fields>
//   CLK,<clock/reset/tod monitor fields>
//   PTP,<Sync/FUP master timing fields>
//   TX,<TX/MUX/Delay_Resp datapath fields>
//   RX,<RX/Delay_Req parser fields>
//   CFG,<static MAC/IP/domain/version fields>
//   DBG,<extra RX timestamp / ToD-sync recovery diagnostics>
//
// All numeric fields are hexadecimal and padded to 8 digits.  STAT remains
// backward compatible with the earlier Stage-1 bring-up line so existing host
// scripts can keep using the first line while the extra lines give enough
// visibility to diagnose WAIT_SYNC / Delay_Req / RX clock / ToD issues.
//////////////////////////////////////////////////////////////////////////////////
module observer_telemetry_formatter(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_pulse,

    // Original STAT fields
    input  wire [31:0] uptime_s,
    input  wire [31:0] rx_frame_count,
    input  wire [31:0] rx_drop_count,
    input  wire [31:0] tx_frame_count,
    input  wire [7:0]  status_flags,
    input  wire [31:0] ptp_sync_count,
    input  wire [31:0] ptp_follow_up_count,
    input  wire [31:0] ptp_timeout_count,
    input  wire [31:0] ptp_sequence_id,
    input  wire [31:0] ptp_last_t1_low,
    input  wire [31:0] tod_rx_sync_count,
    input  wire [31:0] delay_req_count,
    input  wire [31:0] delay_resp_count,
    input  wire [31:0] delay_req_drop_count,
    input  wire [31:0] delay_req_last_seq,
    input  wire [31:0] delay_req_last_t4_low,

    // Clock / monitor fields
    input  wire [31:0] status_ext,
    input  wire [31:0] tx_heartbeat_count,
    input  wire [31:0] rx_heartbeat_count,
    input  wire [31:0] tx_tod_low,
    input  wire [31:0] tx_tod_high,
    input  wire [31:0] rx_tod_low,
    input  wire [31:0] rx_tod_high,
    input  wire [31:0] stat_timeout_count,
    input  wire [31:0] stat_last_wait_cycles,
    input  wire [31:0] rx_status_available,
    input  wire [31:0] tx_tod_valid,
    input  wire [31:0] rx_tod_valid,
    input  wire [31:0] tod_sync_pending,
    input  wire [31:0] tod_sync_timeout_count,
    input  wire [31:0] rx_axis_timestamp_count,

    // PTP master fields
    input  wire [31:0] ptp_last_t1_high,
    input  wire [31:0] ptp_last_t1_seconds_low,
    input  wire [31:0] ptp_last_t1_nanoseconds,
    input  wire [31:0] ptp_master_busy,

    // TX datapath / mux / Delay_Resp fields
    input  wire [31:0] tx_byte_count,
    input  wire [31:0] tx_last_error,
    input  wire [31:0] mux_delay_resp_frame_count,
    input  wire [31:0] mux_ptp_frame_count,
    input  wire [31:0] mux_debug_frame_count,
    input  wire [31:0] delay_resp_last_seq,
    input  wire [31:0] delay_resp_last_t4_low,
    input  wire [31:0] delay_resp_last_t4_high,
    input  wire [31:0] delay_resp_busy,
    input  wire [31:0] test_tx_frame_count,

    // RX datapath / Delay_Req parser fields
    input  wire [31:0] rx_last_error,
    input  wire [31:0] rx_busy,
    input  wire [31:0] delay_req_non_delay_ptp_count,
    input  wire [31:0] delay_req_last_error,
    input  wire [31:0] delay_req_cdc_drop_count,
    input  wire [31:0] delay_req_last_t4_high,

    // Static configuration printout
    input  wire [31:0] local_mac_low,
    input  wire [31:0] local_mac_high,
    input  wire [31:0] dut_mac_low,
    input  wire [31:0] dut_mac_high,
    input  wire [31:0] local_ip,
    input  wire [31:0] dut_ip,
    input  wire [31:0] ptp_domain,
    input  wire [31:0] ptp_sync_interval,
    input  wire [31:0] uart_baud,
    input  wire [31:0] sys_clk_freq,
    input  wire [31:0] build_version,

    output reg  [7:0]  m_data,
    output reg         m_valid,
    input  wire        m_ready,
    output wire        busy
);

    localparam [2:0] ST_IDLE   = 3'd0;
    localparam [2:0] ST_PREFIX = 3'd1;
    localparam [2:0] ST_FIELD  = 3'd2;
    localparam [2:0] ST_COMMA  = 3'd3;
    localparam [2:0] ST_CR     = 3'd4;
    localparam [2:0] ST_LF     = 3'd5;

    localparam [2:0] LINE_STAT = 3'd0;
    localparam [2:0] LINE_CLK  = 3'd1;
    localparam [2:0] LINE_PTP  = 3'd2;
    localparam [2:0] LINE_TX   = 3'd3;
    localparam [2:0] LINE_RX   = 3'd4;
    localparam [2:0] LINE_CFG  = 3'd5;
    localparam [2:0] LINE_DBG  = 3'd6;
    localparam [2:0] LINE_LAST = LINE_DBG;

    localparam [4:0] FIELD_LAST = 5'd15;
    localparam [6:0] TOTAL_FIELDS = 7'd112;

    reg [2:0]  state_r;
    reg [2:0]  line_idx_r;
    reg [3:0]  prefix_idx_r;
    reg [4:0]  field_idx_r;
    reg [3:0]  hex_idx_r;
    reg [6:0]  abs_field_idx_r;
    reg [31:0] field_latched_r [0:111];

    integer i;

    wire fire_w;
    assign fire_w = m_valid && m_ready;
    assign busy   = (state_r != ST_IDLE);

    function [6:0] line_base;
        input [2:0] line;
        begin
            case (line)
                LINE_STAT: line_base = 7'd0;
                LINE_CLK : line_base = 7'd16;
                LINE_PTP : line_base = 7'd32;
                LINE_TX  : line_base = 7'd48;
                LINE_RX  : line_base = 7'd64;
                LINE_CFG : line_base = 7'd80;
                default  : line_base = 7'd96;
            endcase
        end
    endfunction

    function [3:0] prefix_len;
        input [2:0] line;
        begin
            case (line)
                LINE_STAT: prefix_len = 4'd4; // STAT
                LINE_CLK : prefix_len = 4'd3; // CLK
                LINE_PTP : prefix_len = 4'd3; // PTP
                LINE_TX  : prefix_len = 4'd2; // TX
                LINE_RX  : prefix_len = 4'd2; // RX
                default  : prefix_len = 4'd3; // CFG
            endcase
        end
    endfunction

    function [7:0] prefix_char;
        input [2:0] line;
        input [3:0] pos;
        begin
            // pos == prefix_len(line) emits the comma after the prefix.
            if (pos == prefix_len(line)) begin
                prefix_char = 8'h2C;
            end else begin
                case (line)
                    LINE_STAT: begin
                        case (pos)
                            4'd0: prefix_char = 8'h53; // S
                            4'd1: prefix_char = 8'h54; // T
                            4'd2: prefix_char = 8'h41; // A
                            default: prefix_char = 8'h54; // T
                        endcase
                    end
                    LINE_CLK: begin
                        case (pos)
                            4'd0: prefix_char = 8'h43; // C
                            4'd1: prefix_char = 8'h4C; // L
                            default: prefix_char = 8'h4B; // K
                        endcase
                    end
                    LINE_PTP: begin
                        case (pos)
                            4'd0: prefix_char = 8'h50; // P
                            4'd1: prefix_char = 8'h54; // T
                            default: prefix_char = 8'h50; // P
                        endcase
                    end
                    LINE_TX: begin
                        prefix_char = (pos == 4'd0) ? 8'h54 : 8'h58; // TX
                    end
                    LINE_RX: begin
                        prefix_char = (pos == 4'd0) ? 8'h52 : 8'h58; // RX
                    end
                    LINE_DBG: begin
                        case (pos)
                            4'd0: prefix_char = 8'h44; // D
                            4'd1: prefix_char = 8'h42; // B
                            default: prefix_char = 8'h47; // G
                        endcase
                    end
                    default: begin
                        case (pos)
                            4'd0: prefix_char = 8'h43; // C
                            4'd1: prefix_char = 8'h46; // F
                            default: prefix_char = 8'h47; // G
                        endcase
                    end
                endcase
            end
        end
    endfunction

    function [7:0] hex_char;
        input [3:0] nibble;
        begin
            hex_char = (nibble < 4'd10) ? (8'h30 + {4'd0, nibble}) :
                                         (8'h41 + {4'd0, nibble - 4'd10});
        end
    endfunction

    function [7:0] word_hex_char;
        input [31:0] word;
        input [3:0]  pos;
        begin
            case (pos)
                4'd0: word_hex_char = hex_char(word[31:28]);
                4'd1: word_hex_char = hex_char(word[27:24]);
                4'd2: word_hex_char = hex_char(word[23:20]);
                4'd3: word_hex_char = hex_char(word[19:16]);
                4'd4: word_hex_char = hex_char(word[15:12]);
                4'd5: word_hex_char = hex_char(word[11:8]);
                4'd6: word_hex_char = hex_char(word[7:4]);
                default: word_hex_char = hex_char(word[3:0]);
            endcase
        end
    endfunction

    task t_latch_fields;
        begin
            // STAT line: backward-compatible Stage-1 status.
            field_latched_r[0]  <= uptime_s;
            field_latched_r[1]  <= rx_frame_count;
            field_latched_r[2]  <= rx_drop_count;
            field_latched_r[3]  <= tx_frame_count;
            field_latched_r[4]  <= {24'd0, status_flags};
            field_latched_r[5]  <= ptp_sync_count;
            field_latched_r[6]  <= ptp_follow_up_count;
            field_latched_r[7]  <= ptp_timeout_count;
            field_latched_r[8]  <= ptp_sequence_id;
            field_latched_r[9]  <= ptp_last_t1_low;
            field_latched_r[10] <= tod_rx_sync_count;
            field_latched_r[11] <= delay_req_count;
            field_latched_r[12] <= delay_resp_count;
            field_latched_r[13] <= delay_req_drop_count;
            field_latched_r[14] <= delay_req_last_seq;
            field_latched_r[15] <= delay_req_last_t4_low;

            // CLK line: clock/reset/ToD monitor.  Use this first when RX ToD or
            // recovered-clock bring-up is suspicious.
            field_latched_r[16] <= uptime_s;
            field_latched_r[17] <= {24'd0, status_flags};
            field_latched_r[18] <= status_ext;
            field_latched_r[19] <= tx_heartbeat_count;
            field_latched_r[20] <= rx_heartbeat_count;
            field_latched_r[21] <= tx_tod_low;
            field_latched_r[22] <= rx_tod_low;
            field_latched_r[23] <= tod_rx_sync_count;
            field_latched_r[24] <= stat_timeout_count;
            field_latched_r[25] <= stat_last_wait_cycles;
            field_latched_r[26] <= rx_status_available;
            field_latched_r[27] <= tx_tod_valid;
            field_latched_r[28] <= rx_tod_valid;
            field_latched_r[29] <= tod_sync_pending;
            field_latched_r[30] <= tx_tod_high;
            field_latched_r[31] <= rx_tod_high;

            // PTP line: master Sync/FUP source and last t1 details.
            field_latched_r[32] <= ptp_sync_count;
            field_latched_r[33] <= ptp_follow_up_count;
            field_latched_r[34] <= ptp_timeout_count;
            field_latched_r[35] <= ptp_sequence_id;
            field_latched_r[36] <= ptp_last_t1_high;
            field_latched_r[37] <= ptp_last_t1_low;
            field_latched_r[38] <= ptp_last_t1_seconds_low;
            field_latched_r[39] <= ptp_last_t1_nanoseconds;
            field_latched_r[40] <= ptp_master_busy;
            field_latched_r[41] <= tx_tod_valid;
            field_latched_r[42] <= tx_tod_high;
            field_latched_r[43] <= tx_tod_low;
            field_latched_r[44] <= delay_resp_count;
            field_latched_r[45] <= delay_req_count;
            field_latched_r[46] <= stat_timeout_count;
            field_latched_r[47] <= build_version;

            // TX line: MAC TX, mux, debug source and Delay_Resp source.
            field_latched_r[48] <= tx_frame_count;
            field_latched_r[49] <= tx_byte_count;
            field_latched_r[50] <= tx_last_error;
            field_latched_r[51] <= mux_delay_resp_frame_count;
            field_latched_r[52] <= mux_ptp_frame_count;
            field_latched_r[53] <= mux_debug_frame_count;
            field_latched_r[54] <= delay_resp_count;
            field_latched_r[55] <= delay_resp_last_seq;
            field_latched_r[56] <= delay_resp_last_t4_high;
            field_latched_r[57] <= delay_resp_last_t4_low;
            field_latched_r[58] <= delay_resp_busy;
            field_latched_r[59] <= test_tx_frame_count;
            field_latched_r[60] <= tx_heartbeat_count;
            field_latched_r[61] <= tx_tod_high;
            field_latched_r[62] <= tx_tod_low;
            field_latched_r[63] <= build_version;

            // RX line: MAC RX, Delay_Req parser, CDC drop and t4 details.
            field_latched_r[64] <= rx_frame_count;
            field_latched_r[65] <= rx_drop_count;
            field_latched_r[66] <= rx_last_error;
            field_latched_r[67] <= rx_heartbeat_count;
            field_latched_r[68] <= rx_busy;
            field_latched_r[69] <= rx_tod_valid;
            field_latched_r[70] <= rx_tod_high;
            field_latched_r[71] <= rx_tod_low;
            field_latched_r[72] <= delay_req_count;
            field_latched_r[73] <= delay_req_drop_count;
            field_latched_r[74] <= delay_req_non_delay_ptp_count;
            field_latched_r[75] <= delay_req_last_error;
            field_latched_r[76] <= delay_req_cdc_drop_count;
            field_latched_r[77] <= delay_req_last_seq;
            field_latched_r[78] <= delay_req_last_t4_high;
            field_latched_r[79] <= delay_req_last_t4_low;

            // CFG line: static identity and build parameters.  This line is
            // especially useful for catching DUT/Observer MAC/IP/domain mismatch.
            field_latched_r[80] <= local_mac_low;
            field_latched_r[81] <= local_mac_high;
            field_latched_r[82] <= dut_mac_low;
            field_latched_r[83] <= dut_mac_high;
            field_latched_r[84] <= local_ip;
            field_latched_r[85] <= dut_ip;
            field_latched_r[86] <= ptp_domain;
            field_latched_r[87] <= ptp_sync_interval;
            field_latched_r[88] <= uart_baud;
            field_latched_r[89] <= sys_clk_freq;
            field_latched_r[90] <= status_ext;
            field_latched_r[91] <= build_version;
            field_latched_r[92] <= 32'd0;
            field_latched_r[93] <= 32'd0;
            field_latched_r[94] <= 32'd0;
            field_latched_r[95] <= 32'd0;

            // DBG line: compact extra diagnostics for the current PTP bring-up
            // issue.  In a healthy Delay_Req path, rx_axis_timestamp_count
            // should rise with rx_frames and tod_sync_timeout_count should stop
            // rising once RX ToD has recovered.
            field_latched_r[96]  <= uptime_s;
            field_latched_r[97]  <= rx_axis_timestamp_count;
            field_latched_r[98]  <= tod_sync_timeout_count;
            field_latched_r[99]  <= tod_rx_sync_count;
            field_latched_r[100] <= rx_frame_count;
            field_latched_r[101] <= rx_drop_count;
            field_latched_r[102] <= rx_last_error;
            field_latched_r[103] <= delay_req_non_delay_ptp_count;
            field_latched_r[104] <= delay_req_last_error;
            field_latched_r[105] <= delay_req_count;
            field_latched_r[106] <= delay_resp_count;
            field_latched_r[107] <= rx_tod_valid;
            field_latched_r[108] <= tod_sync_pending;
            field_latched_r[109] <= rx_heartbeat_count;
            field_latched_r[110] <= tx_heartbeat_count;
            field_latched_r[111] <= build_version;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r         <= ST_IDLE;
            line_idx_r      <= LINE_STAT;
            prefix_idx_r    <= 4'd0;
            field_idx_r     <= 5'd0;
            hex_idx_r       <= 4'd0;
            abs_field_idx_r <= 7'd0;
            m_data          <= 8'd0;
            m_valid         <= 1'b0;
            for (i = 0; i < TOTAL_FIELDS; i = i + 1)
                field_latched_r[i] <= 32'd0;
        end else begin
            if (state_r == ST_IDLE) begin
                m_valid <= 1'b0;
                if (start_pulse) begin
                    t_latch_fields();
                    line_idx_r      <= LINE_STAT;
                    prefix_idx_r    <= 4'd0;
                    field_idx_r     <= 5'd0;
                    hex_idx_r       <= 4'd0;
                    abs_field_idx_r <= 7'd0;
                    m_data          <= prefix_char(LINE_STAT, 4'd0);
                    m_valid         <= 1'b1;
                    state_r         <= ST_PREFIX;
                end
            end else if (!m_valid || fire_w) begin
                case (state_r)
                    ST_PREFIX: begin
                        if (prefix_idx_r == prefix_len(line_idx_r)) begin
                            state_r     <= ST_FIELD;
                            field_idx_r <= 5'd0;
                            hex_idx_r   <= 4'd0;
                            m_data      <= word_hex_char(field_latched_r[abs_field_idx_r], 4'd0);
                        end else begin
                            prefix_idx_r <= prefix_idx_r + 4'd1;
                            m_data       <= prefix_char(line_idx_r, prefix_idx_r + 4'd1);
                        end
                        m_valid <= 1'b1;
                    end

                    ST_FIELD: begin
                        if (hex_idx_r == 4'd7) begin
                            if (field_idx_r == FIELD_LAST) begin
                                state_r <= ST_CR;
                                m_data  <= 8'h0D;
                            end else begin
                                state_r <= ST_COMMA;
                                m_data  <= 8'h2C;
                            end
                            hex_idx_r <= 4'd0;
                        end else begin
                            hex_idx_r <= hex_idx_r + 4'd1;
                            m_data    <= word_hex_char(field_latched_r[abs_field_idx_r], hex_idx_r + 4'd1);
                        end
                        m_valid <= 1'b1;
                    end

                    ST_COMMA: begin
                        field_idx_r     <= field_idx_r + 5'd1;
                        abs_field_idx_r <= abs_field_idx_r + 7'd1;
                        state_r         <= ST_FIELD;
                        hex_idx_r       <= 4'd0;
                        m_data          <= word_hex_char(field_latched_r[abs_field_idx_r + 7'd1], 4'd0);
                        m_valid         <= 1'b1;
                    end

                    ST_CR: begin
                        state_r <= ST_LF;
                        m_data  <= 8'h0A;
                        m_valid <= 1'b1;
                    end

                    ST_LF: begin
                        if (line_idx_r == LINE_LAST) begin
                            state_r <= ST_IDLE;
                            m_valid <= 1'b0;
                        end else begin
                            line_idx_r      <= line_idx_r + 3'd1;
                            prefix_idx_r    <= 4'd0;
                            field_idx_r     <= 5'd0;
                            hex_idx_r       <= 4'd0;
                            abs_field_idx_r <= line_base(line_idx_r + 3'd1);
                            m_data          <= prefix_char(line_idx_r + 3'd1, 4'd0);
                            m_valid         <= 1'b1;
                            state_r         <= ST_PREFIX;
                        end
                    end

                    default: begin
                        state_r <= ST_IDLE;
                        m_valid <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
`default_nettype wire
