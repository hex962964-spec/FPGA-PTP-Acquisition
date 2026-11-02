`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: tx_debug_sink
// Description:
//   Passive packet-stream monitor / debug sink.
//
//   IMPORTANT:
//   - This module DOES NOT drive back-pressure to the packet source.
//   - It only snoops words that have already completed the downstream
//     handshake: (pkt_valid && pkt_ready).
//   - Therefore it can be inserted inside net_acq_wrapper without changing the
//     upper packet-stream interface or the existing SoC integration.
//
// Function:
//   1) Parse the packet stream produced by packet_formatter.
//   2) Check the fixed V1/V2 header format:
//        word0 : 16'h55AA   magic
//        word1 : [9:8] quality, [7:0] version (v1 is exactly 16'h0001)
//        word2 : seq_id[15:0]
//        word3 : seq_id[31:16]
//        word4 : channel_id[15:0]   (currently only captured implicitly)
//        word5 : sample_count[15:0]
//        word6 : timestamp[15:0]
//        word7 : timestamp[31:16]
//        word8 : timestamp[47:32]
//        word9 : timestamp[63:48]
//   3) Check basic packet consistency:
//        - magic/version correct
//        - seq_id monotonically increments by 1
//        - timestamp monotonically increases packet by packet
//        - pkt_last appears at the final payload word
//        - for zero-payload packet, packet may legally end right after header
//   4) Accumulate debug statistics for future CSR expansion.
//
// Current usage in wrapper:
//   - Only dbg_error_sticky is fed back to wrapper status (OR into stat_error).
//   - Other counters are kept for future extension and for easier waveform debug.
//
// Notes:
//   - This monitor clears on reset, on wrapper soft reset, or when acquisition
//     is disabled.
//   - 新增 clear_status：仅清统计/粘滞状态，不清当前解析上下文与 expected_seq_id。
//   - 这样可以避免 clear_status 后把 seq expectation 错误清回 0，进而在下一包产生
//     伪造的 seq mismatch。
//////////////////////////////////////////////////////////////////////////////////

module tx_debug_sink(
    input               clk_in,
    input               resetn,

    input               enable,
    input               clear,
    input               clear_status,

    input       [15:0]  pkt_data,
    input               pkt_valid,
    input               pkt_last,
    input               pkt_ready,

    output  reg         dbg_error_sticky,
    output  reg [31:0]  dbg_good_packet_count,
    output  reg [31:0]  dbg_bad_packet_count,
    output  reg [31:0]  dbg_last_seq_id,
    output  reg [63:0]  dbg_last_timestamp,
    output  reg [15:0]  dbg_last_sample_count,
    output  reg [15:0]  dbg_last_payload_count,
    output  reg [15:0]  dbg_last_error_code,
    output  reg [15:0]  dbg_last_error_word_index,
    output  reg [7:0]   dbg_last_protocol_version,
    output  reg [1:0]   dbg_last_quality
);

    localparam [15:0] HEADER_MAGIC      = 16'h55AA;
    localparam [15:0] HEADER_VERSION_V1 = 16'h0001;
    localparam [7:0]  PROTOCOL_V2       = 8'h02;
    localparam [15:0] HEADER_WORD_NUM   = 16'd10;

    localparam [15:0] ERR_NONE          = 16'd0;
    localparam [15:0] ERR_MAGIC         = 16'd1;
    localparam [15:0] ERR_VERSION       = 16'd2;
    localparam [15:0] ERR_EARLY_LAST    = 16'd3;
    localparam [15:0] ERR_SEQ_MISMATCH  = 16'd4;
    localparam [15:0] ERR_TS_NON_MONO   = 16'd5;
    localparam [15:0] ERR_MISSING_LAST  = 16'd6;
    localparam [15:0] ERR_TIME_INVALID  = 16'd7;

    reg         in_packet;
    reg         finalize_pending;
    reg         packet_has_error;
    reg         have_prev_packet;

    reg [15:0]  word_index;
    reg [15:0]  payload_words_seen;
    reg [15:0]  finalize_payload_count;

    reg [31:0]  current_seq_id;
    reg [15:0]  current_sample_count;
    reg [63:0]  current_timestamp;
    reg [7:0]   current_protocol_version;
    reg [1:0]   current_quality;
    reg [31:0]  expected_seq_id;

    wire        beat_accepted;
    assign beat_accepted = enable && pkt_valid && pkt_ready;

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            in_packet                 <= 1'b0;
            finalize_pending          <= 1'b0;
            packet_has_error          <= 1'b0;
            have_prev_packet          <= 1'b0;

            word_index                <= 16'd0;
            payload_words_seen        <= 16'd0;
            finalize_payload_count    <= 16'd0;

            current_seq_id            <= 32'd0;
            current_sample_count      <= 16'd0;
            current_timestamp         <= 64'd0;
            current_protocol_version  <= 8'd0;
            current_quality           <= 2'd0;
            expected_seq_id           <= 32'd0;

            dbg_error_sticky          <= 1'b0;
            dbg_good_packet_count     <= 32'd0;
            dbg_bad_packet_count      <= 32'd0;
            dbg_last_seq_id           <= 32'd0;
            dbg_last_timestamp        <= 64'd0;
            dbg_last_sample_count     <= 16'd0;
            dbg_last_payload_count    <= 16'd0;
            dbg_last_error_code       <= ERR_NONE;
            dbg_last_error_word_index <= 16'd0;
            dbg_last_protocol_version <= 8'd0;
            dbg_last_quality          <= 2'd0;
        end else if (clear || !enable) begin
            in_packet                 <= 1'b0;
            finalize_pending          <= 1'b0;
            packet_has_error          <= 1'b0;
            have_prev_packet          <= 1'b0;

            word_index                <= 16'd0;
            payload_words_seen        <= 16'd0;
            finalize_payload_count    <= 16'd0;

            current_seq_id            <= 32'd0;
            current_sample_count      <= 16'd0;
            current_timestamp         <= 64'd0;
            current_protocol_version  <= 8'd0;
            current_quality           <= 2'd0;
            expected_seq_id           <= 32'd0;

            dbg_error_sticky          <= 1'b0;
            dbg_good_packet_count     <= 32'd0;
            dbg_bad_packet_count      <= 32'd0;
            dbg_last_seq_id           <= 32'd0;
            dbg_last_timestamp        <= 64'd0;
            dbg_last_sample_count     <= 16'd0;
            dbg_last_payload_count    <= 16'd0;
            dbg_last_error_code       <= ERR_NONE;
            dbg_last_error_word_index <= 16'd0;
            dbg_last_protocol_version <= 8'd0;
            dbg_last_quality          <= 2'd0;
        end else begin
            // clear_status 只清监视统计/粘滞状态，不打断当前解析中的包，
            // 也不改 expected_seq_id，避免下一包误报 seq mismatch。
            if (clear_status) begin
                have_prev_packet          <= 1'b0;
                dbg_error_sticky          <= 1'b0;
                dbg_good_packet_count     <= 32'd0;
                dbg_bad_packet_count      <= 32'd0;
                dbg_last_seq_id           <= 32'd0;
                dbg_last_timestamp        <= 64'd0;
                dbg_last_sample_count     <= 16'd0;
                dbg_last_payload_count    <= 16'd0;
                dbg_last_error_code       <= ERR_NONE;
                dbg_last_error_word_index <= 16'd0;
                dbg_last_protocol_version <= 8'd0;
                dbg_last_quality          <= 2'd0;
            end

            if (finalize_pending) begin
                dbg_last_seq_id        <= current_seq_id;
                dbg_last_timestamp     <= current_timestamp;
                dbg_last_sample_count  <= current_sample_count;
                dbg_last_payload_count <= finalize_payload_count;
                dbg_last_protocol_version <= current_protocol_version;
                dbg_last_quality          <= current_quality;

                if (packet_has_error)
                    dbg_bad_packet_count <= dbg_bad_packet_count + 32'd1;
                else
                    dbg_good_packet_count <= dbg_good_packet_count + 32'd1;

                expected_seq_id  <= current_seq_id + 32'd1;
                have_prev_packet <= 1'b1;
                finalize_pending <= 1'b0;
                packet_has_error <= 1'b0;
            end

            if (beat_accepted) begin
                if (!in_packet) begin
                    in_packet            <= 1'b1;
                    word_index           <= 16'd1;
                    payload_words_seen   <= 16'd0;
                    current_seq_id       <= 32'd0;
                    current_sample_count <= 16'd0;
                    current_timestamp    <= 64'd0;
                    current_protocol_version <= 8'd0;
                    current_quality      <= 2'd0;
                    packet_has_error     <= 1'b0;

                    if (pkt_data != HEADER_MAGIC) begin
                        dbg_error_sticky          <= 1'b1;
                        packet_has_error          <= 1'b1;
                        dbg_last_error_code       <= ERR_MAGIC;
                        dbg_last_error_word_index <= 16'd0;
                    end

                    if (pkt_last) begin
                        if (pkt_data == HEADER_MAGIC) begin
                            dbg_error_sticky          <= 1'b1;
                            packet_has_error          <= 1'b1;
                            dbg_last_error_code       <= ERR_EARLY_LAST;
                            dbg_last_error_word_index <= 16'd0;
                        end

                        in_packet              <= 1'b0;
                        word_index             <= 16'd0;
                        finalize_pending       <= 1'b1;
                        finalize_payload_count <= 16'd0;
                    end
                end else begin
                    case (word_index)
                        16'd1: begin
                            current_protocol_version <= pkt_data[7:0];

                            if (pkt_data == HEADER_VERSION_V1) begin
                                // V1 has no quality field. The version/quality
                                // pair (1,0) represents UNKNOWN to software.
                                current_quality <= 2'd0;
                            end else if (pkt_data[7:0] == PROTOCOL_V2) begin
                                current_quality <= pkt_data[9:8];
                                if (pkt_data[9:8] == 2'd0) begin
                                    if (!packet_has_error) begin
                                        dbg_last_error_code       <= ERR_TIME_INVALID;
                                        dbg_last_error_word_index <= 16'd1;
                                    end
                                    dbg_error_sticky <= 1'b1;
                                    packet_has_error <= 1'b1;
                                end
                            end else begin
                                current_quality <= 2'd0;
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_VERSION;
                                    dbg_last_error_word_index <= 16'd1;
                                end
                                dbg_error_sticky <= 1'b1;
                                packet_has_error <= 1'b1;
                            end

                            if (pkt_last) begin
                                if (!packet_has_error &&
                                    ((pkt_data == HEADER_VERSION_V1) ||
                                     ((pkt_data[7:0] == PROTOCOL_V2) &&
                                      (pkt_data[9:8] != 2'd0)))) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd1;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd2;
                            end
                        end

                        16'd2: begin
                            current_seq_id[15:0] <= pkt_data;

                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd2;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd3;
                            end
                        end

                        16'd3: begin
                            current_seq_id[31:16] <= pkt_data;

                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd3;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd4;
                            end
                        end

                        16'd4: begin
                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd4;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd5;
                            end
                        end

                        16'd5: begin
                            current_sample_count <= pkt_data;

                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd5;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd6;
                            end
                        end

                        16'd6: begin
                            current_timestamp[15:0] <= pkt_data;

                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd6;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd7;
                            end
                        end

                        16'd7: begin
                            current_timestamp[31:16] <= pkt_data;

                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd7;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd8;
                            end
                        end

                        16'd8: begin
                            current_timestamp[47:32] <= pkt_data;

                            if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd8;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index <= 16'd9;
                            end
                        end

                        16'd9: begin
                            current_timestamp[63:48] <= pkt_data;

                            if (current_seq_id != expected_seq_id) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_SEQ_MISMATCH;
                                    dbg_last_error_word_index <= 16'd9;
                                end
                                dbg_error_sticky <= 1'b1;
                                packet_has_error <= 1'b1;
                            end else if (have_prev_packet && ({pkt_data, current_timestamp[47:0]} <= dbg_last_timestamp)) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_TS_NON_MONO;
                                    dbg_last_error_word_index <= 16'd9;
                                end
                                dbg_error_sticky <= 1'b1;
                                packet_has_error <= 1'b1;
                            end

                            if (current_sample_count == 16'd0) begin
                                in_packet              <= 1'b0;
                                word_index             <= 16'd0;
                                payload_words_seen     <= 16'd0;
                                finalize_pending       <= 1'b1;
                                finalize_payload_count <= 16'd0;
                            end else if (pkt_last) begin
                                if (!packet_has_error) begin
                                    dbg_last_error_code       <= ERR_EARLY_LAST;
                                    dbg_last_error_word_index <= 16'd9;
                                end
                                dbg_error_sticky        <= 1'b1;
                                packet_has_error        <= 1'b1;
                                in_packet               <= 1'b0;
                                word_index              <= 16'd0;
                                payload_words_seen      <= 16'd0;
                                finalize_pending        <= 1'b1;
                                finalize_payload_count  <= 16'd0;
                            end else begin
                                word_index         <= HEADER_WORD_NUM;
                                payload_words_seen <= 16'd0;
                            end
                        end

                        default: begin
                            if ((payload_words_seen + 16'd1) < current_sample_count) begin
                                if (pkt_last) begin
                                    if (!packet_has_error) begin
                                        dbg_last_error_code       <= ERR_EARLY_LAST;
                                        dbg_last_error_word_index <= word_index;
                                    end
                                    dbg_error_sticky        <= 1'b1;
                                    packet_has_error        <= 1'b1;
                                    in_packet               <= 1'b0;
                                    word_index              <= 16'd0;
                                    finalize_pending        <= 1'b1;
                                    finalize_payload_count  <= payload_words_seen + 16'd1;
                                end else begin
                                    payload_words_seen <= payload_words_seen + 16'd1;
                                    word_index         <= word_index + 16'd1;
                                end
                            end else begin
                                if (!pkt_last) begin
                                    if (!packet_has_error) begin
                                        dbg_last_error_code       <= ERR_MISSING_LAST;
                                        dbg_last_error_word_index <= word_index;
                                    end
                                    dbg_error_sticky <= 1'b1;
                                    packet_has_error <= 1'b1;
                                end

                                in_packet              <= 1'b0;
                                word_index             <= 16'd0;
                                payload_words_seen     <= 16'd0;
                                finalize_pending       <= 1'b1;
                                finalize_payload_count <= current_sample_count;
                            end
                        end
                    endcase
                end
            end
        end
    end

endmodule
