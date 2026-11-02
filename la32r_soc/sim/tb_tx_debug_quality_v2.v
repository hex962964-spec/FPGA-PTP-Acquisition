`timescale 1ns / 1ps
`default_nettype none

module tb_tx_debug_quality_v2;
    reg clk_in = 1'b0;
    reg resetn = 1'b0;
    reg enable = 1'b0;
    reg clear = 1'b0;
    reg clear_status = 1'b0;
    reg [15:0] pkt_data = 16'd0;
    reg pkt_valid = 1'b0;
    reg pkt_last = 1'b0;
    reg pkt_ready = 1'b1;

    wire dbg_error_sticky;
    wire [31:0] dbg_good_packet_count;
    wire [31:0] dbg_bad_packet_count;
    wire [31:0] dbg_last_seq_id;
    wire [63:0] dbg_last_timestamp;
    wire [15:0] dbg_last_sample_count;
    wire [15:0] dbg_last_payload_count;
    wire [15:0] dbg_last_error_code;
    wire [15:0] dbg_last_error_word_index;
    wire [7:0] dbg_last_protocol_version;
    wire [1:0] dbg_last_quality;

    integer errors = 0;

    always #5 clk_in = ~clk_in;

    tx_debug_sink dut (
        .clk_in(clk_in),
        .resetn(resetn),
        .enable(enable),
        .clear(clear),
        .clear_status(clear_status),
        .pkt_data(pkt_data),
        .pkt_valid(pkt_valid),
        .pkt_last(pkt_last),
        .pkt_ready(pkt_ready),
        .dbg_error_sticky(dbg_error_sticky),
        .dbg_good_packet_count(dbg_good_packet_count),
        .dbg_bad_packet_count(dbg_bad_packet_count),
        .dbg_last_seq_id(dbg_last_seq_id),
        .dbg_last_timestamp(dbg_last_timestamp),
        .dbg_last_sample_count(dbg_last_sample_count),
        .dbg_last_payload_count(dbg_last_payload_count),
        .dbg_last_error_code(dbg_last_error_code),
        .dbg_last_error_word_index(dbg_last_error_word_index),
        .dbg_last_protocol_version(dbg_last_protocol_version),
        .dbg_last_quality(dbg_last_quality)
    );

    task send_word;
        input [15:0] value;
        input is_last;
        begin
            @(negedge clk_in);
            pkt_data = value;
            pkt_last = is_last;
            pkt_valid = 1'b1;
            @(negedge clk_in);
            pkt_valid = 1'b0;
            pkt_last = 1'b0;
        end
    endtask

    task send_packet;
        input [15:0] version_quality;
        input [31:0] seq_id;
        input [63:0] timestamp;
        begin
            send_word(16'h55AA, 1'b0);
            send_word(version_quality, 1'b0);
            send_word(seq_id[15:0], 1'b0);
            send_word(seq_id[31:16], 1'b0);
            send_word(16'h2468, 1'b0);
            send_word(16'd1, 1'b0);
            send_word(timestamp[15:0], 1'b0);
            send_word(timestamp[31:16], 1'b0);
            send_word(timestamp[47:32], 1'b0);
            send_word(timestamp[63:48], 1'b0);
            send_word(16'hCAFE, 1'b1);
            repeat (2) @(posedge clk_in);
        end
    endtask

    task expect_state;
        input [31:0] good_count;
        input [31:0] bad_count;
        input [7:0] version;
        input [1:0] quality;
        input [15:0] error_code;
        begin
            if (dbg_good_packet_count !== good_count) begin
                $display("ERROR good=%0d expected=%0d", dbg_good_packet_count, good_count);
                errors = errors + 1;
            end
            if (dbg_bad_packet_count !== bad_count) begin
                $display("ERROR bad=%0d expected=%0d", dbg_bad_packet_count, bad_count);
                errors = errors + 1;
            end
            if (dbg_last_protocol_version !== version) begin
                $display("ERROR version=%0d expected=%0d", dbg_last_protocol_version, version);
                errors = errors + 1;
            end
            if (dbg_last_quality !== quality) begin
                $display("ERROR quality=%0d expected=%0d", dbg_last_quality, quality);
                errors = errors + 1;
            end
            if (dbg_last_error_code !== error_code) begin
                $display("ERROR err=%0d expected=%0d", dbg_last_error_code, error_code);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk_in);
        resetn = 1'b1;
        repeat (2) @(posedge clk_in);
        enable = 1'b1;

        // v1 is accepted and represented as UNKNOWN by version=1, quality=0.
        send_packet(16'h0001, 32'd0, 64'd100);
        expect_state(32'd1, 32'd0, 8'd1, 2'd0, 16'd0);

        // v2 INVALID is parsed but excluded from valid measurement statistics.
        send_packet(16'h0002, 32'd1, 64'd200);
        expect_state(32'd1, 32'd1, 8'd2, 2'd0, 16'd7);

        send_packet(16'h0102, 32'd2, 64'd300);
        expect_state(32'd2, 32'd1, 8'd2, 2'd1, 16'd7);
        send_packet(16'h0202, 32'd3, 64'd400);
        expect_state(32'd3, 32'd1, 8'd2, 2'd2, 16'd7);
        send_packet(16'h0302, 32'd4, 64'd500);
        expect_state(32'd4, 32'd1, 8'd2, 2'd3, 16'd7);

        // Unknown version is a bad packet with the existing version error.
        send_packet(16'h0007, 32'd5, 64'd600);
        expect_state(32'd4, 32'd2, 8'd7, 2'd0, 16'd2);

        // Truncated v2 header retains the existing EARLY_LAST semantics.
        send_word(16'h55AA, 1'b0);
        send_word(16'h0202, 1'b0);
        send_word(16'd6, 1'b0);
        send_word(16'd0, 1'b0);
        send_word(16'h2468, 1'b1);
        repeat (2) @(posedge clk_in);
        if (dbg_bad_packet_count !== 32'd3 || dbg_last_error_code !== 16'd3 ||
            dbg_last_error_word_index !== 16'd4) begin
            $display("ERROR truncated packet semantics bad=%0d err=%0d word=%0d",
                     dbg_bad_packet_count, dbg_last_error_code,
                     dbg_last_error_word_index);
            errors = errors + 1;
        end

        repeat (100) @(posedge clk_in);
        if (errors != 0) begin
            $display("FAIL tb_tx_debug_quality_v2 errors=%0d", errors);
            $finish;
        end
        $display("PASS tb_tx_debug_quality_v2");
        $finish;
    end
endmodule

`default_nettype wire
