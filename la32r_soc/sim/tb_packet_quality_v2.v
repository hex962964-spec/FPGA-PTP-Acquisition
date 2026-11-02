`timescale 1ns / 1ps
`default_nettype none

module tb_packet_quality_v2;
    reg clk_in = 1'b0;
    reg resetn = 1'b0;
    reg cfg_enable = 1'b0;
    reg cfg_clear_status = 1'b0;
    reg [31:0] cfg_sample_period = 32'd1;
    reg [15:0] cfg_samples_per_packet = 16'd2;
    reg [3:0] cfg_mode = 4'd0;
    reg [15:0] cfg_channel_id = 16'h2468;
    reg cfg_src_sel = 1'b1;
    reg [1:0] cfg_time_quality = 2'd2;
    reg [63:0] ext_timestamp_ns = 64'h1122_3344_5566_7788;
    reg ext_timestamp_valid = 1'b1;
    reg [15:0] sample_data_in = 16'd0;
    reg sample_valid_in = 1'b0;
    wire sample_ready_out;
    wire [15:0] pkt_data;
    wire pkt_valid;
    wire pkt_last;
    reg pkt_ready = 1'b1;
    wire stat_busy;
    wire stat_error;
    wire [31:0] stat_fifo_level;
    wire [31:0] stat_packet_count;
    wire [63:0] stat_last_timestamp;

    integer packet_index = 0;
    integer word_index = 0;
    integer timeout_cycles = 0;
    integer error_count = 0;

    always #5 clk_in = ~clk_in;

    acquisition_core dut (
        .clk_in(clk_in),
        .resetn(resetn),
        .cfg_enable(cfg_enable),
        .cfg_clear_status(cfg_clear_status),
        .cfg_sample_period(cfg_sample_period),
        .cfg_samples_per_packet(cfg_samples_per_packet),
        .cfg_mode(cfg_mode),
        .cfg_channel_id(cfg_channel_id),
        .cfg_src_sel(cfg_src_sel),
        .cfg_time_quality(cfg_time_quality),
        .ext_timestamp_ns(ext_timestamp_ns),
        .ext_timestamp_valid(ext_timestamp_valid),
        .sample_data_in(sample_data_in),
        .sample_valid_in(sample_valid_in),
        .sample_ready_out(sample_ready_out),
        .pkt_data(pkt_data),
        .pkt_valid(pkt_valid),
        .pkt_last(pkt_last),
        .pkt_ready(pkt_ready),
        .stat_busy(stat_busy),
        .stat_error(stat_error),
        .stat_fifo_level(stat_fifo_level),
        .stat_packet_count(stat_packet_count),
        .stat_last_timestamp(stat_last_timestamp)
    );

    task send_sample;
        input [15:0] value;
        begin
            while (!sample_ready_out)
                @(negedge clk_in);
            sample_data_in = value;
            sample_valid_in = 1'b1;
            @(negedge clk_in);
            sample_valid_in = 1'b0;
        end
    endtask

    task expect_word;
        input [15:0] expected;
        begin
            if (pkt_data !== expected) begin
                $display("ERROR packet=%0d word=%0d got=%h expected=%h", packet_index,
                         word_index, pkt_data, expected);
                error_count = error_count + 1;
            end
        end
    endtask

    always @(posedge clk_in) begin
        if (resetn && pkt_valid && pkt_ready) begin
            case (word_index)
                0: expect_word(16'h55AA);
                1: expect_word(packet_index == 0 ? 16'h0202 : 16'h0302);
                2: expect_word(packet_index[15:0]);
                3: expect_word(16'd0);
                4: expect_word(16'h2468);
                5: expect_word(16'd2);
                6: expect_word(16'h7788);
                7: expect_word(16'h5566);
                8: expect_word(16'h3344);
                9: expect_word(16'h1122);
                10: expect_word(packet_index == 0 ? 16'hA001 : 16'hB001);
                11: expect_word(packet_index == 0 ? 16'hA002 : 16'hB002);
                default: begin
                    $display("ERROR packet emitted more than 12 words");
                    error_count = error_count + 1;
                end
            endcase

            if (pkt_last !== (word_index == 11)) begin
                $display("ERROR packet=%0d word=%0d pkt_last=%b", packet_index,
                         word_index, pkt_last);
                error_count = error_count + 1;
            end

            if (word_index == 11) begin
                packet_index = packet_index + 1;
                word_index = 0;
            end else begin
                word_index = word_index + 1;
            end
        end
    end

    initial begin
        repeat (5) @(posedge clk_in);
        resetn = 1'b1;
        repeat (3) @(posedge clk_in);
        cfg_enable = 1'b1;

        send_sample(16'hA001);
        cfg_time_quality = 2'd3;
        send_sample(16'hA002);
        send_sample(16'hB001);
        send_sample(16'hB002);

        while ((packet_index < 2) && (timeout_cycles < 400)) begin
            @(posedge clk_in);
            timeout_cycles = timeout_cycles + 1;
        end
        // Vivado launch_simulation advances 1000 ns before the caller issues
        // "run all". Keep this test alive beyond that boundary so the batch
        // driver does not invoke run on an already-finished snapshot.
        repeat (100) @(posedge clk_in);
        if (packet_index != 2) begin
            $display("ERROR timeout packet_index=%0d word_index=%0d", packet_index, word_index);
            error_count = error_count + 1;
        end
        if (stat_error !== 1'b0) begin
            $display("ERROR acquisition core asserted stat_error");
            error_count = error_count + 1;
        end
        if (error_count != 0) begin
            $display("FAIL tb_packet_quality_v2 errors=%0d", error_count);
            $finish;
        end

        $display("PASS tb_packet_quality_v2");
        $finish;
    end
endmodule

`default_nettype wire
