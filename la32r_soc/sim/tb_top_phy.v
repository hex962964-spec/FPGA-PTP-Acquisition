`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_top_phy
// -----------------------------------------------------------------------------
// 目标：
//   1) 仅验证 top_phy 的 RGMII RX 链路
//   2) 从 rgmii_rxd/rgmii_rx_ctl/rgmii_rxc 喂入一帧 PTPv2 Sync over UDP/IPv4
//   3) 检查 top_phy 是否能输出去 preamble / 去 FCS 的 MAC-side byte stream
//   4) 检查 ptp_rxstamp_valid 是否出现
//
// 使用说明：
//   - 该 testbench 默认自带一个“行为级 IDDR(SAME_EDGE) 模型”，用于纯 RTL 仿真。
//   - 如果你在 Vivado 里想直接使用 Xilinx UNISIM 的 IDDR，可在编译时定义：
//         +define+USE_XILINX_IDDR
//     这样下面的行为模型就不会参与编译。
//
// 重要说明：
//   - 这个 tb 是“验真”的，不是“保过”的。
//   - 如果你当前 rgmii_rx_if 的 DDR 拼字节逻辑有问题，这个 tb 会直接报 FAIL。
//////////////////////////////////////////////////////////////////////////////////

`ifndef USE_XILINX_IDDR
module IDDR #(
    parameter DDR_CLK_EDGE = "SAME_EDGE",
    parameter INIT_Q1      = 1'b0,
    parameter INIT_Q2      = 1'b0,
    parameter SRTYPE       = "ASYNC"
)(
    output reg Q1,
    output reg Q2,
    input      C,
    input      CE,
    input      D,
    input      R,
    input      S
);
    // 这里实现的是 Xilinx IDDR 的 SAME_EDGE 近似行为：
    //   - Q1：当前上升沿采样值
    //   - Q2：前一个下降沿采样值，在当前上升沿对齐输出
    reg q2_pipe;

    initial begin
        Q1      = INIT_Q1;
        Q2      = INIT_Q2;
        q2_pipe = INIT_Q2;
    end

    always @(negedge C or posedge R or posedge S) begin
        if (R)
            q2_pipe <= INIT_Q2;
        else if (S)
            q2_pipe <= 1'b1;
        else if (CE)
            q2_pipe <= D;
    end

    always @(posedge C or posedge R or posedge S) begin
        if (R) begin
            Q1 <= INIT_Q1;
            Q2 <= INIT_Q2;
        end else if (S) begin
            Q1 <= 1'b1;
            Q2 <= 1'b1;
        end else if (CE) begin
            Q1 <= D;
            Q2 <= q2_pipe;
        end
    end
endmodule
`endif

module tb_top_phy;

    localparam integer FRAME_LEN_NO_FCS = 86;
    localparam integer FRAME_LEN_ON_WIRE = 90;

    reg         clk125;
    reg         clk125_shift;
    reg         rst_n;
    reg         rx_rst_n;

    reg  [7:0]  s_axis_eth_tdata;
    reg         s_axis_eth_tvalid;
    reg         s_axis_eth_tlast;
    wire        s_axis_eth_tready;
    reg  [3:0]  s_axis_eth_tuser;
    reg  [63:0] ptp_tod_time_ns;
    reg         ptp_tod_time_valid;

    reg  [63:0] ptp_tod_time_ns_rx;
    reg         ptp_tod_time_valid_rx;

    wire [63:0] ptp_txstamp_ns;
    wire        ptp_txstamp_valid;
    wire [63:0] ptp_rxstamp_ns;
    wire        ptp_rxstamp_valid;
    wire        mac_link_up;

    wire [7:0]  m_axis_eth_rx_tdata;
    wire        m_axis_eth_rx_tvalid;
    wire        m_axis_eth_rx_tlast;

    wire        rgmii_txc;
    wire [3:0]  rgmii_txd;
    wire        rgmii_tx_ctl;

    reg         rgmii_rxc;
    reg  [3:0]  rgmii_rxd;
    reg         rgmii_rx_ctl;

    wire        phy_mdc;
    wire        phy_mdio;
    wire        phy_rst_n;

    reg [7:0] frame_no_fcs [0:FRAME_LEN_NO_FCS-1];
    reg [7:0] frame_on_wire[0:FRAME_LEN_ON_WIRE-1];

    integer i;
    integer rx_count;
    integer err_count;
    integer stamp_seen;
    reg [63:0] last_stamp_ns;
    reg [31:0] crc_work;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    top_phy #(
        .SIMULATION        (1'b1),
        .RESET_HOLD_CYCLES (4),
        .MIN_FRAME_BYTES   (60),
        .IFG_BYTES         (12)
    ) dut (
        .clk125               (clk125),
        .clk125_shift         (clk125_shift),
        .rst_n                (rst_n),
        .rx_rst_n             (rx_rst_n),
        .s_axis_eth_tdata     (s_axis_eth_tdata),
        .s_axis_eth_tvalid    (s_axis_eth_tvalid),
        .s_axis_eth_tlast     (s_axis_eth_tlast),
        .s_axis_eth_tready    (s_axis_eth_tready),
        .s_axis_eth_tuser     (s_axis_eth_tuser),
        .ptp_tod_time_ns      (ptp_tod_time_ns),
        .ptp_tod_time_valid   (ptp_tod_time_valid),
        .ptp_tod_time_ns_rx   (ptp_tod_time_ns_rx),
        .ptp_tod_time_valid_rx(ptp_tod_time_valid_rx),
        .ptp_txstamp_ns       (ptp_txstamp_ns),
        .ptp_txstamp_valid    (ptp_txstamp_valid),
        .ptp_rxstamp_ns       (ptp_rxstamp_ns),
        .ptp_rxstamp_valid    (ptp_rxstamp_valid),
        .mac_link_up          (mac_link_up),
        .m_axis_eth_rx_tdata  (m_axis_eth_rx_tdata),
        .m_axis_eth_rx_tvalid (m_axis_eth_rx_tvalid),
        .m_axis_eth_rx_tlast  (m_axis_eth_rx_tlast),
        .rgmii_txc            (rgmii_txc),
        .rgmii_txd            (rgmii_txd),
        .rgmii_tx_ctl         (rgmii_tx_ctl),
        .rgmii_rxc            (rgmii_rxc),
        .rgmii_rxd            (rgmii_rxd),
        .rgmii_rx_ctl         (rgmii_rx_ctl),
        .phy_mdc              (phy_mdc),
        .phy_mdio             (phy_mdio),
        .phy_rst_n            (phy_rst_n)
    );

    // -------------------------------------------------------------------------
    // Clock generation
    // -------------------------------------------------------------------------
    initial begin
        clk125 = 1'b0;
        forever #4 clk125 = ~clk125;     // 125 MHz
    end

    initial begin
        rgmii_rxc = 1'b0;
        forever #4 rgmii_rxc = ~rgmii_rxc; // 125 MHz RX clock
    end

    initial begin
        clk125_shift = 1'b0;
        #2;
        forever #4 clk125_shift = ~clk125_shift; // +90 deg for TX path (idle here)
    end

    // -------------------------------------------------------------------------
    // Simple free-running ToD in TX/RX domains
    // -------------------------------------------------------------------------
    always @(posedge clk125 or negedge rst_n) begin
        if (!rst_n) begin
            ptp_tod_time_ns    <= 64'd0;
            ptp_tod_time_valid <= 1'b0;
        end else begin
            ptp_tod_time_ns    <= ptp_tod_time_ns + 64'd8;
            ptp_tod_time_valid <= 1'b1;
        end
    end

    always @(posedge rgmii_rxc or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            ptp_tod_time_ns_rx    <= 64'd0;
            ptp_tod_time_valid_rx <= 1'b0;
        end else begin
            ptp_tod_time_ns_rx    <= ptp_tod_time_ns_rx + 64'd8;
            ptp_tod_time_valid_rx <= 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Ethernet CRC32 helper (same bit order as DUT TX path)
    // -------------------------------------------------------------------------
    function [31:0] crc32_next_d8_tb;
        input [31:0] crc_in;
        input [7:0]  data_in;
        integer j;
        reg [31:0] crc_tmp;
        reg [7:0]  data_tmp;
        begin
            crc_tmp  = crc_in;
            data_tmp = data_in;
            for (j = 0; j < 8; j = j + 1) begin
                if ((crc_tmp[0] ^ data_tmp[0]) == 1'b1)
                    crc_tmp = (crc_tmp >> 1) ^ 32'hEDB8_8320;
                else
                    crc_tmp = (crc_tmp >> 1);
                data_tmp = data_tmp >> 1;
            end
            crc32_next_d8_tb = crc_tmp;
        end
    endfunction

    // -------------------------------------------------------------------------
    // One RGMII wire byte:
    //   rising edge  -> low nibble, RX_DV
    //   falling edge -> high nibble, RX_DV ^ RX_ER
    // -------------------------------------------------------------------------
    task send_rgmii_byte;
        input [7:0] data_b;
        input       dv_b;
        input       er_b;
        begin
            rgmii_rxd    = data_b[3:0];
            rgmii_rx_ctl = dv_b;
            @(posedge rgmii_rxc);
            #1;
            rgmii_rxd    = data_b[7:4];
            rgmii_rx_ctl = dv_b ^ er_b;
            @(negedge rgmii_rxc);
            #1;
        end
    endtask

    task send_idle_bytes;
        input integer idle_n;
        integer k;
        begin
            for (k = 0; k < idle_n; k = k + 1) begin
                send_rgmii_byte(8'h00, 1'b0, 1'b0);
            end
        end
    endtask

    task send_ptp_frame_on_rgmii;
        integer k;
        begin
            // 对齐到一个完整 byte 的起点：从 negedge 后开始准备下一个 rising nibble
            @(negedge rgmii_rxc);
            #1;

            // 7x55 + D5
            for (k = 0; k < 7; k = k + 1)
                send_rgmii_byte(8'h55, 1'b1, 1'b0);
            send_rgmii_byte(8'hD5, 1'b1, 1'b0);

            // Ethernet II frame + FCS
            for (k = 0; k < FRAME_LEN_ON_WIRE; k = k + 1)
                send_rgmii_byte(frame_on_wire[k], 1'b1, 1'b0);

            // 释放为 idle
            send_idle_bytes(4);
        end
    endtask

    // -------------------------------------------------------------------------
    // Scoreboard / monitor
    //   用 negedge 采样，避开 DUT 在 posedge 的 NBA 更新顺序问题
    // -------------------------------------------------------------------------
    always @(negedge rgmii_rxc) begin
        if (rx_rst_n) begin
            if (ptp_rxstamp_valid) begin
                stamp_seen    = stamp_seen + 1;
                last_stamp_ns = ptp_rxstamp_ns;
                $display("[%0t] INFO: ptp_rxstamp_valid=1, stamp_ns=%0d", $time, ptp_rxstamp_ns);
            end

            if (m_axis_eth_rx_tvalid) begin
                if (rx_count >= FRAME_LEN_NO_FCS) begin
                    err_count = err_count + 1;
                    $display("[%0t] ERROR: RX output longer than expected, extra byte=0x%02x", $time, m_axis_eth_rx_tdata);
                end else begin
                    if (m_axis_eth_rx_tdata !== frame_no_fcs[rx_count]) begin
                        err_count = err_count + 1;
                        $display("[%0t] ERROR: byte[%0d] mismatch, exp=0x%02x, got=0x%02x",
                                 $time, rx_count, frame_no_fcs[rx_count], m_axis_eth_rx_tdata);
                    end

                    if (m_axis_eth_rx_tlast !== (rx_count == FRAME_LEN_NO_FCS-1)) begin
                        err_count = err_count + 1;
                        $display("[%0t] ERROR: byte[%0d] tlast mismatch, exp=%0d, got=%0d",
                                 $time, rx_count, (rx_count == FRAME_LEN_NO_FCS-1), m_axis_eth_rx_tlast);
                    end
                end

                rx_count = rx_count + 1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Build one concrete PTPv2 Sync / UDP / IPv4 frame
    //   DA  = 01:1B:19:00:00:00  (PTP multicast)
    //   SA  = 02:00:00:00:00:01
    //   IP  = 192.168.1.10 -> 224.0.1.129
    //   UDP = 319 -> 319
    //   PTP = v2 Sync, messageLength = 44 bytes
    // -------------------------------------------------------------------------
    initial begin
        for (i = 0; i < FRAME_LEN_NO_FCS; i = i + 1)
            frame_no_fcs[i] = 8'h00;
        for (i = 0; i < FRAME_LEN_ON_WIRE; i = i + 1)
            frame_on_wire[i] = 8'h00;

        frame_no_fcs[00] = 8'h01;
        frame_no_fcs[01] = 8'h1b;
        frame_no_fcs[02] = 8'h19;
        frame_no_fcs[03] = 8'h00;
        frame_no_fcs[04] = 8'h00;
        frame_no_fcs[05] = 8'h00;
        frame_no_fcs[06] = 8'h02;
        frame_no_fcs[07] = 8'h00;
        frame_no_fcs[08] = 8'h00;
        frame_no_fcs[09] = 8'h00;
        frame_no_fcs[10] = 8'h00;
        frame_no_fcs[11] = 8'h01;
        frame_no_fcs[12] = 8'h08;
        frame_no_fcs[13] = 8'h00;
        frame_no_fcs[14] = 8'h45;
        frame_no_fcs[15] = 8'h00;
        frame_no_fcs[16] = 8'h00;
        frame_no_fcs[17] = 8'h48;
        frame_no_fcs[18] = 8'h12;
        frame_no_fcs[19] = 8'h34;
        frame_no_fcs[20] = 8'h40;
        frame_no_fcs[21] = 8'h00;
        frame_no_fcs[22] = 8'h40;
        frame_no_fcs[23] = 8'h11;
        frame_no_fcs[24] = 8'h85;
        frame_no_fcs[25] = 8'h3d;
        frame_no_fcs[26] = 8'hc0;
        frame_no_fcs[27] = 8'ha8;
        frame_no_fcs[28] = 8'h01;
        frame_no_fcs[29] = 8'h0a;
        frame_no_fcs[30] = 8'he0;
        frame_no_fcs[31] = 8'h00;
        frame_no_fcs[32] = 8'h01;
        frame_no_fcs[33] = 8'h81;
        frame_no_fcs[34] = 8'h01;
        frame_no_fcs[35] = 8'h3f;
        frame_no_fcs[36] = 8'h01;
        frame_no_fcs[37] = 8'h3f;
        frame_no_fcs[38] = 8'h00;
        frame_no_fcs[39] = 8'h34;
        frame_no_fcs[40] = 8'h00;
        frame_no_fcs[41] = 8'h00;
        frame_no_fcs[42] = 8'h00;
        frame_no_fcs[43] = 8'h02;
        frame_no_fcs[44] = 8'h00;
        frame_no_fcs[45] = 8'h2c;
        frame_no_fcs[46] = 8'h00;
        frame_no_fcs[47] = 8'h00;
        frame_no_fcs[48] = 8'h00;
        frame_no_fcs[49] = 8'h00;
        frame_no_fcs[50] = 8'h00;
        frame_no_fcs[51] = 8'h00;
        frame_no_fcs[52] = 8'h00;
        frame_no_fcs[53] = 8'h00;
        frame_no_fcs[54] = 8'h00;
        frame_no_fcs[55] = 8'h00;
        frame_no_fcs[56] = 8'h00;
        frame_no_fcs[57] = 8'h00;
        frame_no_fcs[58] = 8'h00;
        frame_no_fcs[59] = 8'h00;
        frame_no_fcs[60] = 8'h00;
        frame_no_fcs[61] = 8'h00;
        frame_no_fcs[62] = 8'h02;
        frame_no_fcs[63] = 8'h00;
        frame_no_fcs[64] = 8'h00;
        frame_no_fcs[65] = 8'hff;
        frame_no_fcs[66] = 8'hfe;
        frame_no_fcs[67] = 8'h00;
        frame_no_fcs[68] = 8'h00;
        frame_no_fcs[69] = 8'h01;
        frame_no_fcs[70] = 8'h00;
        frame_no_fcs[71] = 8'h01;
        frame_no_fcs[72] = 8'h00;
        frame_no_fcs[73] = 8'h01;
        frame_no_fcs[74] = 8'h00;
        frame_no_fcs[75] = 8'h00;
        frame_no_fcs[76] = 8'h00;
        frame_no_fcs[77] = 8'h00;
        frame_no_fcs[78] = 8'h00;
        frame_no_fcs[79] = 8'h00;
        frame_no_fcs[80] = 8'h00;
        frame_no_fcs[81] = 8'h00;
        frame_no_fcs[82] = 8'h00;
        frame_no_fcs[83] = 8'h00;
        frame_no_fcs[84] = 8'h00;
        frame_no_fcs[85] = 8'h00;

        crc_work = 32'hFFFF_FFFF;
        for (i = 0; i < FRAME_LEN_NO_FCS; i = i + 1) begin
            frame_on_wire[i] = frame_no_fcs[i];
            crc_work = crc32_next_d8_tb(crc_work, frame_no_fcs[i]);
        end
        crc_work = ~crc_work;
        frame_on_wire[FRAME_LEN_NO_FCS + 0] = crc_work[7:0];
        frame_on_wire[FRAME_LEN_NO_FCS + 1] = crc_work[15:8];
        frame_on_wire[FRAME_LEN_NO_FCS + 2] = crc_work[23:16];
        frame_on_wire[FRAME_LEN_NO_FCS + 3] = crc_work[31:24];
    end

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        rst_n                = 1'b0;
        rx_rst_n             = 1'b0;
        s_axis_eth_tdata     = 8'h00;
        s_axis_eth_tvalid    = 1'b0;
        s_axis_eth_tlast     = 1'b0;
        s_axis_eth_tuser     = 4'h0;
        rgmii_rxd            = 4'h0;
        rgmii_rx_ctl         = 1'b0;
        ptp_tod_time_ns      = 64'd0;
        ptp_tod_time_valid   = 1'b0;
        ptp_tod_time_ns_rx   = 64'd0;
        ptp_tod_time_valid_rx= 1'b0;
        rx_count             = 0;
        err_count            = 0;
        stamp_seen           = 0;
        last_stamp_ns        = 64'd0;

        repeat (8) @(posedge clk125);
        rst_n    = 1'b1;
        rx_rst_n = 1'b1;

        // 等 PHY reset 释放并同步到 RX 域
        wait (dut.rx_enable_w === 1'b1);
        $display("[%0t] INFO: rx_enable_w is high, start feeding PTP frame", $time);
        $display("[%0t] INFO: expected FCS = %02x %02x %02x %02x",
                 $time,
                 frame_on_wire[86], frame_on_wire[87], frame_on_wire[88], frame_on_wire[89]);

        send_idle_bytes(4);
        send_ptp_frame_on_rgmii();

        // 给接收状态机留足时间把最后几个 pending byte 吐出来
        repeat (80) @(negedge rgmii_rxc);

        if (rx_count != FRAME_LEN_NO_FCS) begin
            err_count = err_count + 1;
            $display("[%0t] ERROR: RX byte count mismatch, exp=%0d, got=%0d",
                     $time, FRAME_LEN_NO_FCS, rx_count);
        end

        if (stamp_seen != 1) begin
            err_count = err_count + 1;
            $display("[%0t] ERROR: ptp_rxstamp_valid count mismatch, exp=1, got=%0d",
                     $time, stamp_seen);
        end

        if (err_count == 0) begin
            $display("============================================================");
            $display("PASS: top_phy RGMII RX path accepted the PTP frame correctly.");
            $display("      RX bytes = %0d, stamp_seen = %0d, stamp_ns = %0d",
                     rx_count, stamp_seen, last_stamp_ns);
            $display("============================================================");
        end else begin
            $display("============================================================");
            $display("FAIL: top_phy RGMII RX path did not pass the PTP frame test.");
            $display("      err_count = %0d, rx_count = %0d, stamp_seen = %0d", err_count, rx_count, stamp_seen);
            $display("      若 rx_count=0 或 preamble 阶段就丢包，优先检查 rgmii_rx_if 的 DDR 拼字节逻辑。");
            $display("============================================================");
        end

        $finish;
    end

    // 可选波形
    initial begin
        $dumpfile("tb_top_phy.vcd");
        $dumpvars(0, tb_top_phy);
    end

endmodule
