`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: phy_mdio_ctrl
// Description:
//   Generic Clause-22 PHY management controller for the current RGMII backend.
//
//   Responsibilities:
//     1) Hold the external PHY reset pin low for RESET_HOLD_CYCLES.
//     2) Wait a short post-reset settling interval.
//     3) Discover PHY address automatically (or use a fixed address override).
//     4) Perform a generic initialization sequence:
//          - BMCR soft reset
//          - wait until BMCR.RESET clears (with timeout)
//          - ANAR advertisement write
//          - 1000BASE-T control write
//          - BMCR auto-neg enable + restart
//     5) Periodically poll standard status registers and derive:
//          - real link_up
//          - autoneg_complete
//          - speed / duplex
//     6) Accept software-triggered soft-reset / auto-negotiation-restart pulses.
//
//   Design policy:
//     - No vendor-specific registers are required for the baseline flow.
//     - The block is intentionally standard/generic first; vendor-specific board
//       tuning can be layered on later if a concrete PHY model is confirmed.
//
//   phy_status_word bit definition:
//     [31:24] last_error
//     [23:18] reserved
//     [17]    link_usable_for_current_mac  (1G + full duplex)
//     [16]    init_error_sticky
//     [15:11] phy_addr_active
//     [10]    phy_rst_n
//     [9]     mdio_busy
//     [8:7]   speed (00=10M, 01=100M, 10=1000M, 11=unknown)
//     [6]     full_duplex
//     [5]     autoneg_complete
//     [4]     link_up_raw
//     [3]     scan_done
//     [2]     init_done
//     [1]     init_busy
//     [0]     phy_present
//////////////////////////////////////////////////////////////////////////////////
module phy_mdio_ctrl #(
    parameter integer RESET_HOLD_CYCLES       = 1024,
    parameter integer POST_RESET_WAIT_CYCLES  = 125000,
    parameter integer POLL_INTERVAL_CYCLES    = 1250000,
    parameter integer MDIO_MDC_HALF_PERIOD    = 25,
    parameter integer BMCR_RESET_POLL_LIMIT   = 16,
    parameter         USE_FIXED_PHY_ADDR      = 1'b0,
    parameter [4:0]   FIXED_PHY_ADDR          = 5'd1,
    parameter [15:0]  PHY_ANAR_INIT           = 16'h0DE1,
    parameter [15:0]  PHY_CTRL1000_INIT       = 16'h0200,
    parameter [15:0]  PHY_BMCR_AN_INIT        = 16'h1200,
    parameter [15:0]  PHY_BMCR_RESET_INIT     = 16'h9000
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire        cmd_soft_reset_pulse,
    input  wire        cmd_restart_an_pulse,

    output wire        phy_mdc,
    output wire        phy_mdio_o,
    output wire        phy_mdio_oe,
    input  wire        phy_mdio_i,
    output wire        phy_rst_n,

    output wire        link_up,
    output wire [31:0] phy_status_word,
    output wire [127:0] phy_reg_snapshot
);

    // ------------------------------------------------------------------------
    // Standard Clause-22 register numbers
    // ------------------------------------------------------------------------
    localparam [4:0] REG_BMCR     = 5'h00;
    localparam [4:0] REG_BMSR     = 5'h01;
    localparam [4:0] REG_PHYID1   = 5'h02;
    localparam [4:0] REG_PHYID2   = 5'h03;
    localparam [4:0] REG_ANAR     = 5'h04;
    localparam [4:0] REG_ANLPAR   = 5'h05;
    localparam [4:0] REG_CTRL1000 = 5'h09;
    localparam [4:0] REG_STAT1000 = 5'h0A;

    // BMCR bits
    localparam [15:0] BMCR_RESET_BIT    = 16'h8000;
    localparam [15:0] BMCR_ANENABLE_BIT = 16'h1000;

    // BMSR bits
    localparam [15:0] BMSR_LSTATUS_BIT      = 16'h0004;
    localparam [15:0] BMSR_ANEGCOMPLETE_BIT = 16'h0020;

    // ANAR / ANLPAR bits
    localparam [15:0] ADV_10HALF_BIT  = 16'h0020;
    localparam [15:0] ADV_10FULL_BIT  = 16'h0040;
    localparam [15:0] ADV_100HALF_BIT = 16'h0080;
    localparam [15:0] ADV_100FULL_BIT = 16'h0100;

    // 1000BASE-T control / status bits
    localparam [15:0] ADV_1000HALF_BIT = 16'h0100;
    localparam [15:0] ADV_1000FULL_BIT = 16'h0200;
    localparam [15:0] LPA_1000HALF_BIT = 16'h0400;
    localparam [15:0] LPA_1000FULL_BIT = 16'h0800;

    // speed encoding exposed in PHY_STATUS.bit[8:7]
    localparam [1:0] SPEED_10      = 2'b00;
    localparam [1:0] SPEED_100     = 2'b01;
    localparam [1:0] SPEED_1000    = 2'b10;
    localparam [1:0] SPEED_UNKNOWN = 2'b11;

    // sticky error code
    localparam [7:0] ERR_NONE          = 8'd0;
    localparam [7:0] ERR_NO_PHY_FOUND  = 8'd1;
    localparam [7:0] ERR_BMCR_TIMEOUT  = 8'd2;

    localparam [5:0]
        ST_RESET_HOLD          = 6'd0,
        ST_POST_RESET_WAIT     = 6'd1,
        ST_SCAN_ID1_REQ        = 6'd2,
        ST_SCAN_ID1_WAIT       = 6'd3,
        ST_SCAN_ID2_REQ        = 6'd4,
        ST_SCAN_ID2_WAIT       = 6'd5,
        ST_INIT_BMCR_RESET_REQ = 6'd6,
        ST_INIT_BMCR_RESET_WAIT= 6'd7,
        ST_INIT_BMCR_POLL_REQ  = 6'd8,
        ST_INIT_BMCR_POLL_WAIT = 6'd9,
        ST_INIT_ANAR_REQ       = 6'd10,
        ST_INIT_ANAR_WAIT      = 6'd11,
        ST_INIT_GCTRL_REQ      = 6'd12,
        ST_INIT_GCTRL_WAIT     = 6'd13,
        ST_INIT_BMCR_AN_REQ    = 6'd14,
        ST_INIT_BMCR_AN_WAIT   = 6'd15,
        ST_CMD_RESTART_AN_REQ  = 6'd16,
        ST_CMD_RESTART_AN_WAIT = 6'd17,
        ST_POLL_WAIT           = 6'd18,
        ST_POLL_BMSR1_REQ      = 6'd19,
        ST_POLL_BMSR1_WAIT     = 6'd20,
        ST_POLL_BMSR2_REQ      = 6'd21,
        ST_POLL_BMSR2_WAIT     = 6'd22,
        ST_POLL_ANLPAR_REQ     = 6'd23,
        ST_POLL_ANLPAR_WAIT    = 6'd24,
        ST_POLL_GSTAT_REQ      = 6'd25,
        ST_POLL_GSTAT_WAIT     = 6'd26,
        ST_POLL_BMCR_REQ       = 6'd27,
        ST_POLL_BMCR_WAIT      = 6'd28;

    reg [5:0]  state_r;

    // ------------------------------------------------------------------------
    // PHY reset sequencing
    // ------------------------------------------------------------------------
    reg [31:0] reset_hold_cnt_r;
    reg [31:0] post_reset_wait_cnt_r;
    reg        phy_rst_n_r;

    assign phy_rst_n = phy_rst_n_r;

    // ------------------------------------------------------------------------
    // MDIO master command interface
    // ------------------------------------------------------------------------
    reg         mdio_start_r;
    reg         mdio_op_wr_r;
    reg  [4:0]  mdio_phy_addr_r;
    reg  [4:0]  mdio_reg_addr_r;
    reg  [15:0] mdio_wr_data_r;
    wire        mdio_busy_w;
    wire        mdio_done_w;
    wire [15:0] mdio_rd_data_w;
    wire        mdio_rd_valid_w;

    mdio_clause22_master #(
        .MDC_HALF_PERIOD (MDIO_MDC_HALF_PERIOD)
    ) u_mdio_clause22_master (
        .clk       (clk),
        .resetn    (resetn),
        .start     (mdio_start_r),
        .op_wr     (mdio_op_wr_r),
        .phy_addr  (mdio_phy_addr_r),
        .reg_addr  (mdio_reg_addr_r),
        .wr_data   (mdio_wr_data_r),
        .busy      (mdio_busy_w),
        .done      (mdio_done_w),
        .rd_data   (mdio_rd_data_w),
        .rd_valid  (mdio_rd_valid_w),
        .mdc       (phy_mdc),
        .mdio_o    (phy_mdio_o),
        .mdio_oe   (phy_mdio_oe),
        .mdio_i    (phy_mdio_i)
    );

    // ------------------------------------------------------------------------
    // Latches / raw register snapshots
    // ------------------------------------------------------------------------
    reg        phy_present_r;
    reg        scan_done_r;
    reg        init_busy_r;
    reg        init_done_r;
    reg        init_error_r;
    reg [7:0]  last_error_r;
    reg [4:0]  phy_addr_r;
    reg [4:0]  scan_addr_r;
    reg [15:0] phy_id1_r;
    reg [15:0] phy_id2_r;
    reg [15:0] phy_bmcr_r;
    reg [15:0] phy_bmsr_r;
    reg [15:0] phy_anar_r;
    reg [15:0] phy_anlpar_r;
    reg [15:0] phy_ctrl1000_r;
    reg [15:0] phy_stat1000_r;
    reg [7:0]  bmcr_reset_poll_cnt_r;
    reg [31:0] poll_wait_cnt_r;

    reg        pending_soft_reset_r;
    reg        pending_restart_an_r;

    wire [15:0] common_an_w;
    wire        common_1000_full_w;
    wire        common_1000_half_w;
    reg  [1:0]  resolved_speed_r;
    reg         resolved_full_duplex_r;

    assign common_an_w        = phy_anar_r & phy_anlpar_r;
    assign common_1000_full_w = phy_ctrl1000_r[9] & phy_stat1000_r[11];
    assign common_1000_half_w = phy_ctrl1000_r[8] & phy_stat1000_r[10];

    wire link_up_w       = phy_present_r && phy_rst_n_r && ((phy_bmsr_r & BMSR_LSTATUS_BIT) != 16'd0);
    wire autoneg_complete_w = phy_present_r && ((phy_bmsr_r & BMSR_ANEGCOMPLETE_BIT) != 16'd0);
    wire link_usable_w   = link_up_w && (resolved_speed_r == SPEED_1000) && resolved_full_duplex_r;

    assign link_up = link_up_w;

    assign phy_status_word = {
        last_error_r,
        6'd0,
        link_usable_w,
        init_error_r,
        phy_addr_r,
        phy_rst_n_r,
        mdio_busy_w,
        resolved_speed_r,
        resolved_full_duplex_r,
        autoneg_complete_w,
        link_up_w,
        scan_done_r,
        init_done_r,
        init_busy_r,
        phy_present_r
    };

    assign phy_reg_snapshot = {
        phy_id1_r,
        phy_id2_r,
        phy_bmcr_r,
        phy_bmsr_r,
        phy_anar_r,
        phy_anlpar_r,
        phy_ctrl1000_r,
        phy_stat1000_r
    };

    // ------------------------------------------------------------------------
    // Speed / duplex decode from standard AN registers
    // ------------------------------------------------------------------------
    always @(*) begin
        resolved_speed_r       = SPEED_UNKNOWN;
        resolved_full_duplex_r = 1'b0;

        if (link_up_w) begin
            if (common_1000_full_w) begin
                resolved_speed_r       = SPEED_1000;
                resolved_full_duplex_r = 1'b1;
            end else if (common_1000_half_w) begin
                resolved_speed_r       = SPEED_1000;
                resolved_full_duplex_r = 1'b0;
            end else if ((common_an_w & ADV_100FULL_BIT) != 16'd0) begin
                resolved_speed_r       = SPEED_100;
                resolved_full_duplex_r = 1'b1;
            end else if ((common_an_w & ADV_100HALF_BIT) != 16'd0) begin
                resolved_speed_r       = SPEED_100;
                resolved_full_duplex_r = 1'b0;
            end else if ((common_an_w & ADV_10FULL_BIT) != 16'd0) begin
                resolved_speed_r       = SPEED_10;
                resolved_full_duplex_r = 1'b1;
            end else if ((common_an_w & ADV_10HALF_BIT) != 16'd0) begin
                resolved_speed_r       = SPEED_10;
                resolved_full_duplex_r = 1'b0;
            end else if ((phy_bmcr_r & BMCR_ANENABLE_BIT) == 16'd0) begin
                if (phy_bmcr_r[6])
                    resolved_speed_r = SPEED_1000;
                else if (phy_bmcr_r[13])
                    resolved_speed_r = SPEED_100;
                else
                    resolved_speed_r = SPEED_10;
                resolved_full_duplex_r = phy_bmcr_r[8];
            end
        end
    end

    // ------------------------------------------------------------------------
    // Command latches
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            pending_soft_reset_r <= 1'b0;
            pending_restart_an_r <= 1'b0;
        end else begin
            if (cmd_soft_reset_pulse)
                pending_soft_reset_r <= 1'b1;
            if (cmd_restart_an_pulse)
                pending_restart_an_r <= 1'b1;

            if (state_r == ST_INIT_BMCR_RESET_REQ) begin
                pending_soft_reset_r <= 1'b0;
                pending_restart_an_r <= 1'b0;
            end else if (state_r == ST_CMD_RESTART_AN_REQ) begin
                pending_restart_an_r <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state_r                <= ST_RESET_HOLD;
            reset_hold_cnt_r       <= 32'd0;
            post_reset_wait_cnt_r  <= 32'd0;
            phy_rst_n_r            <= 1'b0;

            mdio_start_r           <= 1'b0;
            mdio_op_wr_r           <= 1'b0;
            mdio_phy_addr_r        <= 5'd0;
            mdio_reg_addr_r        <= 5'd0;
            mdio_wr_data_r         <= 16'd0;

            phy_present_r          <= 1'b0;
            scan_done_r            <= 1'b0;
            init_busy_r            <= 1'b0;
            init_done_r            <= 1'b0;
            init_error_r           <= 1'b0;
            last_error_r           <= ERR_NONE;
            phy_addr_r             <= 5'd0;
            scan_addr_r            <= 5'd0;
            phy_id1_r              <= 16'd0;
            phy_id2_r              <= 16'd0;
            phy_bmcr_r             <= 16'd0;
            phy_bmsr_r             <= 16'd0;
            phy_anar_r             <= 16'd0;
            phy_anlpar_r           <= 16'd0;
            phy_ctrl1000_r         <= 16'd0;
            phy_stat1000_r         <= 16'd0;
            bmcr_reset_poll_cnt_r  <= 8'd0;
            poll_wait_cnt_r        <= 32'd0;
        end else begin
            mdio_start_r <= 1'b0;

            case (state_r)
                ST_RESET_HOLD: begin
                    phy_present_r          <= 1'b0;
                    scan_done_r            <= 1'b0;
                    init_busy_r            <= 1'b0;
                    init_done_r            <= 1'b0;
                    init_error_r           <= 1'b0;
                    last_error_r           <= ERR_NONE;
                    phy_addr_r             <= 5'd0;
                    phy_id1_r              <= 16'd0;
                    phy_id2_r              <= 16'd0;
                    phy_bmcr_r             <= 16'd0;
                    phy_bmsr_r             <= 16'd0;
                    phy_anar_r             <= 16'd0;
                    phy_anlpar_r           <= 16'd0;
                    phy_ctrl1000_r         <= 16'd0;
                    phy_stat1000_r         <= 16'd0;
                    bmcr_reset_poll_cnt_r  <= 8'd0;
                    poll_wait_cnt_r        <= 32'd0;

                    if (reset_hold_cnt_r == (RESET_HOLD_CYCLES - 1)) begin
                        phy_rst_n_r           <= 1'b1;
                        reset_hold_cnt_r      <= 32'd0;
                        post_reset_wait_cnt_r <= 32'd0;
                        if (USE_FIXED_PHY_ADDR)
                            scan_addr_r <= FIXED_PHY_ADDR;
                        else
                            scan_addr_r <= 5'd0;
                        state_r <= ST_POST_RESET_WAIT;
                    end else begin
                        reset_hold_cnt_r <= reset_hold_cnt_r + 32'd1;
                    end
                end

                ST_POST_RESET_WAIT: begin
                    if (post_reset_wait_cnt_r == (POST_RESET_WAIT_CYCLES - 1)) begin
                        post_reset_wait_cnt_r <= 32'd0;
                        state_r <= ST_SCAN_ID1_REQ;
                    end else begin
                        post_reset_wait_cnt_r <= post_reset_wait_cnt_r + 32'd1;
                    end
                end

                ST_SCAN_ID1_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= scan_addr_r;
                    mdio_reg_addr_r <= REG_PHYID1;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_SCAN_ID1_WAIT;
                end

                ST_SCAN_ID1_WAIT: begin
                    if (mdio_done_w) begin
                        phy_id1_r <= mdio_rd_data_w;
                        state_r   <= ST_SCAN_ID2_REQ;
                    end
                end

                ST_SCAN_ID2_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= scan_addr_r;
                    mdio_reg_addr_r <= REG_PHYID2;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_SCAN_ID2_WAIT;
                end

                ST_SCAN_ID2_WAIT: begin
                    if (mdio_done_w) begin
                        if ((phy_id1_r != 16'h0000) && (phy_id1_r != 16'hFFFF) &&
                            (mdio_rd_data_w != 16'h0000) && (mdio_rd_data_w != 16'hFFFF)) begin
                            phy_id2_r        <= mdio_rd_data_w;
                            scan_done_r      <= 1'b1;
                            phy_present_r    <= 1'b1;
                            phy_addr_r       <= scan_addr_r;
                            init_busy_r      <= 1'b1;
                            init_done_r      <= 1'b0;
                            init_error_r     <= 1'b0;
                            last_error_r     <= ERR_NONE;
                            bmcr_reset_poll_cnt_r <= 8'd0;
                            state_r          <= ST_INIT_BMCR_RESET_REQ;
                        end else if (!USE_FIXED_PHY_ADDR && (scan_addr_r != 5'd31)) begin
                            scan_addr_r <= scan_addr_r + 5'd1;
                            state_r     <= ST_SCAN_ID1_REQ;
                        end else begin
                            scan_done_r      <= 1'b1;
                            phy_present_r    <= 1'b0;
                            phy_addr_r       <= 5'd0;
                            phy_id1_r        <= 16'd0;
                            phy_id2_r        <= 16'd0;
                            init_busy_r      <= 1'b0;
                            init_done_r      <= 1'b0;
                            init_error_r     <= 1'b1;
                            last_error_r     <= ERR_NO_PHY_FOUND;
                            poll_wait_cnt_r  <= 32'd0;
                            state_r          <= ST_POLL_WAIT;
                        end
                    end
                end

                ST_INIT_BMCR_RESET_REQ: begin
                    init_busy_r            <= 1'b1;
                    init_done_r            <= 1'b0;
                    init_error_r           <= 1'b0;
                    last_error_r           <= ERR_NONE;
                    phy_bmsr_r             <= 16'd0;
                    phy_anlpar_r           <= 16'd0;
                    phy_stat1000_r         <= 16'd0;
                    mdio_start_r           <= 1'b1;
                    mdio_op_wr_r           <= 1'b1;
                    mdio_phy_addr_r        <= phy_addr_r;
                    mdio_reg_addr_r        <= REG_BMCR;
                    mdio_wr_data_r         <= PHY_BMCR_RESET_INIT;
                    phy_bmcr_r             <= PHY_BMCR_RESET_INIT;
                    bmcr_reset_poll_cnt_r  <= 8'd0;
                    state_r                <= ST_INIT_BMCR_RESET_WAIT;
                end

                ST_INIT_BMCR_RESET_WAIT: begin
                    if (mdio_done_w) begin
                        state_r <= ST_INIT_BMCR_POLL_REQ;
                    end
                end

                ST_INIT_BMCR_POLL_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_BMCR;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_INIT_BMCR_POLL_WAIT;
                end

                ST_INIT_BMCR_POLL_WAIT: begin
                    if (mdio_done_w) begin
                        phy_bmcr_r <= mdio_rd_data_w;
                        if (((mdio_rd_data_w & BMCR_RESET_BIT) == 16'd0) ||
                            (bmcr_reset_poll_cnt_r == (BMCR_RESET_POLL_LIMIT - 1))) begin
                            if ((mdio_rd_data_w & BMCR_RESET_BIT) != 16'd0) begin
                                init_error_r <= 1'b1;
                                last_error_r <= ERR_BMCR_TIMEOUT;
                            end
                            state_r <= ST_INIT_ANAR_REQ;
                        end else begin
                            bmcr_reset_poll_cnt_r <= bmcr_reset_poll_cnt_r + 8'd1;
                            state_r <= ST_INIT_BMCR_POLL_REQ;
                        end
                    end
                end

                ST_INIT_ANAR_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b1;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_ANAR;
                    mdio_wr_data_r  <= PHY_ANAR_INIT;
                    phy_anar_r      <= PHY_ANAR_INIT;
                    state_r         <= ST_INIT_ANAR_WAIT;
                end

                ST_INIT_ANAR_WAIT: begin
                    if (mdio_done_w) begin
                        state_r <= ST_INIT_GCTRL_REQ;
                    end
                end

                ST_INIT_GCTRL_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b1;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_CTRL1000;
                    mdio_wr_data_r  <= PHY_CTRL1000_INIT;
                    phy_ctrl1000_r  <= PHY_CTRL1000_INIT;
                    state_r         <= ST_INIT_GCTRL_WAIT;
                end

                ST_INIT_GCTRL_WAIT: begin
                    if (mdio_done_w) begin
                        state_r <= ST_INIT_BMCR_AN_REQ;
                    end
                end

                ST_INIT_BMCR_AN_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b1;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_BMCR;
                    mdio_wr_data_r  <= PHY_BMCR_AN_INIT;
                    phy_bmcr_r      <= PHY_BMCR_AN_INIT;
                    state_r         <= ST_INIT_BMCR_AN_WAIT;
                end

                ST_INIT_BMCR_AN_WAIT: begin
                    if (mdio_done_w) begin
                        init_busy_r     <= 1'b0;
                        init_done_r     <= 1'b1;
                        poll_wait_cnt_r <= 32'd0;
                        state_r         <= ST_POLL_WAIT;
                    end
                end

                ST_CMD_RESTART_AN_REQ: begin
                    if (phy_present_r) begin
                        mdio_start_r    <= 1'b1;
                        mdio_op_wr_r    <= 1'b1;
                        mdio_phy_addr_r <= phy_addr_r;
                        mdio_reg_addr_r <= REG_BMCR;
                        mdio_wr_data_r  <= PHY_BMCR_AN_INIT;
                        phy_bmcr_r      <= PHY_BMCR_AN_INIT;
                        state_r         <= ST_CMD_RESTART_AN_WAIT;
                    end else begin
                        scan_done_r <= 1'b0;
                        if (USE_FIXED_PHY_ADDR)
                            scan_addr_r <= FIXED_PHY_ADDR;
                        else
                            scan_addr_r <= 5'd0;
                        state_r <= ST_SCAN_ID1_REQ;
                    end
                end

                ST_CMD_RESTART_AN_WAIT: begin
                    if (mdio_done_w) begin
                        poll_wait_cnt_r <= 32'd0;
                        state_r         <= ST_POLL_WAIT;
                    end
                end

                ST_POLL_WAIT: begin
                    if (pending_soft_reset_r) begin
                        if (phy_present_r) begin
                            state_r <= ST_INIT_BMCR_RESET_REQ;
                        end else begin
                            scan_done_r <= 1'b0;
                            if (USE_FIXED_PHY_ADDR)
                                scan_addr_r <= FIXED_PHY_ADDR;
                            else
                                scan_addr_r <= 5'd0;
                            state_r <= ST_SCAN_ID1_REQ;
                        end
                    end else if (pending_restart_an_r) begin
                        state_r <= ST_CMD_RESTART_AN_REQ;
                    end else if (poll_wait_cnt_r == (POLL_INTERVAL_CYCLES - 1)) begin
                        poll_wait_cnt_r <= 32'd0;
                        if (phy_present_r) begin
                            state_r <= ST_POLL_BMSR1_REQ;
                        end else begin
                            scan_done_r <= 1'b0;
                            if (USE_FIXED_PHY_ADDR)
                                scan_addr_r <= FIXED_PHY_ADDR;
                            else
                                scan_addr_r <= 5'd0;
                            state_r <= ST_SCAN_ID1_REQ;
                        end
                    end else begin
                        poll_wait_cnt_r <= poll_wait_cnt_r + 32'd1;
                    end
                end

                ST_POLL_BMSR1_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_BMSR;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_POLL_BMSR1_WAIT;
                end

                ST_POLL_BMSR1_WAIT: begin
                    if (mdio_done_w) begin
                        state_r <= ST_POLL_BMSR2_REQ;
                    end
                end

                ST_POLL_BMSR2_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_BMSR;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_POLL_BMSR2_WAIT;
                end

                ST_POLL_BMSR2_WAIT: begin
                    if (mdio_done_w) begin
                        phy_bmsr_r <= mdio_rd_data_w;
                        state_r    <= ST_POLL_ANLPAR_REQ;
                    end
                end

                ST_POLL_ANLPAR_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_ANLPAR;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_POLL_ANLPAR_WAIT;
                end

                ST_POLL_ANLPAR_WAIT: begin
                    if (mdio_done_w) begin
                        phy_anlpar_r <= mdio_rd_data_w;
                        state_r      <= ST_POLL_GSTAT_REQ;
                    end
                end

                ST_POLL_GSTAT_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_STAT1000;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_POLL_GSTAT_WAIT;
                end

                ST_POLL_GSTAT_WAIT: begin
                    if (mdio_done_w) begin
                        phy_stat1000_r <= mdio_rd_data_w;
                        state_r        <= ST_POLL_BMCR_REQ;
                    end
                end

                ST_POLL_BMCR_REQ: begin
                    mdio_start_r    <= 1'b1;
                    mdio_op_wr_r    <= 1'b0;
                    mdio_phy_addr_r <= phy_addr_r;
                    mdio_reg_addr_r <= REG_BMCR;
                    mdio_wr_data_r  <= 16'd0;
                    state_r         <= ST_POLL_BMCR_WAIT;
                end

                ST_POLL_BMCR_WAIT: begin
                    if (mdio_done_w) begin
                        phy_bmcr_r      <= mdio_rd_data_w;
                        poll_wait_cnt_r <= 32'd0;
                        state_r         <= ST_POLL_WAIT;
                    end
                end

                default: begin
                    state_r <= ST_RESET_HOLD;
                end
            endcase
        end
    end

    // keep rd_valid visible to synthesis; read data is consumed in-controller.
    wire _unused_keep;
    assign _unused_keep = mdio_rd_valid_w;

endmodule
