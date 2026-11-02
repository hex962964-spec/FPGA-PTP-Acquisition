`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_cfg_cmd_manager
// Description:
//   PTP 控制面管理器。
//
//   该模块把原先 net_acq_ptp_subsystem 中最容易变成“大杂烩”的 sys-domain
//   配置/命令/时间戳状态逻辑单独收口，职责固定为：
//     1) 接收 CSR 影子配置与 W1P 控制脉冲；
//     2) 维护 sys->eth_tx / sys->eth_rx 的 PTP 配置 CDC；
//     3) 维护 sys->eth_tx 的主动 TX 命令 CDC；
//     4) 维护 eth_tx->sys 的“命令已被 TX path 接收”ACK 脉冲 CDC；
//     5) 维护 eth_tx->sys 的 TX timestamp CDC 与 low->high 消费语义；
//     6) 汇总 sys 侧看到的 busy / pending / outstanding 状态。
//
// Design notes:
//   - 对外行为与旧版保持一致：
//       * 软件触发的主动 TX 仍固定收口到 Delay_Req + need_timestamp=1；
//       * TXSTAMP 的软件消费顺序仍为 low->high destructive read；
//       * tx_abort / clear_status / soft_reset 会清空本地 pending/outstanding。
//   - 本模块只处理“控制面”；真正的 PTP RX/TX 数据路径分别放到独立子模块。
//////////////////////////////////////////////////////////////////////////////////
module ptp_cfg_cmd_manager(
    input  wire         clk_in,
    input  wire         resetn,
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,
    input  wire         eth_rx_clk,
    input  wire         eth_rx_resetn,

    // ------------------------------------------------------------------------
    // sys-domain reset / clear pulses shared with the wrapper
    // ------------------------------------------------------------------------
    input  wire         cfg_soft_reset_pulse,
    input  wire         cfg_clear_status_pulse,

    // ------------------------------------------------------------------------
    // PTP shadow configuration / control from CSR (sys domain)
    // ------------------------------------------------------------------------
    input  wire         cfg_ptp_enable,
    input  wire         cfg_ptp_one_step,
    input  wire         cfg_ptp_two_step,
    input  wire         cfg_ptp_event_priority_enable,
    input  wire         cfg_ptp_hold,
    input  wire         cfg_ptp_strict_priority,
    input  wire [7:0]   cfg_ptp_domain,
    input  wire [63:0]  cfg_ptp_clock_id,
    input  wire [15:0]  cfg_ptp_port_num,
    input  wire         cfg_ptp_update_pulse,
    input  wire         cfg_ptp_set_time_pulse,
    input  wire         cfg_ptp_step_pulse,
    input  wire         cfg_ptp_tx_abort_pulse,
    input  wire         cfg_ptp_tx_trigger_pulse,
    input  wire [63:0]  cfg_ptp_set_time_ns,
    input  wire [31:0]  cfg_ptp_step_ns,
    input  wire [31:0]  cfg_ptp_freq_adj_ppb,
    input  wire [3:0]   cfg_ptp_tx_message_type,
    input  wire [3:0]   cfg_ptp_tx_transport_specific,
    input  wire [15:0]  cfg_ptp_tx_sequence_id,
    input  wire [15:0]  cfg_ptp_tx_flags,
    input  wire [7:0]   cfg_ptp_tx_log_message_interval,
    input  wire         cfg_ptp_tx_need_timestamp,

    // ------------------------------------------------------------------------
    // eth_tx sideband coming back from the TX datapath / board edge
    // ------------------------------------------------------------------------
    input  wire         ptp_tx_cmd_exec_ack_pulse_eth,
    input  wire         ptp_path_busy_eth,
    input  wire         ptp_tx_pending_any_eth,
    input  wire [63:0]  ptp_txstamp_ns,
    input  wire         ptp_txstamp_valid,

    // ------------------------------------------------------------------------
    // sys-domain CSR-visible timestamp / status
    // ------------------------------------------------------------------------
    output wire [63:0]  ptp_txstamp_ns_latched,
    output wire         ptp_txstamp_valid_latched,
    output wire         ptp_txstamp_overflow_sticky,
    input  wire         ptp_txstamp_consume_pulse_sys,
    output wire         ptp_tx_busy_status_sys,
    output wire         ptp_tx_cmd_pending_status_sys,

    // ------------------------------------------------------------------------
    // Decoded active PTP config for the eth_tx domain
    // ------------------------------------------------------------------------
    output wire         tx_cfg_enable_eth,
    output wire         tx_cfg_hold_eth,
    output wire         tx_cfg_strict_priority_eth,
    output wire         tx_cfg_event_priority_enable_eth,
    output wire [7:0]   tx_cfg_domain_eth,
    output wire [63:0]  tx_cfg_clock_id_eth,
    output wire [15:0]  tx_cfg_port_num_eth,
    output wire [63:0]  tx_cfg_set_time_ns_eth,
    output wire [31:0]  tx_cfg_step_ns_eth,
    output wire [31:0]  tx_cfg_freq_adj_ppb_eth,
    output wire         tx_cfg_set_time_pulse_eth,
    output wire         tx_cfg_step_pulse_eth,
    output wire         tx_cfg_update_pulse_eth,

    // ------------------------------------------------------------------------
    // Decoded active PTP config for the eth_rx domain (used by RX ToD fabric)
    // ------------------------------------------------------------------------
    output wire         rx_tod_enable_eth,
    output wire         rx_tod_hold_eth,
    output wire [63:0]  rx_tod_set_time_ns_eth,
    output wire [31:0]  rx_tod_step_ns_eth,
    output wire [31:0]  rx_tod_freq_adj_ppb_eth,
    output wire         rx_tod_set_time_pulse_eth,
    output wire         rx_tod_step_pulse_eth,
    output wire         rx_tod_update_pulse_eth,

    // ------------------------------------------------------------------------
    // Decoded TX command for the eth_tx PTP TX datapath
    // ------------------------------------------------------------------------
    output wire         tx_cmd_update_pulse_eth,
    output wire         tx_cmd_need_timestamp_eth,
    output wire [7:0]   tx_cmd_log_message_interval_eth,
    output wire [15:0]  tx_cmd_flags_eth,
    output wire [15:0]  tx_cmd_sequence_id_eth,
    output wire [3:0]   tx_cmd_transport_specific_eth,
    output wire [3:0]   tx_cmd_message_type_eth,
    output wire [15:0]  tx_cmd_port_num_eth,
    output wire [63:0]  tx_cmd_clock_id_eth,
    output wire [7:0]   tx_cmd_domain_eth,

    // ------------------------------------------------------------------------
    // Back to sibling subsystems / wrapper
    // ------------------------------------------------------------------------
    output wire         ptp_cfg_enable_eth,
    output wire         ptp_cfg_strict_priority_eth,
    output wire         ptp_cfg_event_priority_enable_eth,
    output wire         ptp_txstamp_src_ready_eth
);

    localparam integer C_PTP_CFG_W          = 224;
    localparam integer C_PTP_TX_CMD_W       = 137;
    localparam [3:0]   C_PTP_MSG_DELAY_REQ  = 4'h1;

    // ------------------------------------------------------------------------
    // sys -> eth{tx,rx} config transport state
    // ------------------------------------------------------------------------
    reg  [C_PTP_CFG_W-1:0]    ptp_cfg_bus_sys_r;
    reg                       ptp_cfg_req_toggle_r;
    reg                       ptp_cfg_ack_toggle_seen_r;
    reg                       ptp_cfg_pending_r;
    reg                       ptp_cfg_apply_pending_r;
    reg                       ptp_set_time_pending_r;
    reg                       ptp_step_pending_r;

    reg                       ptp_cfg_rx_req_toggle_r;
    reg                       ptp_cfg_rx_ack_toggle_seen_r;
    reg                       ptp_cfg_rx_pending_r;

    wire                      ptp_cfg_ack_toggle_sys_w;
    wire [C_PTP_CFG_W-1:0]    ptp_cfg_bus_eth_w;
    wire                      ptp_cfg_update_pulse_eth_w;
    wire                      ptp_cfg_rx_ack_toggle_sys_w;
    wire [C_PTP_CFG_W-1:0]    ptp_cfg_bus_rx_w;
    wire                      ptp_cfg_update_pulse_rx_w;

    // ------------------------------------------------------------------------
    // sys -> eth_tx TX command transport state
    // ------------------------------------------------------------------------
    reg                       ptp_tx_trigger_pending_r;
    reg  [C_PTP_TX_CMD_W-1:0] ptp_tx_cmd_bus_sys_r;
    reg                       ptp_tx_cmd_req_toggle_r;
    reg                       ptp_tx_cmd_ack_toggle_seen_r;
    reg                       ptp_tx_cmd_cdc_pending_sys_r;
    reg                       ptp_tx_cmd_wait_ack_sys_r;
    reg                       ptp_tx_cmd_need_timestamp_sys_r;
    reg                       ptp_tx_outstanding_sys_r;

    wire                      ptp_tx_cmd_ack_toggle_sys_w;
    wire [C_PTP_TX_CMD_W-1:0] ptp_tx_cmd_bus_eth_w;
    wire                      ptp_tx_cmd_update_pulse_eth_w;
    wire                      ptp_tx_cmd_exec_ack_pulse_sys_w;

    // ------------------------------------------------------------------------
    // TX timestamp CDC + sys-domain latched readback semantics
    // ------------------------------------------------------------------------
    reg  [63:0] ptp_txstamp_ns_latched_r;
    reg         ptp_txstamp_valid_latched_r;
    wire        ptp_txstamp_cdc_src_ready_w;
    wire        ptp_txstamp_cdc_dst_valid_w;
    wire [63:0] ptp_txstamp_cdc_dst_ns_w;
    wire        ptp_txstamp_cdc_dst_overflow_sticky_w;
    wire        ptp_txstamp_cdc_src_overflow_sticky_w;
    wire        ptp_txstamp_cdc_dst_pop_w;
    wire        ptp_txstamp_take_local_w;
    wire        ptp_txstamp_latched_clear_sys_w;
    wire        ptp_txstamp_overflow_sticky_w;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] ptp_txstamp_overflow_src_sync_sys_r;

    // ------------------------------------------------------------------------
    // Small sys-domain synchronizers for eth_tx status visibility
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] ptp_tx_busy_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] ptp_tx_cmd_pending_sys_r;

    // ------------------------------------------------------------------------
    // Decoded eth_tx / eth_rx config fields
    // ------------------------------------------------------------------------
    wire                      tx_cfg_one_step_eth_w;
    wire                      tx_cfg_two_step_eth_w;
    wire                      rx_cfg_one_step_eth_w;
    wire                      rx_cfg_two_step_eth_w;
    wire                      rx_cfg_event_priority_enable_eth_w;
    wire                      rx_cfg_strict_priority_eth_w;
    wire [7:0]                rx_cfg_domain_eth_w;
    wire [63:0]               rx_cfg_clock_id_eth_w;
    wire [15:0]               rx_cfg_port_num_eth_w;

    wire                      tx_cmd_need_timestamp_eth_w;
    wire [7:0]                tx_cmd_log_message_interval_eth_w;
    wire [15:0]               tx_cmd_flags_eth_w;
    wire [15:0]               tx_cmd_sequence_id_eth_w;
    wire [3:0]                tx_cmd_transport_specific_eth_w;
    wire [3:0]                tx_cmd_message_type_eth_w;
    wire [15:0]               tx_cmd_port_num_eth_w;
    wire [63:0]               tx_cmd_clock_id_eth_w;
    wire [7:0]                tx_cmd_domain_eth_w;

    wire                      ptp_cfg_launch_w;
    wire                      ptp_tx_cmd_launch_sys_w;
    wire [C_PTP_CFG_W-1:0]    ptp_cfg_bus_pack_sys_w;
    wire [C_PTP_TX_CMD_W-1:0] ptp_tx_cmd_bus_pack_sys_w;
    wire                      ptp_set_time_new_pulse_w;
    wire                      ptp_step_new_pulse_w;
    wire                      ptp_tx_trigger_new_pulse_w;
    wire                      unused_keep_w;

    assign ptp_txstamp_ns_latched      = ptp_txstamp_ns_latched_r;
    assign ptp_txstamp_valid_latched   = ptp_txstamp_valid_latched_r;
    assign ptp_txstamp_overflow_sticky = ptp_txstamp_overflow_sticky_w;
    assign ptp_txstamp_src_ready_eth   = ptp_txstamp_cdc_src_ready_w;

    assign ptp_txstamp_latched_clear_sys_w =
        ptp_txstamp_consume_pulse_sys || cfg_clear_status_pulse;
    assign ptp_set_time_new_pulse_w =
        cfg_ptp_update_pulse && cfg_ptp_set_time_pulse;
    assign ptp_step_new_pulse_w =
        cfg_ptp_update_pulse && cfg_ptp_step_pulse;
    assign ptp_tx_trigger_new_pulse_w =
        cfg_ptp_update_pulse && cfg_ptp_tx_trigger_pulse;

    assign ptp_cfg_launch_w =
        ptp_cfg_apply_pending_r && !ptp_cfg_pending_r && !ptp_cfg_rx_pending_r;
    assign ptp_tx_cmd_launch_sys_w =
        ptp_tx_trigger_pending_r &&
        cfg_ptp_enable &&
        !ptp_tx_cmd_cdc_pending_sys_r &&
        !ptp_tx_cmd_wait_ack_sys_r &&
        !ptp_tx_outstanding_sys_r;

    assign ptp_cfg_bus_pack_sys_w = {
        ptp_step_pending_r,
        ptp_set_time_pending_r,
        cfg_ptp_freq_adj_ppb,
        cfg_ptp_step_ns,
        cfg_ptp_set_time_ns,
        cfg_ptp_port_num,
        cfg_ptp_clock_id,
        cfg_ptp_domain,
        cfg_ptp_strict_priority,
        cfg_ptp_hold,
        cfg_ptp_event_priority_enable,
        cfg_ptp_two_step,
        cfg_ptp_one_step,
        cfg_ptp_enable
    };

    // 当前比赛版主动 TX 固定收口到 Delay_Req + need_timestamp=1。
    assign ptp_tx_cmd_bus_pack_sys_w = {
        1'b1,
        cfg_ptp_tx_log_message_interval,
        cfg_ptp_tx_flags,
        cfg_ptp_tx_sequence_id,
        cfg_ptp_tx_transport_specific,
        C_PTP_MSG_DELAY_REQ,
        cfg_ptp_port_num,
        cfg_ptp_clock_id,
        cfg_ptp_domain
    };

    assign {
        rx_tod_step_pulse_eth,
        rx_tod_set_time_pulse_eth,
        rx_tod_freq_adj_ppb_eth,
        rx_tod_step_ns_eth,
        rx_tod_set_time_ns_eth,
        rx_cfg_port_num_eth_w,
        rx_cfg_clock_id_eth_w,
        rx_cfg_domain_eth_w,
        rx_cfg_strict_priority_eth_w,
        rx_tod_hold_eth,
        rx_cfg_event_priority_enable_eth_w,
        rx_cfg_two_step_eth_w,
        rx_cfg_one_step_eth_w,
        rx_tod_enable_eth
    } = ptp_cfg_bus_rx_w;

    assign {
        tx_cfg_step_pulse_eth,
        tx_cfg_set_time_pulse_eth,
        tx_cfg_freq_adj_ppb_eth,
        tx_cfg_step_ns_eth,
        tx_cfg_set_time_ns_eth,
        tx_cfg_port_num_eth,
        tx_cfg_clock_id_eth,
        tx_cfg_domain_eth,
        tx_cfg_strict_priority_eth,
        tx_cfg_hold_eth,
        tx_cfg_event_priority_enable_eth,
        tx_cfg_two_step_eth_w,
        tx_cfg_one_step_eth_w,
        tx_cfg_enable_eth
    } = ptp_cfg_bus_eth_w;

    assign {
        tx_cmd_need_timestamp_eth_w,
        tx_cmd_log_message_interval_eth_w,
        tx_cmd_flags_eth_w,
        tx_cmd_sequence_id_eth_w,
        tx_cmd_transport_specific_eth_w,
        tx_cmd_message_type_eth_w,
        tx_cmd_port_num_eth_w,
        tx_cmd_clock_id_eth_w,
        tx_cmd_domain_eth_w
    } = ptp_tx_cmd_bus_eth_w;

    assign tx_cfg_update_pulse_eth            = ptp_cfg_update_pulse_eth_w;
    assign rx_tod_update_pulse_eth            = ptp_cfg_update_pulse_rx_w;
    assign tx_cmd_update_pulse_eth            = ptp_tx_cmd_update_pulse_eth_w;
    assign tx_cmd_need_timestamp_eth          = tx_cmd_need_timestamp_eth_w;
    assign tx_cmd_log_message_interval_eth    = tx_cmd_log_message_interval_eth_w;
    assign tx_cmd_flags_eth                   = tx_cmd_flags_eth_w;
    assign tx_cmd_sequence_id_eth             = tx_cmd_sequence_id_eth_w;
    assign tx_cmd_transport_specific_eth      = tx_cmd_transport_specific_eth_w;
    assign tx_cmd_message_type_eth            = tx_cmd_message_type_eth_w;
    assign tx_cmd_port_num_eth                = tx_cmd_port_num_eth_w;
    assign tx_cmd_clock_id_eth                = tx_cmd_clock_id_eth_w;
    assign tx_cmd_domain_eth                  = tx_cmd_domain_eth_w;

    assign ptp_cfg_enable_eth                 = tx_cfg_enable_eth;
    assign ptp_cfg_strict_priority_eth        = tx_cfg_strict_priority_eth;
    assign ptp_cfg_event_priority_enable_eth  = tx_cfg_event_priority_enable_eth;

    assign ptp_tx_busy_status_sys =
        ptp_tx_busy_sys_r[1] |
        ptp_tx_trigger_pending_r |
        ptp_tx_cmd_wait_ack_sys_r |
        ptp_tx_outstanding_sys_r;

    assign ptp_tx_cmd_pending_status_sys =
        ptp_tx_cmd_pending_sys_r[1] |
        ptp_tx_trigger_pending_r |
        ptp_tx_cmd_cdc_pending_sys_r |
        ptp_tx_cmd_wait_ack_sys_r |
        ptp_tx_outstanding_sys_r;

    // ------------------------------------------------------------------------
    // sys-domain config apply / dual-domain launch FSM
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            ptp_cfg_bus_sys_r            <= {C_PTP_CFG_W{1'b0}};
            ptp_cfg_req_toggle_r         <= 1'b0;
            ptp_cfg_ack_toggle_seen_r    <= 1'b0;
            ptp_cfg_pending_r            <= 1'b0;
            ptp_cfg_apply_pending_r      <= 1'b0;
            ptp_set_time_pending_r       <= 1'b0;
            ptp_step_pending_r           <= 1'b0;
            ptp_cfg_rx_req_toggle_r      <= 1'b0;
            ptp_cfg_rx_ack_toggle_seen_r <= 1'b0;
            ptp_cfg_rx_pending_r         <= 1'b0;
        end else begin
            if (ptp_cfg_ack_toggle_sys_w != ptp_cfg_ack_toggle_seen_r) begin
                ptp_cfg_ack_toggle_seen_r <= ptp_cfg_ack_toggle_sys_w;
                ptp_cfg_pending_r         <= 1'b0;
            end
            if (ptp_cfg_rx_ack_toggle_sys_w != ptp_cfg_rx_ack_toggle_seen_r) begin
                ptp_cfg_rx_ack_toggle_seen_r <= ptp_cfg_rx_ack_toggle_sys_w;
                ptp_cfg_rx_pending_r         <= 1'b0;
            end

            if (cfg_soft_reset_pulse || cfg_clear_status_pulse) begin
                ptp_cfg_apply_pending_r <= 1'b0;
                ptp_set_time_pending_r  <= 1'b0;
                ptp_step_pending_r      <= 1'b0;
            end else begin
                if (cfg_ptp_update_pulse) begin
                    ptp_cfg_apply_pending_r <= 1'b1;
                    if (cfg_ptp_set_time_pulse)
                        ptp_set_time_pending_r <= 1'b1;
                    if (cfg_ptp_step_pulse)
                        ptp_step_pending_r <= 1'b1;
                end

                if (ptp_cfg_launch_w) begin
                    ptp_cfg_bus_sys_r         <= ptp_cfg_bus_pack_sys_w;
                    ptp_cfg_req_toggle_r      <= ~ptp_cfg_req_toggle_r;
                    ptp_cfg_pending_r         <= 1'b1;
                    ptp_cfg_rx_req_toggle_r   <= ~ptp_cfg_rx_req_toggle_r;
                    ptp_cfg_rx_pending_r      <= 1'b1;
                    ptp_cfg_apply_pending_r   <= cfg_ptp_update_pulse;
                    ptp_set_time_pending_r    <= ptp_set_time_new_pulse_w;
                    ptp_step_pending_r        <= ptp_step_new_pulse_w;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // sys-domain TX command launch FSM
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            ptp_tx_trigger_pending_r        <= 1'b0;
            ptp_tx_cmd_bus_sys_r            <= {C_PTP_TX_CMD_W{1'b0}};
            ptp_tx_cmd_req_toggle_r         <= 1'b0;
            ptp_tx_cmd_ack_toggle_seen_r    <= 1'b0;
            ptp_tx_cmd_cdc_pending_sys_r    <= 1'b0;
            ptp_tx_cmd_wait_ack_sys_r       <= 1'b0;
            ptp_tx_cmd_need_timestamp_sys_r <= 1'b0;
        end else begin
            if (ptp_tx_cmd_ack_toggle_sys_w != ptp_tx_cmd_ack_toggle_seen_r) begin
                ptp_tx_cmd_ack_toggle_seen_r <= ptp_tx_cmd_ack_toggle_sys_w;
                ptp_tx_cmd_cdc_pending_sys_r <= 1'b0;
            end

            if (cfg_soft_reset_pulse || cfg_clear_status_pulse ||
                cfg_ptp_tx_abort_pulse || !cfg_ptp_enable) begin
                ptp_tx_trigger_pending_r        <= 1'b0;
                ptp_tx_cmd_cdc_pending_sys_r    <= 1'b0;
                ptp_tx_cmd_wait_ack_sys_r       <= 1'b0;
                ptp_tx_cmd_need_timestamp_sys_r <= 1'b0;
            end else begin
                if (ptp_tx_trigger_new_pulse_w && !ptp_tx_cmd_wait_ack_sys_r) begin
                    ptp_tx_cmd_bus_sys_r     <= ptp_tx_cmd_bus_pack_sys_w;
                    ptp_tx_trigger_pending_r <= 1'b1;
                end

                if (ptp_tx_cmd_launch_sys_w) begin
                    ptp_tx_cmd_req_toggle_r         <= ~ptp_tx_cmd_req_toggle_r;
                    ptp_tx_trigger_pending_r        <= 1'b0;
                    ptp_tx_cmd_cdc_pending_sys_r    <= 1'b1;
                    ptp_tx_cmd_wait_ack_sys_r       <= 1'b1;
                    ptp_tx_cmd_need_timestamp_sys_r <= ptp_tx_cmd_bus_sys_r[C_PTP_TX_CMD_W-1];
                end

                if (ptp_tx_cmd_exec_ack_pulse_sys_w && ptp_tx_cmd_wait_ack_sys_r) begin
                    ptp_tx_cmd_wait_ack_sys_r       <= 1'b0;
                    ptp_tx_cmd_need_timestamp_sys_r <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // TX timestamp CDC + sys destructive-read state
    // ------------------------------------------------------------------------
    assign ptp_txstamp_take_local_w      =
        ptp_txstamp_cdc_dst_valid_w && !ptp_txstamp_valid_latched_r;
    assign ptp_txstamp_cdc_dst_pop_w     = ptp_txstamp_take_local_w;
    assign ptp_txstamp_overflow_sticky_w =
        ptp_txstamp_cdc_dst_overflow_sticky_w |
        ptp_txstamp_overflow_src_sync_sys_r[1];

    ptp_txstamp_cdc_bridge #(
        .P_TS_W  (64),
        .P_TAG_W (16)
    ) u_ptp_txstamp_cdc_bridge (
        .src_clk             (eth_tx_clk),
        .src_resetn          (eth_tx_resetn),
        .src_clear_status    (eth_clear_status_pulse_eth | eth_soft_reset_pulse_eth),
        .src_ts_valid        (ptp_txstamp_valid),
        .src_ts_ns           (ptp_txstamp_ns),
        .src_ts_tag          (16'd0),
        .src_ts_ready        (ptp_txstamp_cdc_src_ready_w),
        .src_overflow_sticky (ptp_txstamp_cdc_src_overflow_sticky_w),
        .dst_clk             (clk_in),
        .dst_resetn          (resetn),
        .dst_clear_status    (cfg_clear_status_pulse | cfg_soft_reset_pulse),
        .dst_ts_valid        (ptp_txstamp_cdc_dst_valid_w),
        .dst_ts_ns           (ptp_txstamp_cdc_dst_ns_w),
        .dst_ts_tag          (),
        .dst_ts_pulse        (),
        .dst_ts_pop          (ptp_txstamp_cdc_dst_pop_w),
        .dst_overflow_sticky (ptp_txstamp_cdc_dst_overflow_sticky_w)
    );

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            ptp_txstamp_ns_latched_r            <= 64'd0;
            ptp_txstamp_valid_latched_r         <= 1'b0;
            ptp_txstamp_overflow_src_sync_sys_r <= 2'b00;
            ptp_tx_outstanding_sys_r            <= 1'b0;
        end else begin
            ptp_txstamp_overflow_src_sync_sys_r <= {
                ptp_txstamp_overflow_src_sync_sys_r[0],
                ptp_txstamp_cdc_src_overflow_sticky_w
            };

            if (cfg_soft_reset_pulse || cfg_clear_status_pulse || cfg_ptp_tx_abort_pulse) begin
                ptp_txstamp_ns_latched_r          <= 64'd0;
                ptp_txstamp_valid_latched_r       <= 1'b0;
                ptp_tx_outstanding_sys_r          <= 1'b0;
            end else begin
                if (ptp_txstamp_latched_clear_sys_w) begin
                    ptp_txstamp_valid_latched_r <= 1'b0;
                    ptp_tx_outstanding_sys_r    <= 1'b0;
                end

                if (ptp_txstamp_take_local_w) begin
                    ptp_txstamp_ns_latched_r    <= ptp_txstamp_cdc_dst_ns_w;
                    ptp_txstamp_valid_latched_r <= 1'b1;
                end

                if (!cfg_ptp_enable) begin
                    ptp_tx_outstanding_sys_r <= 1'b0;
                end else if (ptp_tx_cmd_exec_ack_pulse_sys_w && ptp_tx_cmd_wait_ack_sys_r &&
                             ptp_tx_cmd_need_timestamp_sys_r) begin
                    ptp_tx_outstanding_sys_r <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Multi-bit CDC bridges
    // ------------------------------------------------------------------------
    net_acq_cfg_cdc_bridge #(
        .P_CFG_W   (C_PTP_CFG_W),
        .P_INIT_CFG({C_PTP_CFG_W{1'b0}})
    ) u_ptp_cfg_cdc_bridge (
        .sys_clk              (clk_in),
        .sys_resetn           (resetn),
        .sys_cfg_bus          (ptp_cfg_bus_sys_r),
        .sys_req_toggle       (ptp_cfg_req_toggle_r),
        .sys_ack_toggle       (ptp_cfg_ack_toggle_sys_w),
        .eth_clk              (eth_tx_clk),
        .eth_resetn           (eth_tx_resetn),
        .eth_cfg_bus          (ptp_cfg_bus_eth_w),
        .eth_cfg_update_pulse (ptp_cfg_update_pulse_eth_w)
    );

    net_acq_cfg_cdc_bridge #(
        .P_CFG_W   (C_PTP_CFG_W),
        .P_INIT_CFG({C_PTP_CFG_W{1'b0}})
    ) u_ptp_rx_cfg_cdc_bridge (
        .sys_clk              (clk_in),
        .sys_resetn           (resetn),
        .sys_cfg_bus          (ptp_cfg_bus_sys_r),
        .sys_req_toggle       (ptp_cfg_rx_req_toggle_r),
        .sys_ack_toggle       (ptp_cfg_rx_ack_toggle_sys_w),
        .eth_clk              (eth_rx_clk),
        .eth_resetn           (eth_rx_resetn),
        .eth_cfg_bus          (ptp_cfg_bus_rx_w),
        .eth_cfg_update_pulse (ptp_cfg_update_pulse_rx_w)
    );

    net_acq_cfg_cdc_bridge #(
        .P_CFG_W   (C_PTP_TX_CMD_W),
        .P_INIT_CFG({C_PTP_TX_CMD_W{1'b0}})
    ) u_ptp_tx_cmd_cdc_bridge (
        .sys_clk              (clk_in),
        .sys_resetn           (resetn),
        .sys_cfg_bus          (ptp_tx_cmd_bus_sys_r),
        .sys_req_toggle       (ptp_tx_cmd_req_toggle_r),
        .sys_ack_toggle       (ptp_tx_cmd_ack_toggle_sys_w),
        .eth_clk              (eth_tx_clk),
        .eth_resetn           (eth_tx_resetn),
        .eth_cfg_bus          (ptp_tx_cmd_bus_eth_w),
        .eth_cfg_update_pulse (ptp_tx_cmd_update_pulse_eth_w)
    );

    net_acq_pulse_cdc u_ptp_tx_cmd_exec_ack_cdc (
        .src_clk    (eth_tx_clk),
        .src_resetn (eth_tx_resetn),
        .src_pulse  (ptp_tx_cmd_exec_ack_pulse_eth),
        .dst_clk    (clk_in),
        .dst_resetn (resetn),
        .dst_pulse  (ptp_tx_cmd_exec_ack_pulse_sys_w)
    );

    // ------------------------------------------------------------------------
    // Small status synchronizers from eth_tx to sys
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            ptp_tx_busy_sys_r        <= 2'b00;
            ptp_tx_cmd_pending_sys_r <= 2'b00;
        end else begin
            ptp_tx_busy_sys_r        <= {ptp_tx_busy_sys_r[0], ptp_path_busy_eth};
            ptp_tx_cmd_pending_sys_r <= {ptp_tx_cmd_pending_sys_r[0], ptp_tx_pending_any_eth};
        end
    end

    // ------------------------------------------------------------------------
    // Local keepers for reserved/currently-planned compatibility bits.
    // ------------------------------------------------------------------------
    assign unused_keep_w = cfg_ptp_tx_message_type[0] |
                           cfg_ptp_tx_need_timestamp |
                           tx_cfg_one_step_eth_w |
                           tx_cfg_two_step_eth_w |
                           rx_cfg_one_step_eth_w |
                           rx_cfg_two_step_eth_w |
                           rx_cfg_event_priority_enable_eth_w |
                           rx_cfg_strict_priority_eth_w |
                           rx_cfg_domain_eth_w[0] |
                           rx_cfg_clock_id_eth_w[0] |
                           rx_cfg_port_num_eth_w[0] |
                           tx_cmd_message_type_eth_w[0];

endmodule
