# Board pin mapping recovered from the supplied la32r_soc.zip.
# Current board top uses active-low reset_n and internal BRAM.
set_property PACKAGE_PIN W17 [get_ports UART_RX]
set_property PACKAGE_PIN V17 [get_ports UART_TX]
set_property PACKAGE_PIN W19 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports UART_RX]
set_property IOSTANDARD LVCMOS33 [get_ports UART_TX]
set_property DRIVE 12 [get_ports UART_RX]
set_property DRIVE 12 [get_ports UART_TX]
set_property SLEW FAST [get_ports UART_RX]
set_property SLEW FAST [get_ports UART_TX]

set_property PACKAGE_PIN A20 [get_ports {rgmii_rxd[0]}]
set_property PACKAGE_PIN B21 [get_ports {rgmii_rxd[1]}]
set_property PACKAGE_PIN A21 [get_ports {rgmii_rxd[2]}]
set_property PACKAGE_PIN E21 [get_ports {rgmii_rxd[3]}]
set_property PACKAGE_PIN D21 [get_ports {rgmii_txd[0]}]
set_property PACKAGE_PIN G21 [get_ports {rgmii_txd[1]}]
set_property PACKAGE_PIN G22 [get_ports {rgmii_txd[2]}]
set_property PACKAGE_PIN G15 [get_ports {rgmii_txd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_rxd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {rgmii_txd[0]}]
set_property PACKAGE_PIN C18 [get_ports rgmii_rxc]
set_property PACKAGE_PIN B20 [get_ports rgmii_rx_ctl]
set_property PACKAGE_PIN F20 [get_ports rgmii_tx_ctl]
set_property PACKAGE_PIN C19 [get_ports rgmii_txc]
set_property PACKAGE_PIN J14 [get_ports phy_mdc]
set_property PACKAGE_PIN G16 [get_ports phy_mdio]
set_property PACKAGE_PIN H14 [get_ports phy_rst_n]
set_property DRIVE 12 [get_ports phy_mdc]
set_property DRIVE 12 [get_ports phy_mdio]
set_property DRIVE 12 [get_ports phy_rst_n]
set_property SLEW FAST [get_ports phy_mdc]
set_property SLEW FAST [get_ports phy_mdio]
set_property SLEW FAST [get_ports phy_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports phy_mdc]
set_property IOSTANDARD LVCMOS33 [get_ports phy_mdio]
set_property IOSTANDARD LVCMOS33 [get_ports phy_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_rx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_rxc]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_tx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports rgmii_txc]
set_property DRIVE 12 [get_ports rgmii_tx_ctl]
set_property DRIVE 12 [get_ports rgmii_txc]
set_property SLEW FAST [get_ports rgmii_tx_ctl]
set_property SLEW FAST [get_ports rgmii_txc]
set_property SLEW FAST [get_ports {rgmii_txd[3]}]
set_property SLEW FAST [get_ports {rgmii_txd[2]}]
set_property SLEW FAST [get_ports {rgmii_txd[1]}]
set_property SLEW FAST [get_ports {rgmii_txd[0]}]

set_property PACKAGE_PIN Y19 [get_ports reset_n]
set_property IOSTANDARD LVCMOS33 [get_ports reset_n]

# Clocking Wizard constrains the 50 MHz board input and 125 MHz PHY RX input
# and derives all PLL/MMCM output clocks. Duplicating them here overrides IP clocks.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
# RGMII external delay numbers require the PHY model and PCB skew budget.
# Keep internal clock paths timed; do not hide them with blanket false paths.

# Wildfire MINI V1.1 common-anode display, schematic CN1/CN2 -> U18.
# Segment order: A B C D E F G DP. Both I/O banks use 3.3 V.
set_property PACKAGE_PIN U22 [get_ports {seg_cathode[0]}]; # B14_L3_P / A
set_property PACKAGE_PIN P19 [get_ports {seg_cathode[1]}]; # B14_L5_P / B
set_property PACKAGE_PIN W21 [get_ports {seg_cathode[2]}]; # B14_L7_P / C
set_property PACKAGE_PIN V22 [get_ports {seg_cathode[3]}]; # B14_L3_N / D
set_property PACKAGE_PIN AB20 [get_ports {seg_cathode[4]}]; # B14_L15_N / E
set_property PACKAGE_PIN W22 [get_ports {seg_cathode[5]}]; # B14_L7_N / F
set_property PACKAGE_PIN AA20 [get_ports {seg_cathode[6]}]; # B14_L8_P / G
set_property PACKAGE_PIN AA21 [get_ports {seg_cathode[7]}]; # B14_L8_N / DP
set_property PACKAGE_PIN K18 [get_ports {seg_anode[0]}]; # B15_L13_P / DIG1
set_property PACKAGE_PIN K16 [get_ports {seg_anode[1]}]; # B15_L23_N / DIG2
set_property PACKAGE_PIN L16 [get_ports {seg_anode[2]}]; # B15_L23_P / DIG3
set_property PACKAGE_PIN G20 [get_ports {seg_anode[3]}]; # B15_L8_N / DIG4
set_property PACKAGE_PIN H20 [get_ports {seg_anode[4]}]; # B15_L8_P / DIG5
set_property PACKAGE_PIN H18 [get_ports {seg_anode[5]}]; # B15_L6_N / DIG6
set_property IOSTANDARD LVCMOS33 [get_ports {seg_cathode[*] seg_anode[*]}]
set_property DRIVE 4 [get_ports {seg_cathode[*] seg_anode[*]}]
set_property SLEW SLOW [get_ports {seg_cathode[*] seg_anode[*]}]
