# Implementation constraints for the verified req/ack and stable-hold CDC structure.
# Keep TX/sys clocks timed together. The PHY RX clock is independently recovered.
# First synchronizer stage only; second stages retain normal setup/hold checks.
set first_sync [get_cells -quiet -hier -regexp {.*(sync0_r_reg|sys0_r_reg|src_ack_sync_r_reg\[0\]|dst_req_sync_r_reg\[0\])$}]
set first_sync [filter $first_sync {ASYNC_REG == TRUE}]
set_false_path -to [get_pins -of_objects $first_sync -filter {REF_PIN_NAME == D}]

# The source hold bank stays stable until the request has crossed two flops
# and the destination has captured it. Bound each bus path to one RX period
# (8 ns); these paths remain subject to placement/routing timing optimization.
set held [get_cells -quiet -hier -regexp {.*(src_hold_bus_r_reg\[[0-9]+\]|tx_hold_(ns|seconds|nanoseconds)_r_reg\[[0-9]+\])$}]
set_max_delay 8.000 -datapath_only -from $held
