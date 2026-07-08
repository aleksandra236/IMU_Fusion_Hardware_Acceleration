## =============================================================================
## top_zybo.xdc
## Constraints za Zybo Z7-10 (xc7z010clg400-1)
## =============================================================================

## Clock: 125 MHz, pin K17
set_property PACKAGE_PIN K17 [get_ports clk_i]
set_property IOSTANDARD LVCMOS33 [get_ports clk_i]
create_clock -period 8.000 -name sys_clk_pin -waveform {0.000 4.000} [get_ports clk_i]

## Buttons
set_property PACKAGE_PIN K18 [get_ports {btn_i[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_i[0]}]
set_property PACKAGE_PIN P16 [get_ports {btn_i[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_i[1]}]
set_property PACKAGE_PIN V16 [get_ports {btn_i[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_i[2]}]
set_property PACKAGE_PIN Y16 [get_ports {btn_i[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {btn_i[3]}]

## LEDs
set_property PACKAGE_PIN M14 [get_ports {led_o[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_o[0]}]
set_property PACKAGE_PIN M15 [get_ports {led_o[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_o[1]}]
set_property PACKAGE_PIN G14 [get_ports {led_o[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_o[2]}]
set_property PACKAGE_PIN D18 [get_ports {led_o[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led_o[3]}]