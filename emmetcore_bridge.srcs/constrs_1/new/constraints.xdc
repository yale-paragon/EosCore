set_property LOC GTY_QUAD_X1Y1 [get_cells design_1_i/gt_quad_base_0/inst/quad_inst]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

create_clock -period 10.000 [get_ports {CLK_IN_D_0_clk_p[0]}]
create_clock -period 5.000 [get_ports CLK_IN1_D_0_clk_p]
create_clock -period 10.000 [get_ports CLK_IN1_D_1_clk_p]

set_property PACKAGE_PIN AB11 [get_ports {CLK_IN_D_0_clk_p[0]}]
set_property PACKAGE_PIN AW27 [get_ports CLK_IN1_D_0_clk_p]
set_property PACKAGE_PIN AE42 [get_ports CLK_IN1_D_1_clk_p]

set_property IOSTANDARD LVDS15 [get_ports CLK_IN1_D_0_clk_p]
set_property IOSTANDARD LVDS15 [get_ports CLK_IN1_D_0_clk_n]
set_property IOSTANDARD LVDS15 [get_ports CLK_IN1_D_1_clk_p]
set_property IOSTANDARD LVDS15 [get_ports CLK_IN1_D_1_clk_n]


set_false_path -from [get_pins design_1_i/emmetcore_0/inst/resetstate_n_initclk_reg/C] -to [get_pins design_1_i/emmetcore_0/inst/resetstate_n_intermediate_reg/D]
