`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/12/2024 11:16:39 AM
// Design Name: 
// Module Name: inverter
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module inverter(
    input [1:0] rx_header_in,
    input [63:0] rx_data_in,
    output [1:0] rx_header_out,
    output [63:0] rx_data_out
    );
    
    assign rx_header_out = ~rx_header_in;
    assign rx_data_out = ~rx_data_in;
    
endmodule
