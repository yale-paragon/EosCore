`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/24/2024 10:10:41 AM
// Design Name: 
// Module Name: selector
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


module selector(
    input [63:0] data_in,
    input valid_in,
    input [63:0] data_loop,
    input valid_loop,
    input loopback,
    
    output wire [63:0] data_selected,
    output wire valid_selected
    );
    
    assign data_selected = loopback ? data_loop : data_in;
    assign valid_selected = loopback ? valid_loop : valid_in;
    
endmodule
