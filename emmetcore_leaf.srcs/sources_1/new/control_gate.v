`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/18/2024 11:48:02 AM
// Design Name: 
// Module Name: control_gate
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


module control_gate(
    input ctrl,
    input [63:0] data_in,
    input [1:0] header_in,
    input [6:0] sequence_in,
    output [63:0] data_out,
    output [1:0] header_out,
    output [6:0] sequence_out
    );
    
    assign data_out = ctrl ? data_in : 64'b0;
    assign header_out = ctrl ? header_in : 2'b0;
    assign sequence_out = ctrl ? sequence_in : 7'b0;

endmodule
