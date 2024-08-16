`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/09/2024 03:19:16 PM
// Design Name: 
// Module Name: program_dummy
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


module program_dummy(
    input send,
    input clk,
    input ready,
    output reg [31:0] data,
    output reg valid
    );
    
    always @(posedge clk) begin
        if (send && ready) begin
            data <= data + 32'h1;
            valid <= 1;
        end else if (!send) begin
            valid <= 0;
            data <= 0;
        end
    end
    
endmodule
