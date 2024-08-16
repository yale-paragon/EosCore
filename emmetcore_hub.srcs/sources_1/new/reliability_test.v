`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/25/2024 11:24:05 AM
// Design Name: 
// Module Name: reliability_test
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


module reliability_test(
    input clk,
    input [63:0] data,
    input valid,
    output wire rd_enable,
    output reg unreliable_error,
    output wire [63:0] received_data,
    output reg [63:0] error_count,
    output reg end_of_test
    );
    
    reg init;
    reg [63:0] prevdata;
    assign rd_enable = 1;
    assign received_data = data;
    
    always @(posedge clk) begin
        if (valid) begin
            if (!init) begin 
                init <= 1;
                error_count <= 64'b0;
            end
            prevdata <= data;
            if (data == 64'b0) begin
                end_of_test <= 1;
                error_count <= 64'b0;
            end else begin
                 end_of_test <= 0;
                 if (init && data != prevdata + 1) begin 
                    unreliable_error <= 1;
                    error_count <= error_count + 1;
                end else unreliable_error <= 0;
            end
        end
        else if (!init) begin
            unreliable_error <= 0;
            end_of_test <= 0;
        end
    end
    
endmodule
