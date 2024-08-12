`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
//
// Company: Yale Efficient Computing Lab
// Engineer: Emmet Houghton
//
// Create Date: 07/02/2024 12:48:57 PM
// Design Name: 64B/66B Low-latency transceiver core
// Module Name: emmetcore_32b
// Target Devices: Programmed on xcvm1802-vsva2197-2MP-e-S (VMK180 Evaluation Board)
// Tool Versions: V1.0
// Description: Low-latency transceiver module for 64B/66B encoding with data width of 32 bits.
//
//////////////////////////////////////////////////////////////////////////////////

module emmetcore_32b(
    // Reset signal (active low)
    input tx_corereset_n,
    // Separate (slower) clock for initialization
    input init_clk,
    
    // Transceiver primitives
    input tx_userclk,
    input rx_userclk,
    input gt_powergood,
    input tx_resetdone,
    input rx_resetdone,

    // TX Handshake
    input [31:0] tx_data,
    input tx_valid,
    output reg tx_ready,
    
    // TX Lane
    output reg [63:0] tx_userdata_out,
    output reg [1:0] tx_header_out,
    output reg [6:0] tx_sequence_out,

    // RX Lane
    input [63:0] rx_userdata_in,
    input rx_datavalid_in,
    input [1:0] rx_header_in,
    input rx_headervalid_in,
    output reg rx_gearboxslip,
    output reg rx_polarity,
    
    // RX End
    output reg [31:0] rx_data,
    output reg rx_valid,
    output reg error_flag
    );

    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= RESET/INITIALIZATION =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
          
    // Reset for at least 512 cycles to ensure gearbox alignment
    reg [9:0] reset_cycle_count;
    // Registers to lock when reset is complete. Need extra registers to cross clock domains     
    reg resetstate_n_initclk;
    reg resetstate_n_intermediate;
    reg resetstate_n_txuserclk;
    // Raised when this core's gearbox is aligned
    reg coreresetdone_rxclk;
    reg coreresetdone_txclkmeta;  
    reg coreresetdone_txclkstable; 
    // Raised when partner's gearbox is aligned
    reg partner_coreresetdone_rxclk;    
    reg partner_coreresetdone_txclkmeta; 
    reg partner_coreresetdone_txclkstable; 
    
    // Some initialization logic from transceivers must operate on separate clock for timing purposes (too slow for GT clocks)                  
    always @(posedge init_clk) begin
        // Initiate a reset under a set of conditions
        if (!tx_corereset_n || !tx_resetdone || !rx_resetdone) resetstate_n_initclk <= 0;
        else if (!resetstate_n_initclk) resetstate_n_initclk <= 1;
    end         
    
    
    // Note: functionality could be combined to use only 2 of these keys, but I keep them distinct for debugging/readability.
    // Initial random seed for LFSR (16x1, 16x0)
    localparam [31:0] LFSR_SEED = 32'hE973254A;
    // Used to tell partner when gearbox slips (17x1, 15x0)
    localparam [31:0] RESET_SIGNAL = 32'hAA8A9BCD;
    // Signals gearbox is aligned and checks polarity (17x1, 15x0)
    localparam [31:0] POLARITY_CHECK = 32'hB4A5D4AE;    
    // Scrambles data for DC balance (32x1, 32x0)
    localparam [63:0] SCRAMBLE_KEY = 64'h65B68EA4B5CCA8B4;

    // Semi-random sequence generator to align gearbox
    reg [31:0] lfsr_reg;
    wire feedback;
    assign feedback = lfsr_reg[31] ^ lfsr_reg[30] ^ lfsr_reg[29] ^ lfsr_reg[28];
    
    // Generates next LFSR sequence and passes it to TX lane
    task cycle_lfsr;
        if (lfsr_reg == 32'b0) begin 
            lfsr_reg <= LFSR_SEED;
            send_encoded_msg(2'b01, LFSR_SEED, SCRAMBLE_KEY, tx_userdata_out);
        end
        else begin
            lfsr_reg <= {lfsr_reg[30:0], feedback}; 
            send_encoded_msg(2'b01, lfsr_reg, SCRAMBLE_KEY, tx_userdata_out);
        end
    endtask
    
    
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= ERROR CORRECTION ENCODER =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    
    // Calculate all parity bits for a 64-bit message
    task send_encoded_msg;
        input [1:0] header;
        input [31:0] message;
        input [63:0] scrambler;
        output [63:0] destination;
        begin 
            destination[63:62] <= header ^ scrambler[63:62];
            destination[61:60] <= header ^ scrambler[61:60];
            
            destination[48] <= (message[31]) ^ (message[29]) ^ (message[27]) ^ (message[25]) ^ (message[23]) ^ (message[21]) ^ (message[19]) ^ (message[17])
                                ^ (message[15]) ^ (message[13]) ^ (message[11]) ^ (message[9]) ^ (message[7]) ^ (message[5]) ^ (message[3]) ^ (message[1]) ^ scrambler[48];   
            destination[54] <= (message[31]) ^ (message[29]) ^ (message[27]) ^ (message[25]) ^ (message[23]) ^ (message[21]) ^ (message[19]) ^ (message[17])
                                ^ (message[15]) ^ (message[13]) ^ (message[11]) ^ (message[9]) ^ (message[7]) ^ (message[5]) ^ (message[3]) ^ (message[1]) ^ scrambler[54];      

            destination[49] <= (^message[31:30]) ^ (^message[27:26]) ^ (^message[23:22]) ^ (^message[19:18]) ^ (^message[15:14]) ^ (^message[11:10]) ^ (^message[7:6]) ^ (^message[3:2]) ^ scrambler[49];
            destination[55] <= (^message[31:30]) ^ (^message[27:26]) ^ (^message[23:22]) ^ (^message[19:18]) ^ (^message[15:14]) ^ (^message[11:10]) ^ (^message[7:6]) ^ (^message[3:2]) ^ scrambler[55];

            destination[50] <= (^message[31:28]) ^ (^message[23:20]) ^ (^message[15:12]) ^ (^message[7:4]) ^ scrambler[50];
            destination[56] <= (^message[31:28]) ^ (^message[23:20]) ^ (^message[15:12]) ^ (^message[7:4]) ^ scrambler[56];

            destination[51] <= (^message[31:24]) ^ (^message[15:8]) ^ scrambler[51];
            destination[57] <= (^message[31:24]) ^ (^message[15:8]) ^ scrambler[57];

            destination[52] <= ^message[31:16] ^ scrambler[52];
            destination[58] <= ^message[31:16] ^ scrambler[58];
            
            destination[53] <= message[0] ^ scrambler[53];
            destination[59] <= message[0] ^ scrambler[59];

            destination[47:32] <= (message[31:16] + message[15:0]) ^ scrambler[47:32];

            destination[31:0] <= message[31:0] ^ scrambler[31:0];
        end
    endtask
    
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= TRANSMITTER SIDE =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    
    // Sequence increments every *other* cycle
    reg increment_sequence;
    wire sequence_pause_next;
    // Wire for indicating whether or not data can be sent on the next cycle due to sequence pause
    assign sequence_pause_next = ((tx_sequence_out == 7'h1f && increment_sequence) || (tx_sequence_out == 7'h20 && !increment_sequence));
    
    
    // TX Processing
    always @(posedge tx_userclk) begin
        // Stabilize cross-clock domain registers
        resetstate_n_intermediate <= resetstate_n_initclk;
        coreresetdone_txclkmeta <= coreresetdone_rxclk;
        coreresetdone_txclkstable <= coreresetdone_txclkmeta;
        partner_coreresetdone_txclkmeta <= partner_coreresetdone_rxclk;
        partner_coreresetdone_txclkstable <= partner_coreresetdone_txclkmeta;
        
        // Sequence
        increment_sequence <= !increment_sequence;
        if (increment_sequence) begin
            if (tx_sequence_out >= 7'h20) tx_sequence_out <= 0;
            else tx_sequence_out <= tx_sequence_out + 1;
        end
                
        // Reset state
        if (!resetstate_n_txuserclk || !coreresetdone_txclkstable || !partner_coreresetdone_txclkstable || reset_cycle_count > 0) begin
            tx_ready <= 0;
            // Every 512 cycles consider exiting the reset
            if (reset_cycle_count >= 10'b1000000000 && !sequence_pause_next) begin
                reset_cycle_count <= 0;
                // If my gearbox is aligned, send a polarity check message
                if (coreresetdone_txclkstable) begin
                    tx_header_out <= 2'b00;
                    send_encoded_msg(2'b00, POLARITY_CHECK, SCRAMBLE_KEY, tx_userdata_out);
                    // If partner's gearbox is also aligned, exit reset (assuming reset input is low, power is good)
                    if (partner_coreresetdone_txclkstable) resetstate_n_txuserclk <= resetstate_n_intermediate;
                end else begin
                    tx_header_out <= 2'b01;
                    cycle_lfsr();
                end
            end
            // In between, send semi-random sequences with control headers
            else begin
                reset_cycle_count <= reset_cycle_count + 1;
                if (reset_cycle_count == 64'b0 && !coreresetdone_txclkstable) begin
                    tx_header_out <= 2'b00;
                    send_encoded_msg(2'b00, RESET_SIGNAL, SCRAMBLE_KEY, tx_userdata_out);
                end
                else begin
                    tx_header_out <= 2'b01;
                    cycle_lfsr();
                end
            end
        end  
        
        else begin
            // Prepare/synchronize reset registers
            resetstate_n_txuserclk <= resetstate_n_intermediate;
            reset_cycle_count <= 0;
            
            // Streaming state
            if (gt_powergood) begin
                // Cannot accept data during sequence pause
                if (tx_sequence_out == 7'h1f) tx_ready <= 0;
                else tx_ready <= 1;
                
                // Handshake with user program, scramble and pass data to TX lane, update bitwise parity counts
                if (tx_valid && tx_ready) begin
                    send_encoded_msg(2'b10, tx_data, SCRAMBLE_KEY, tx_userdata_out);
                    tx_header_out <= 2'b10;
                end else begin
                    cycle_lfsr();
                    tx_header_out <= 2'b01;
                end
            end
             
            // Waiting for gigabit transceiver, send idle (semi-random) blocks
            else begin
                cycle_lfsr();
                tx_header_out <= 2'b01;
                tx_ready <= 0;
            end 
        end
    end
    
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= RECEIVER SIDE =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    
    // Need many healthy cycles to confirm gearbox alignment
    reg [6:0] gearboxaligned_count;
    
    // Registers used for determining if partner shuts down. Typically, when a board shuts down, it sends ...101010... or ...111111... across the link
    reg [1:0] prev_header;
    reg [63:0] prev_data;
    reg [9:0] consecutiveidenticalblock_count;
    
    wire [63:0] rx_data_unscrambled;
    assign rx_data_unscrambled = rx_userdata_in ^ SCRAMBLE_KEY;
    
    wire [1:0] majority_header;
    assign majority_header = (rx_data_unscrambled[63:62] & rx_data_unscrambled[61:60]) | (rx_header_in & rx_data_unscrambled[63:62]) | (rx_header_in & rx_data_unscrambled[61:60]);
    
    wire [5:0] rx_parity_reg;
    assign rx_parity_reg[5] = rx_data_unscrambled[0];
    assign rx_parity_reg[4] = ^rx_data_unscrambled[31:16];
    assign rx_parity_reg[3] = (^rx_data_unscrambled[31:24]) ^ (^rx_data_unscrambled[15:8]);
    assign rx_parity_reg[2] = (^rx_data_unscrambled[31:28]) ^ (^rx_data_unscrambled[23:20]) ^ (^rx_data_unscrambled[15:12]) ^ (^rx_data_unscrambled[7:4]);
    assign rx_parity_reg[1] = (^rx_data_unscrambled[31:30]) ^ (^rx_data_unscrambled[27:26]) ^ (^rx_data_unscrambled[23:22]) ^ (^rx_data_unscrambled[19:18]) ^ (^rx_data_unscrambled[15:14]) ^ (^rx_data_unscrambled[11:10]) ^ (^rx_data_unscrambled[7:6]) ^ (^rx_data_unscrambled[3:2]);
    assign rx_parity_reg[0] = (rx_data_unscrambled[31]) ^ (rx_data_unscrambled[29]) ^ (rx_data_unscrambled[27]) ^ (rx_data_unscrambled[25]) ^ (rx_data_unscrambled[23]) ^ (rx_data_unscrambled[21]) ^ (rx_data_unscrambled[19]) ^ (rx_data_unscrambled[17])
                              ^ (rx_data_unscrambled[15]) ^ (rx_data_unscrambled[13]) ^ (rx_data_unscrambled[11]) ^ (rx_data_unscrambled[9]) ^ (rx_data_unscrambled[7]) ^ (rx_data_unscrambled[5]) ^ (rx_data_unscrambled[3]) ^ (rx_data_unscrambled[1]);
    
    // RX processing
    always @(posedge rx_userclk) begin  
        if (gt_powergood) begin
            if (rx_headervalid_in == rx_datavalid_in) begin
            
                // Count consecutive identical blocks to detect unexpected partner shutdown
                if (rx_datavalid_in) begin
                    prev_header <= majority_header;
                    prev_data <= rx_userdata_in;
                    if (prev_header == majority_header && prev_data == rx_userdata_in) consecutiveidenticalblock_count <= consecutiveidenticalblock_count + 1;
                    else consecutiveidenticalblock_count <= 0;
                end
                // If partner shut down, wait in reset state
                if (partner_coreresetdone_rxclk && consecutiveidenticalblock_count > 10'b1000000000) begin
                    consecutiveidenticalblock_count <= 0;
                    coreresetdone_rxclk <= 0;
                    partner_coreresetdone_rxclk <= 0;
                    error_flag <= 1;
                end 
            
                else case (majority_header)
                    // Data header received
                    2'b10: begin
                        rx_gearboxslip <= 0;
                        if (rx_headervalid_in && rx_userdata_in != prev_data && !coreresetdone_rxclk) gearboxaligned_count <= gearboxaligned_count +1;
                        // Add data to the queue if header is valid and reset is done
                        if (rx_headervalid_in && coreresetdone_rxclk) begin
                            rx_valid <= 1;
                            if (rx_data_unscrambled[47:32] != rx_data_unscrambled[31:16] + rx_data_unscrambled[15:0]) begin
                                rx_data <= rx_data_unscrambled[31:0];
                                error_flag <= 1;
                            end else if (rx_data_unscrambled[59:54] == rx_data_unscrambled[53:48] && rx_data_unscrambled[53:48] != rx_parity_reg) begin
                                rx_data <= rx_data_unscrambled[31:0] ^ (32'b1 << (rx_data_unscrambled[53:48] ^ rx_parity_reg));
                                error_flag <= 0;
                            end else begin
                                rx_data <= rx_data_unscrambled[31:0];
                                error_flag <= 0;
                            end
                        end else begin
                            error_flag <= 0;
                            rx_valid <= 0;
                        end
                    end
                    // Control header received
                    2'b01: begin                    
                        rx_gearboxslip <= 0;
                        rx_valid <= 0;
                        if (rx_userdata_in != prev_data && !coreresetdone_rxclk) gearboxaligned_count <= gearboxaligned_count +1;
                        // Wait to exit reset until gearbox is aligned for 64 consecutive cycles
                        if (gearboxaligned_count > 7'b1000000) begin
                            gearboxaligned_count <= 0;
                            coreresetdone_rxclk <= 1;
                        end
                    end 
                    // 2'b00 and 2'b11 are unused
                    default begin
                        if (rx_headervalid_in) begin
                            // After partner gearbox is aligned it sends a known message allowing us to check polarity
                            if (majority_header == 2'b11 && rx_data_unscrambled[31:0] == ~POLARITY_CHECK) begin
                                rx_polarity <= !rx_polarity;
                                rx_gearboxslip <= 0;
                                partner_coreresetdone_rxclk <= 1;
                            end else if (majority_header == 2'b00 && rx_data_unscrambled[31:0] == POLARITY_CHECK) begin
                                rx_gearboxslip <= 0;
                                partner_coreresetdone_rxclk <= 1;
                            // If partner gearbox is (potentially) not aligned, it sends reset signals so its partner knows to sending messages
                            end else if (majority_header == 2'b11 && rx_data_unscrambled[31:0] == ~RESET_SIGNAL) begin
                                rx_polarity <= !rx_polarity;
                                rx_gearboxslip <= 0;
                                if (partner_coreresetdone_rxclk) begin
                                    coreresetdone_rxclk <= 0;
                                    partner_coreresetdone_rxclk <= 0;
                                    error_flag <= 1;
                                end
                            end else if (majority_header == 2'b00 && rx_data_unscrambled[31:0] == RESET_SIGNAL) begin
                                rx_gearboxslip <= 0;
                                if (partner_coreresetdone_rxclk) begin
                                    coreresetdone_rxclk <= 0;
                                    partner_coreresetdone_rxclk <= 0;
                                    error_flag <= 1;
                                end
                            // All other 00/11 headers are gearbox slips. As such, header bit flips can cause unnecessary resets and interrupt transmission
                            end else begin
                                rx_gearboxslip <= 1;
                                gearboxaligned_count <= 0;
                                coreresetdone_rxclk <= 0;
                                if (coreresetdone_rxclk || rx_valid) error_flag <= 1;
                            end                    
                        end
                    end
                endcase
            end else error_flag <= 1;
        end
    end
        
endmodule
