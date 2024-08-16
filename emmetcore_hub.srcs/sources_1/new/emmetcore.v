`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
//
// Company: Yale Efficient Computing Lab
// Engineer: Emmet Houghton
//
// Create Date: 07/02/2024 12:48:57 PM
// Design Name: 64B/66B Low-latency transceiver core
// Module Name: emmetcore
// Target Devices: Programmed on xcvm1802-vsva2197-2MP-e-S (VMK180 Evaluation Board)
// Tool Versions: V1.0
// Description: Low-latency transceiver module for 64B/66B encoding
//
// Some notes on using this core:
// - Recommended line rate: link is stable at line rates up to 20 Gbps error free with a cable less than 1 meter in length
// - The core resets while tx_corereset_n is *low* so that it resets upon initialization automatically. Expect delay after reset.
// - The code assumes both partners are set to the same line rate (so that tx_userclk and rx_userclk have the same frequencies on each end).
// - An early error flag (within ~128 cycles after streaming begins) suggests all data is likely invalid.
// - A later error flag could suggest a gearbox drift, partner shutdown, or bit flips. Try resending packet.
// - If streaming doesn't resume after boards are reprogrammed, lower tx_corereset_n for at least one cycle.
//
//////////////////////////////////////////////////////////////////////////////////

module emmetcore(
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
    input [63:0] tx_data,
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
    output reg [63:0] rx_data,
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

    // Initial random seed for LFSR (32x1, 32x0)
    localparam [63:0] LFSR_SEED = 64'hE9734555354A526D;
    // Used to tell partner when gearbox slips (33x1, 31x0)
    localparam [63:0] RESET_SIGNAL = 64'hAA8A8BCD4E95FD3C;
    // Signals gearbox is aligned and checks polarity (33x1, 31x0)
    localparam [63:0] POLARITY_CHECK = 64'hB4A5D4AE376C9323;    
    // Key to scramble parity messages must be different from scrambler (32x1, 32x0)
    localparam [63:0] PARITY_KEY = 64'hD5F4A1B3E6C9D2F1;
    // Scrambles data for DC balance (32x1, 32x0)
    localparam [63:0] SCRAMBLE_KEY = 64'h56B68EA4B5CCA8B4;

    // Semi-random sequence generator to align gearbox
    reg [63:0] lfsr_reg;
    wire feedback;
    assign feedback = lfsr_reg[63] ^ lfsr_reg[62] ^ lfsr_reg[60] ^ lfsr_reg[59];
   
    // Generates next LFSR sequence and passes it to TX lane
    task cycle_lfsr;
        if (lfsr_reg == 64'b0) begin
            lfsr_reg <= LFSR_SEED;
            tx_userdata_out <= LFSR_SEED;
        end
        else begin
            lfsr_reg <= {lfsr_reg[62:0], feedback};
            tx_userdata_out <= lfsr_reg ^ SCRAMBLE_KEY;
        end
    endtask
   
   
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= ERROR CORRECTION ENCODER =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
   
    // Calculate all parity bits for a 64-bit message
    task hamming_encode;
        input [63:0] message_to_encode;
        inout [63:0] parity_register;
        begin
            parity_register[63:40] <= parity_register[55:32];
            parity_register[31:8] <= parity_register[23:0];

            parity_register[0] <= (message_to_encode[63]) ^ (message_to_encode[61]) ^ (message_to_encode[59]) ^ (message_to_encode[57]) ^ (message_to_encode[55]) ^ (message_to_encode[53]) ^ (message_to_encode[51]) ^ (message_to_encode[49])
                                  ^ (message_to_encode[47]) ^ (message_to_encode[45]) ^ (message_to_encode[43]) ^ (message_to_encode[41]) ^ (message_to_encode[39]) ^ (message_to_encode[37]) ^ (message_to_encode[35]) ^ (message_to_encode[33])
                                  ^ (message_to_encode[31]) ^ (message_to_encode[29]) ^ (message_to_encode[27]) ^ (message_to_encode[25]) ^ (message_to_encode[23]) ^ (message_to_encode[21]) ^ (message_to_encode[19]) ^ (message_to_encode[17])
                                  ^ (message_to_encode[15]) ^ (message_to_encode[13]) ^ (message_to_encode[11]) ^ (message_to_encode[9]) ^ (message_to_encode[7]) ^ (message_to_encode[5]) ^ (message_to_encode[3]) ^ (message_to_encode[1]);
            parity_register[32] <= (message_to_encode[63]) ^ (message_to_encode[61]) ^ (message_to_encode[59]) ^ (message_to_encode[57]) ^ (message_to_encode[55]) ^ (message_to_encode[53]) ^ (message_to_encode[51]) ^ (message_to_encode[49])
                                  ^ (message_to_encode[47]) ^ (message_to_encode[45]) ^ (message_to_encode[43]) ^ (message_to_encode[41]) ^ (message_to_encode[39]) ^ (message_to_encode[37]) ^ (message_to_encode[35]) ^ (message_to_encode[33])
                                  ^ (message_to_encode[31]) ^ (message_to_encode[29]) ^ (message_to_encode[27]) ^ (message_to_encode[25]) ^ (message_to_encode[23]) ^ (message_to_encode[21]) ^ (message_to_encode[19]) ^ (message_to_encode[17])
                                  ^ (message_to_encode[15]) ^ (message_to_encode[13]) ^ (message_to_encode[11]) ^ (message_to_encode[9]) ^ (message_to_encode[7]) ^ (message_to_encode[5]) ^ (message_to_encode[3]) ^ (message_to_encode[1]);      

            parity_register[1] <= (^message_to_encode[63:62]) ^ (^message_to_encode[59:58]) ^ (^message_to_encode[55:54]) ^ (^message_to_encode[51:50]) ^ (^message_to_encode[47:46]) ^ (^message_to_encode[43:42]) ^ (^message_to_encode[39:38]) ^ (^message_to_encode[35:34])
                                  ^ (^message_to_encode[31:30]) ^ (^message_to_encode[27:26]) ^ (^message_to_encode[23:22]) ^ (^message_to_encode[19:18]) ^ (^message_to_encode[15:14]) ^ (^message_to_encode[11:10]) ^ (^message_to_encode[7:6]) ^ (^message_to_encode[3:2]);
            parity_register[33] <= (^message_to_encode[63:62]) ^ (^message_to_encode[59:58]) ^ (^message_to_encode[55:54]) ^ (^message_to_encode[51:50]) ^ (^message_to_encode[47:46]) ^ (^message_to_encode[43:42]) ^ (^message_to_encode[39:38]) ^ (^message_to_encode[35:34])
                                  ^ (^message_to_encode[31:30]) ^ (^message_to_encode[27:26]) ^ (^message_to_encode[23:22]) ^ (^message_to_encode[19:18]) ^ (^message_to_encode[15:14]) ^ (^message_to_encode[11:10]) ^ (^message_to_encode[7:6]) ^ (^message_to_encode[3:2]);

            parity_register[2] <= (^message_to_encode[63:60]) ^ (^message_to_encode[55:52]) ^ (^message_to_encode[47:44]) ^ (^message_to_encode[39:36]) ^ (^message_to_encode[31:28]) ^ (^message_to_encode[23:20]) ^ (^message_to_encode[15:12]) ^ (^message_to_encode[7:4]);
            parity_register[34] <= (^message_to_encode[63:60]) ^ (^message_to_encode[55:52]) ^ (^message_to_encode[47:44]) ^ (^message_to_encode[39:36]) ^ (^message_to_encode[31:28]) ^ (^message_to_encode[23:20]) ^ (^message_to_encode[15:12]) ^ (^message_to_encode[7:4]);

            parity_register[3] <= (^message_to_encode[63:56]) ^ (^message_to_encode[47:40]) ^ (^message_to_encode[31:24]) ^ (^message_to_encode[15:8]);
            parity_register[35] <= (^message_to_encode[63:56]) ^ (^message_to_encode[47:40]) ^ (^message_to_encode[31:24]) ^ (^message_to_encode[15:8]);

            parity_register[4] <= (^message_to_encode[63:48]) ^ (^message_to_encode[31:16]);
            parity_register[36] <= (^message_to_encode[63:48]) ^ (^message_to_encode[31:16]);

            parity_register[5] <= ^message_to_encode[63:32];
            parity_register[37] <= ^message_to_encode[63:32];
           
            parity_register[6] <= message_to_encode[0];
            parity_register[7] <= ^message_to_encode;
            parity_register[38] <= message_to_encode[0];
            parity_register[39] <= ^message_to_encode;
        end
    endtask
   
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= TRANSMITTER SIDE =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
   
    // Sequence increments every *other* cycle
    reg increment_sequence;
    wire sequence_pause_next;
    // Wire for indicating whether or not data can be sent on the next cycle due to sequence pause
    assign sequence_pause_next = ((tx_sequence_out == 7'h1f && increment_sequence) || (tx_sequence_out == 7'h20 && !increment_sequence));
   
    // Register to store the hamming code information of last four data blocks. Last 8 bits for most recent data message, previous 8 for penultimate, etc.
    reg [63:0] tx_parity_reg;
    // Counter for ensuring we do not send more than 4 data blocks in a row (caps latency for error correction at cost of throughput)
    reg [2:0] tx_datablockcount;
   
   
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
                    tx_userdata_out <= POLARITY_CHECK;
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
                    tx_userdata_out <= RESET_SIGNAL;
                end
                else begin
                    tx_header_out <= 2'b01;
                    cycle_lfsr();
                end
            end
            tx_parity_reg <= 64'b0;
            tx_datablockcount <= 3'b0;      
        end  
       
        else begin
            // Prepare/synchronize reset registers
            resetstate_n_txuserclk <= resetstate_n_intermediate;
            reset_cycle_count <= 0;
           
            // Streaming state
            if (gt_powergood) begin
                // Cannot accept data during sequence pause
                if (tx_sequence_out == 7'h1f || (tx_datablockcount == 3'b11 && tx_ready && tx_valid) || (tx_datablockcount == 3'b100 && sequence_pause_next)) tx_ready <= 0;
                else tx_ready <= 1;
               
                // Handshake with user program, scramble and pass data to TX lane, update bitwise parity counts
                if (tx_valid && tx_ready) begin
                    tx_userdata_out <= tx_data ^ SCRAMBLE_KEY;
                    tx_header_out <= 2'b10;
                    hamming_encode(tx_data, tx_parity_reg);
                    tx_datablockcount <= tx_datablockcount + 1;
                end else begin
                    // Bitwise parity of data from last set of cycles is scrambled and sent as control block on sequence 7'h00 and between data blocks
                    if (!sequence_pause_next && (tx_header_out == 2'b10 || tx_datablockcount != 3'b000)) begin
                        tx_userdata_out <= tx_parity_reg ^ PARITY_KEY;
                        tx_header_out <= 2'b01;
                        tx_datablockcount <= 3'b0;      
                    end else begin
                        cycle_lfsr();
                        tx_header_out <= 2'b01;
                    end
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
   
    // Receiver maintains a queue of 4 data blocks because it must wait for error correction information
    reg [63:0] rx_dataqueue_1;
    reg [63:0] rx_dataqueue_2;
    reg [63:0] rx_dataqueue_3;
    reg [63:0] rx_dataqueue_4;
    reg [2:0] queuesize;
    reg [2:0] queuecorrected;
    reg queueinit;
   
    // Wire which unscrambles error correction messages (useful abstraction)
    wire [63:0] parity_data_in;
    assign parity_data_in = (rx_userdata_in ^ PARITY_KEY);
    // tx_parity_reg is updated every time a data block is sent, rx_parity_reg is updated every time a data message is received
    reg [63:0] rx_parity_reg;
   
    // Need many healthy cycles to confirm gearbox alignment
    reg [6:0] gearboxaligned_count;
   
    // Registers used for determining if partner shuts down. Typically, when a board shuts down, it sends ...101010... or ...111111... across the link
    reg [1:0] prev_header;
    reg [63:0] prev_data;
    reg [9:0] consecutiveidenticalblock_count;
   
   
    // When a data block is received, enqueue appropriately and dequeue if a corrected message is available
    task shift_queue;    
        begin
            hamming_encode(rx_userdata_in ^ SCRAMBLE_KEY, rx_parity_reg);
             
            rx_dataqueue_1 <= rx_userdata_in ^ SCRAMBLE_KEY;
            if (!queueinit || queuesize == 3'b0) begin
                queueinit <= 1'b1;
                queuesize <= 3'b1;
                queuecorrected <= 3'b0;
                rx_valid <= 0;
            end else if (queuecorrected > 3'b0 || queuesize >= 3'b100) begin
                if (queuesize > 3'b1) begin
                    rx_dataqueue_2 <= rx_dataqueue_1;
                    if (queuesize > 3'b10) begin
                        rx_dataqueue_3 <= rx_dataqueue_2;
                        if (queuesize > 3'b11) begin
                            rx_dataqueue_4 <= rx_dataqueue_3;
                            rx_data <= rx_dataqueue_4;
                        end else rx_data <= rx_dataqueue_3;
                    end else rx_data <= rx_dataqueue_2;
                end else rx_data <= rx_dataqueue_1;
                rx_valid <= 1;
                if (queuecorrected > 3'b0) queuecorrected <= queuecorrected - 1;
               
            end else begin
                queuesize <= queuesize +1;
                rx_dataqueue_4 <= rx_dataqueue_3;
                rx_dataqueue_3 <= rx_dataqueue_2;
                rx_dataqueue_2 <= rx_dataqueue_1;
                rx_valid <= 0;
            end
        end
    endtask
   
    // When a non-data block is received, dequeue if a corrected message is available
    task dequeue_only;
        begin
            if (queuesize > 3'b0 && queuecorrected > 3'b0) begin
                queuesize <= queuesize - 1;
                case (queuesize)
                    3'b100: rx_data <= rx_dataqueue_4;
                    3'b011: rx_data <= rx_dataqueue_3;
                    3'b010: rx_data <= rx_dataqueue_2;
                    3'b001: rx_data <= rx_dataqueue_1;
                endcase
                rx_valid <= 1;
                queuecorrected <= queuecorrected - 1;
            end
            else rx_valid <= 0;
        end
    endtask
   
   
    // RX processing
    always @(posedge rx_userclk) begin  
        if (gt_powergood) begin
            if (rx_headervalid_in == rx_datavalid_in) begin
           
                // Count consecutive identical blocks to detect unexpected partner shutdown
                if (rx_datavalid_in) begin
                    prev_header <= rx_header_in;
                    prev_data <= rx_userdata_in;
                    if (prev_header == rx_header_in && prev_data == rx_userdata_in) consecutiveidenticalblock_count <= consecutiveidenticalblock_count + 1;
                    else consecutiveidenticalblock_count <= 0;
                end
                // If partner shut down, wait in reset state
                if (partner_coreresetdone_rxclk && consecutiveidenticalblock_count > 10'b1000000000) begin
                    consecutiveidenticalblock_count <= 0;
                    coreresetdone_rxclk <= 0;
                    partner_coreresetdone_rxclk <= 0;
                    error_flag <= 1;
                end
           
                else case (rx_header_in)
                    // Data header received
                    2'b10: begin
                        rx_gearboxslip <= 0;
                        if (rx_headervalid_in && rx_userdata_in != prev_data && !coreresetdone_rxclk) gearboxaligned_count <= gearboxaligned_count +1;
                        else gearboxaligned_count <= 0;
                        // Add data to the queue if header is valid and reset is done
                        if (rx_headervalid_in && coreresetdone_rxclk) shift_queue();
                        else dequeue_only();
                        error_flag <= (queuecorrected > 3'b0) ? 0 : 1;
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
                        // Error correction logic corrects single bit flips, detects double bit flips in data field
                        if (coreresetdone_rxclk) begin
                            if (rx_headervalid_in && prev_header == 2'b10) begin
                                rx_valid <= 0;
                                queuecorrected <= queuesize;
                                if (parity_data_in != rx_parity_reg) begin
                                    if (((queuesize - queuecorrected) > 3'b0) && (parity_data_in[38:32] == parity_data_in[6:0]) && (parity_data_in[6:0] != rx_parity_reg[6:0])) begin
                                        if (parity_data_in[39] == parity_data_in[7] && parity_data_in[7] == rx_parity_reg[7]) error_flag <= 1;
                                        else begin rx_dataqueue_1 <= rx_dataqueue_1 ^ (64'b1 << (parity_data_in[5:0] ^ rx_parity_reg[5:0])); error_flag <= 0; end
                                    end if (((queuesize - queuecorrected) > 3'b1) && (parity_data_in[46:40] == parity_data_in[14:8]) && (parity_data_in[14:8] != rx_parity_reg[14:8])) begin
                                        if (parity_data_in[47] == parity_data_in[15] && parity_data_in[15] == rx_parity_reg[15]) error_flag <= 1;
                                        else begin rx_dataqueue_2 <= rx_dataqueue_2 ^ (64'b1 << (parity_data_in[13:8] ^ rx_parity_reg[13:8])); error_flag <= 0; end                                                                        
                                    end if (((queuesize - queuecorrected) > 3'b10) && (parity_data_in[54:48] == parity_data_in[22:16]) && (parity_data_in[22:16] != rx_parity_reg[22:16])) begin
                                        if (parity_data_in[55] == parity_data_in[23] && parity_data_in[23] == rx_parity_reg[23]) error_flag <= 1;
                                        else begin rx_dataqueue_3 <= rx_dataqueue_3 ^ (64'b1 << (parity_data_in[21:16] ^ rx_parity_reg[21:16])); error_flag <= 0; end                                                                     
                                    end if (((queuesize - queuecorrected) > 3'b11) && (parity_data_in[62:56] == parity_data_in[30:24]) && (parity_data_in[30:24] != rx_parity_reg[30:24])) begin
                                        if (parity_data_in[63] == parity_data_in[31] && parity_data_in[31] == rx_parity_reg[31]) error_flag <= 1;
                                        else begin rx_dataqueue_4 <= rx_dataqueue_4 ^ (64'b1 << (parity_data_in[29:24] ^ rx_parity_reg[29:24])); error_flag <= 0; end                                                                        
                                    end
                                end else begin
                                    error_flag <= 0;
                                    dequeue_only();
                                end
                            end else begin 
                                dequeue_only();
                                error_flag <= 0;
                            end
                        end
                    end
                    // 2'b00 and 2'b11 are unused
                    default begin
                        if (coreresetdone_rxclk) dequeue_only();
                        if (rx_headervalid_in) begin
                            // After partner gearbox is aligned it sends a known message allowing us to check polarity
                            if (rx_header_in == 2'b11 && rx_userdata_in == ~POLARITY_CHECK) begin
                                rx_polarity <= !rx_polarity;
                                rx_gearboxslip <= 0;
                                partner_coreresetdone_rxclk <= 1;
                            end else if (rx_header_in == 2'b00 && rx_userdata_in == POLARITY_CHECK) begin
                                rx_gearboxslip <= 0;
                                partner_coreresetdone_rxclk <= 1;
                            // If partner gearbox is (potentially) not aligned, it sends reset signals so its partner knows to sending messages
                            end else if (rx_header_in == 2'b11 && rx_userdata_in == ~RESET_SIGNAL) begin
                                rx_polarity <= !rx_polarity;
                                rx_gearboxslip <= 0;
                                if (partner_coreresetdone_rxclk) begin
                                    coreresetdone_rxclk <= 0;
                                    partner_coreresetdone_rxclk <= 0;
                                    error_flag <= 1;
                                end
                            end else if (rx_header_in == 2'b00 && rx_userdata_in == RESET_SIGNAL) begin
                                rx_gearboxslip <= 0;
                                if (coreresetdone_rxclk) begin
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
