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
// - The core resets while tx_corereset_n is *low* so that it resets upon initialization automatically.
// - The code assumes both partners are set to the same line rate (so that tx_userclk and rx_userclk have the same frequencies on each end).
// - Expect a delay of roughly 1500 cycles of {tx/rx}_userclk upon each reset. This delay ensures both gearboxes are aligned properly.
// - An early error flag (within ~128 cycles after streaming begins) suggests all data is likely invalid.
// - A later error flag could suggest a gearbox drift, partner shutdown, or a bit flip. Try resetting and re-attempt.
// - If streaming doesn't resume after boards are reprogrammed, raise tx_corereset_n for at least one cycle.
//
//////////////////////////////////////////////////////////////////////////////////

module emmetcore(
    // Reset signal (active low)
    input tx_corereset_n,
    // Separate, slower clock for initialization
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
                  
    always @(posedge init_clk) begin
        // Initiate a reset under a set of conditions
        if (!tx_corereset_n || !tx_resetdone || !rx_resetdone) resetstate_n_initclk <= 0;
        else if (!resetstate_n_initclk) resetstate_n_initclk <= 1;
    end         
    
    
    // Initial random seed for LFSR, also used for scrambling
    localparam [63:0] SEED_KEY = 64'hEABCC48C6C567872;
    // Key to scramble parity messages (must be different from scrambler, otherwise parity messages likely contain large bit majorities)
    localparam [63:0] POLARITY_PARITY_KEY = 64'hD5F4A1B3E6C9D2F1;

    // Semi-random sequence generator to align gearbox
    reg [63:0] lfsr_reg;
    wire feedback;
    assign feedback = lfsr_reg[63] ^ lfsr_reg[62] ^ lfsr_reg[60] ^ lfsr_reg[59];
    
    // Generates next LFSR sequence and passes it to TX lane
    task cycle_lfsr;
        if (lfsr_reg == 64'b0) begin 
            lfsr_reg <= SEED_KEY;
            tx_userdata_out <= SEED_KEY;
        end
        else begin
            lfsr_reg <= {lfsr_reg[62:0], feedback}; 
            tx_userdata_out <= lfsr_reg;
        end
    endtask
    
    
    // Bitwise parity counters for weak error detection
    reg [63:0] tx_parity_reg;
    reg [63:0] rx_parity_reg;
    
    // Sequence increments every *other* cycle
    reg increment_sequence;
       
    
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
            // Every 512 cycles consider exiting the reset cycle
            if (reset_cycle_count >= 10'b1000000000 && !((tx_sequence_out == 7'h1f && increment_sequence) || (tx_sequence_out == 7'h20 && !increment_sequence))) begin
                reset_cycle_count <= 0;
                // If my gearbox is aligned, send a polarity check message
                if (coreresetdone_txclkstable) begin
                    tx_header_out <= 2'b00;
                    tx_userdata_out <= POLARITY_PARITY_KEY;
                    // If partner's gearbox is also aligned, exit reset (assuming reset input is low, power is good)
                    if (partner_coreresetdone_txclkstable) resetstate_n_txuserclk <= resetstate_n_intermediate;
                end else begin
                    tx_header_out <= 2'b00;
                    tx_userdata_out <= SEED_KEY;
                end
            end
            // In between, send semi-random sequences with control headers
            else begin
                reset_cycle_count <= reset_cycle_count + 1;
                if (!coreresetdone_txclkstable && reset_cycle_count == 0) begin
                    // Immediately send message letting partner know my gearbox slipped (for error detecting purposes)
                    tx_userdata_out <= SEED_KEY;
                    tx_header_out <= 2'b00;
                end else begin 
                    tx_header_out <= 2'b01;
                    cycle_lfsr();
                end 
            end
            tx_parity_reg <= 64'b0;
        end 
        
        else begin
            // Prepare/synchronize reset registers
            resetstate_n_txuserclk <= resetstate_n_intermediate;
            reset_cycle_count <= 0;
            
            // Streaming state
            if (gt_powergood) begin
                // Cannot accept data during sequence pause
                if (tx_sequence_out == 7'h1f || (tx_sequence_out == 7'h20 && !increment_sequence)) tx_ready <= 0;
                else tx_ready <= 1;
                
                // Handshake with user program, scramble and pass data to TX lane, update bitwise parity counts
                if (tx_valid && tx_ready) begin
                    tx_userdata_out <= tx_data ^ SEED_KEY;
                    tx_header_out <= 2'b10;
                    tx_parity_reg <= tx_parity_reg ^ tx_data;
                end else begin
                    // Bitwise parity of data from last set of cycles is scrambled and sent as control block on sequence 7'h00 and between data blocks
                    if ((tx_sequence_out == 7'h20 && increment_sequence) || (tx_ready && tx_header_out == 2'b10)) begin
                        tx_userdata_out <= tx_parity_reg ^ POLARITY_PARITY_KEY;
                        tx_header_out <= 2'b01;
                        tx_parity_reg <= 64'b0;                        
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
                tx_parity_reg <= 64'b0;
                tx_ready <= 0;
            end 
        end
    end
    
    
    // Determining if partner shuts down
    reg [1:0] prev_header;
    reg [63:0] prev_data;
    reg [9:0] consecutiveidenticalblock_count;
    
    // Need many healthy cycles to confirm gearbox alignment
    reg [6:0] gearboxaligned_count;
    
    // RX processing
    always @(posedge rx_userclk) begin  
        if (gt_powergood) begin
            if (rx_headervalid_in == rx_datavalid_in) begin
            
                // Check to see if partner shut down. If so, wait in reset state
                if (rx_datavalid_in) begin
                    prev_header <= rx_header_in;
                    prev_data <= rx_userdata_in;
                    if (prev_header == rx_header_in && prev_data == rx_userdata_in) consecutiveidenticalblock_count <= consecutiveidenticalblock_count + 1;
                    else consecutiveidenticalblock_count <= 0;
                end
                if (partner_coreresetdone_rxclk && consecutiveidenticalblock_count > 10'b1000000000) begin
                    consecutiveidenticalblock_count <= 0;
                    coreresetdone_rxclk <= 0;
                    partner_coreresetdone_rxclk <= 0;
                    error_flag <= 1;
                end 
            
                else case (rx_header_in)
                    // Data header
                    2'b10: begin
                        rx_gearboxslip <= 0;
                        error_flag <= 0;
                        if (rx_headervalid_in) begin
                            rx_data <= rx_userdata_in ^ SEED_KEY;
                            rx_parity_reg <= rx_parity_reg ^ (rx_userdata_in ^ SEED_KEY);
                            if (coreresetdone_rxclk) rx_valid <= 1;
                            else if (rx_userdata_in != prev_data && !coreresetdone_rxclk) gearboxaligned_count <= gearboxaligned_count +1;
                        end else rx_valid <= 0;
                    end
                    // Control header
                    2'b01: begin                    
                        rx_gearboxslip <= 0;
                        rx_valid <= 0;
                        if (rx_userdata_in != prev_data && !coreresetdone_rxclk) gearboxaligned_count <= gearboxaligned_count +1;
                        // Wait to exit reset until gearbox is aligned for 64 consecutive cycles
                        if (gearboxaligned_count > 7'b1000000) begin
                            gearboxaligned_count <= 0;
                            coreresetdone_rxclk <= 1;
                        end
                        // Parity check for error detection
                        if (rx_headervalid_in && rx_valid && rx_parity_reg != (rx_userdata_in ^ POLARITY_PARITY_KEY)) error_flag <= 1;
                        else error_flag <= 0;
                        if (rx_headervalid_in) rx_parity_reg <= 0; 
                    end 
                    // 2'b00 and 2'b11 are unused
                    default begin
                        if (rx_headervalid_in) begin
                            if (rx_header_in == 2'b11 && rx_userdata_in == ~POLARITY_PARITY_KEY) begin
                                rx_polarity <= !rx_polarity;
                                rx_gearboxslip <= 0;
                                partner_coreresetdone_rxclk <= 1;
                            end else if (rx_header_in == 2'b00 && rx_userdata_in == POLARITY_PARITY_KEY) begin
                                rx_gearboxslip <= 0;
                                partner_coreresetdone_rxclk <= 1;
                            end else if (rx_header_in == 2'b11 && rx_userdata_in == ~SEED_KEY) begin
                                rx_polarity <= !rx_polarity;
                                rx_gearboxslip <= 0;
                                if (partner_coreresetdone_rxclk) begin
                                    partner_coreresetdone_rxclk <= 0;
                                    error_flag <= 1;
                                end
                            end else if (rx_header_in == 2'b00 && rx_userdata_in == SEED_KEY) begin
                                rx_gearboxslip <= 0;
                                if (partner_coreresetdone_rxclk) begin
                                    partner_coreresetdone_rxclk <= 0;
                                    error_flag <= 1;
                                end
                            end else begin
                                rx_gearboxslip <= 1;
                                gearboxaligned_count <= 0;
                                coreresetdone_rxclk <= 0;
                                if (coreresetdone_rxclk || rx_valid) error_flag <= 1;
                            end                    
                            rx_valid <= 0;
                        end
                    end
                endcase
            end else error_flag <= 1;
        end
    end
    
endmodule

