// ------------------------ Disclaimer -----------------------
// No warranty of correctness, synthesizability or 
// functionality of this code is given.
// Use this code under your own risk.
// When using this code, copy this disclaimer at the top of 
// Your file
//
// (c) Luca Hanel 2020
//
// ------------------------------------------------------------
//
// Module name: uart_tx
// 
// Functionality: TX part of uart
//
// ------------------------------------------------------------

module uart_tx(
    input logic             clk,
    input logic             rstn_i,
    input logic [31:0]      clk_div_i,
    input logic [7:0]       tx_data_i,
    input logic             tx_valid_i,
    output logic            tx_done_o,
    input logic             parity_en_i,
    output logic            tx_o
);

enum logic [1:0] {IDLE, TX, PARITY, STOP} CS, NS;

// for baud clk
logic [31:0]    clk_cnt;

// tx
logic           tx;

logic           parity;
logic           tx_done_n, tx_done_q;

// tx data counter
logic [2:0]     tx_cnt;
logic           incr_cnt;
logic           rst_cnt;

assign tx_o = tx;
assign tx_done_o = tx_done_q;

always_comb
begin
    tx = 1'b1;
    NS = CS;
    incr_cnt = 1'b0;
    rst_cnt = 1'b0;
    tx_done_n = 1'b0;

    if(tx_valid_i) begin
        case(CS)
            IDLE: begin
                rst_cnt = 1'b1;
                tx = 1'b1;
                if(tx_valid_i) begin
                    tx = 1'b0;
                    NS = TX;
                end
            end

            TX: begin
                tx = tx_data_i[tx_cnt];
                incr_cnt = 1'b1;
                if(tx_cnt == 'd7)
                    if(parity_en_i)
                        NS = PARITY;
                    else
                        NS = STOP;
            end

            PARITY: begin
                tx = parity;
                NS = STOP;
            end

            STOP: begin
                tx_done_n = 1'b1;
                tx = 1'b1;
                NS = IDLE;
            end
        endcase
    end
end

always_ff @(posedge clk, negedge rstn_i)
begin
     if(!rstn_i) begin
        CS <= IDLE;
        tx_cnt <= 'b0;
        clk_cnt <= 'b0;
        parity <= 'b0;
     end else begin
        // Count the clock to get the baudrate
        if(clk_cnt == clk_div_i) begin // baud tick
            CS <= NS;
            clk_cnt <= 'b0;
            tx_done_q <= tx_done_n;

            if(rst_cnt) begin
                // Reset counter and parity
                tx_cnt <= 'b0;
                parity <= 1'b0;
            end else if(incr_cnt) begin
                // increment the counter and update parity bit
                tx_cnt <= tx_cnt + 1;
                parity <= parity ^ tx_data_i[tx_cnt];
            end            
        end else begin
            // only send tx_done for one cycle
            tx_done_q <= 1'b0;
            // Only count if we are actually transmitting
            if(tx_valid_i)
                clk_cnt <= clk_cnt + 1;
            else begin
                clk_cnt <= 'b0;
                CS <= IDLE;
            end
        end //~if(clk_cnt == clk_div_i)
     end
end

endmodule