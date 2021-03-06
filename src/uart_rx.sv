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
// Module name: uart_rx
// 
// Functionality: RX part of uart
//
// ------------------------------------------------------------

module uart_rx (
    input logic             clk,
    input logic             rstn_i,
    input logic             rx_enable_i,
    input logic [31:0]      clk_div_i,
    output logic [7:0]      rx_data_o,
    output logic            rx_valid_o,
    output logic            rx_err_o,
    input logic             parity_en_i,
    input logic             rx_i
);

enum logic [2:0] {IDLE, START, RX, PARITY, STOP} CS, NS;

// For baud clk
logic [31:0]    clk_cnt;

// double flop rx
logic           rx, rx_buf;

// rx data
logic [7:0]     data_n, data_q;
logic           rx_err_n, rx_err_q;
logic           parity;
logic           rx_valid_n, rx_valid_q;

// rx data counter
logic [2:0]     rx_cnt;
logic           incr_cnt;
logic           rst_cnt;

assign rx_valid_o = rx_valid_q;
assign rx_data_o  = data_q;
assign rx_err_o   = (parity_en_i) ? rx_err_q : 1'b0;

always_comb
begin
    rst_cnt = 1'b0;
    incr_cnt = 1'b0;
    rx_valid_n = 1'b0;
    data_n = data_q;
    rx_err_n = rx_err_q;
    NS = CS;

    if(rx_enable_i) begin
        case(CS)
            IDLE: begin
                rx_err_n = 1'b0;
                rst_cnt = 1'b1;
                // Leaving IDLE is done in synchronous part, as well as
                // synchronizing with the baudrate
            end

            START: begin
                rst_cnt = 1'b1;
                NS = RX;
            end

            RX: begin
                data_n[rx_cnt] = rx;
                incr_cnt = 1'b1;
                if(rx_cnt == 'd7)
                    if(parity_en_i)
                        NS = PARITY;
                    else
                        NS = STOP;
            end

            PARITY: begin
                rx_err_n = 1'b0;
                if(parity != rx)
                    rx_err_n = 1'b1;
                NS = STOP;
            end

            STOP: begin
                rx_valid_n = 1'b1;
                NS = IDLE;
            end

            default: begin end
        endcase
    end
end

always_ff @(posedge clk, negedge rstn_i)
begin
    if(!rstn_i) begin
        CS       <= IDLE;
        data_q   <= 'b0;
        rx_err_q <= 'b0;
        rx_cnt   <= 'b0;
        clk_cnt  <= 'b0;
        parity   <= 'b0;
        rx_buf   <= 1'b1;
        rx       <= 1'b1;
    end else begin
        rx_err_q <= rx_err_n;

        // Double flop rx to mitigate metastability
        rx_buf   <= rx_i;
        rx       <= rx_buf;

        // Count the clock to get the baudrate
        if(clk_cnt == clk_div_i) begin // baud tick
            data_q     <= data_n;
            rx_valid_q <= rx_valid_n;
            clk_cnt    <= 'b0;
            CS         <= NS;

            if(rst_cnt) begin
                // Reset counter and parity
                rx_cnt <= 'b0;
                parity <= 'b0;
            end else if(incr_cnt) begin
                // increment the counter and update parity bit
                rx_cnt <= rx_cnt + 1;
                parity <= parity ^ data_n[rx_cnt];
            end
        end else begin
            // Only send rx_valid for one cycle
            rx_valid_q <= 1'b0;
            
            if(rx_enable_i) begin
                clk_cnt <= clk_cnt + 1;
                // Synchronize
                if(CS == IDLE && !rx) begin
                    clk_cnt <= clk_div_i >> 1;
                    CS      <= START;
                end
            end else begin
                clk_cnt <= 0;
                CS      <= IDLE;
            end
        end //~if(clk_cnt == clk_div_i)
    end
end

endmodule