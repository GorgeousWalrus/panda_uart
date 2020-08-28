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
// Module name: uart_module
// 
// Functionality: top module for the uart with APB bus
//
// ------------------------------------------------------------

`ifndef APB_BUS_SV
`include "apb_intf.sv"
`endif

// uart registers
`define TX_DATA 0
`define RX_DATA 1
`define CLK_DIV 2
`define CTRL 3

// control bits
`define TX_EN 0
`define TX_FULL 1
`define RX_FULL 2
`define RX_ERR 3

module uart_module #(
)(
    apb_bus_t           apb_bus,
    input logic         rx_i,
    output logic        tx_o
);

logic clk   = apb_bus.PCLK;
logic rstn  = apb_bus.PRESETn;

// APB
logic           PREADY;
logic [31:0]    PRDATA;

// registers
logic [31:0] uart_regs_n[3:0];
logic [31:0] uart_regs_q[3:0];

// TX signals
logic [1:0]     tx_cnt;
logic [7:0]     tx_data;
logic           tx_incr_cnt;
logic           tx_rst_cnt;
logic           tx_done;
logic           tx_enable;

// RX signals
logic [1:0]     rx_cnt;
logic [7:0]     rx_data;
logic           rx_incr_cnt;
logic           rx_rst_cnt;
logic           rx_valid;
logic           rx_parity_err;

assign apb_bus.PRDATA = PRDATA;
assign apb_bus.PREADY = PREADY;

always_comb
begin
    tx_enable   = 1'b0;
    tx_incr_cnt = 1'b0;
    tx_rst_cnt  = 1'b0;

    rx_incr_cnt = 1'b0;
    rx_rst_cnt  = 1'b0;

    PREADY = 1'b0;
    PRDATA = 'b0;
    uart_regs_n = uart_regs_q;

    // APB slave
    if(apb_bus.PSEL && apb_bus.PENABLE) begin
        PREADY = 1'b1;
        if(apb_bus.PWRITE) // Write
            uart_regs_n[apb_bus.PADDR[3:2]] = apb_bus.PWDATA;
            // If we write to TX data, it is full until sent
            if(apb_bus.PADDR[3:2] == `TX_DATA) begin
                uart_regs_n[`CTRL][`TX_FULL] = 1'b1;
                uart_regs_n[`CTRL][`TX_EN]   = 1'b1;
            end

        else begin // Read
            PRDATA = uart_regs_q[apb_bus.PADDR[3:2]];
            // If we read from RX data, it is no longer full
            if(apb_bus.PADDR[3:2] == `RX_DATA) begin
                uart_regs_n[`CTRL][`RX_FULL] = 1'b0;
                uart_regs_n[`RX_DATA] = 'b0;
            end
        end
    end

    // UART TX
    if(uart_regs_q[`CTRL][`TX_EN]) begin
        tx_enable = 1'b1;
        /* verilator lint_off WIDTH */
        tx_data = (uart_regs_q[`TX_DATA] >> (8*tx_cnt));
        /* verilator lint_on WIDTH */
        if(tx_done) begin
            tx_enable = 1'b0;
            tx_incr_cnt = 1'b1;
            if(tx_cnt == 2'b11) begin
                tx_rst_cnt = 1'b1;
                uart_regs_n[`CTRL][`TX_EN]   = 1'b0;
                uart_regs_n[`CTRL][`TX_FULL] = 1'b0;
            end
        end
    end

    // UART RX
    if(!uart_regs_q[`CTRL][`RX_FULL]) begin
        if(rx_valid) begin
            uart_regs_n[`RX_DATA] = uart_regs_q[`RX_DATA] | ({{24{1'b0}}, rx_data} << (8*rx_cnt));
            rx_incr_cnt = 1'b1;
            if(rx_cnt == 2'b11) begin
                rx_rst_cnt = 1'b1;
                uart_regs_n[`CTRL][`RX_FULL] = 1'b1;
                uart_regs_n[`CTRL][`RX_ERR]  = rx_parity_err;
            end
        end
    end
end

always_ff @(posedge clk, negedge rstn)
begin
    if(!rstn) begin
        uart_regs_q[`CLK_DIV] <= 32'hffffffff;
        uart_regs_q[`CTRL]    <= 'b0;
        uart_regs_q[`TX_DATA] <= 'b0;
        uart_regs_q[`RX_DATA] <= 'b0;
    end else begin
        uart_regs_q[`CLK_DIV] <= uart_regs_n[`CLK_DIV];
        uart_regs_q[`CTRL]    <= uart_regs_n[`CTRL];
        uart_regs_q[`TX_DATA] <= uart_regs_n[`TX_DATA];
        uart_regs_q[`RX_DATA] <= uart_regs_n[`RX_DATA];

        // increment and reset TX counter
        if(tx_rst_cnt)
            tx_cnt <= 'b0;
        else if(tx_incr_cnt)
            tx_cnt <= tx_cnt + 1;

        // increment and reset RX counter
        if(rx_rst_cnt)
            rx_cnt <= 'b0;
        else if(rx_incr_cnt)
            rx_cnt <= rx_cnt + 1;
    end
end

uart_tx tx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn                      ),
    .clk_div_i  ( uart_regs_q[`CLK_DIV]     ),
    .tx_data_i  ( tx_data                   ),
    .tx_valid_i ( tx_enable                 ),
    .tx_done_o  ( tx_done                   ),
    .tx_o       ( tx_o                      )
);

uart_rx rx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn                      ),
    .clk_div_i  ( uart_regs_q[`CLK_DIV]     ),
    .rx_data_o  ( rx_data                   ),
    .rx_valid_o ( rx_valid                  ),
    .rx_err_o   ( rx_parity_err             ),
    .rx_i       ( rx_i                      )
);

endmodule