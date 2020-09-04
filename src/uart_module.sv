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

module uart_module #(
)(
    apb_bus_t.slave     apb_bus,
    output logic        irq_o,
    input logic         rx_i,
    output logic        tx_o
);

// uart registers
localparam TX_DATA = 0;
localparam RX_DATA = 1;
localparam CLK_DIV = 2;
localparam CTRL = 3;
localparam STATUS = 4;

// control bits
localparam TX_EN = 0;
localparam RX_EN = 1;
localparam PARITY_EN = 2;

// Status bits
localparam TX_EMPTY = 0;
localparam TX_FULL = 1;
localparam RX_EMPTY = 2;
localparam RX_FULL = 3;
localparam RX_ERR = 4;

logic clk   = apb_bus.PCLK;
logic rstn  = apb_bus.PRESETn;

// APB
logic           PREADY;
logic           PSLVERR;
logic [31:0]    PRDATA;

// registers
logic [31:0] uart_regs_n[4:0];
logic [31:0] uart_regs_q[4:0];

// TX signals
logic [7:0]     tx_data;
logic           tx_done;
logic           tx_enable;

// RX signals
logic [7:0]     rx_data;
logic           rx_valid;
logic           rx_parity_err;

// fifo signals
logic           rx_fifo_read;
logic           rx_fifo_write_n, rx_fifo_write_q;
logic [7:0]     rx_data_read;
logic           rx_empty, rx_full;
logic           tx_fifo_read;
logic           tx_fifo_write_n, tx_fifo_write_q;
logic           tx_empty, tx_full;

assign apb_bus.PRDATA = PRDATA;
assign apb_bus.PREADY = PREADY;
assign apb_bus.PSLVERR = PSLVERR;

// send interrupt request if the data register is not empty
assign irq_o = !uart_regs_q[STATUS][RX_EMPTY];

always_comb
begin
    PREADY = 1'b0;
    PRDATA = 'b0;
    PSLVERR = 1'b0;
    uart_regs_n = uart_regs_q;

    // UART tx
    tx_enable   = 1'b0;

    // tx fifo
    tx_fifo_read  = 1'b0;
    tx_fifo_write_n = 1'b0;
    uart_regs_n[STATUS][TX_EMPTY] = tx_empty;
    uart_regs_n[STATUS][TX_FULL] = tx_full;

    // rx fifo
    rx_fifo_read = 1'b0;
    rx_fifo_write_n = 1'b0;
    uart_regs_n[STATUS][RX_EMPTY] = rx_empty;
    uart_regs_n[STATUS][RX_FULL] = rx_full;
    uart_regs_n[RX_DATA] = {24'b0, rx_data_read};

    // APB slave
    if(apb_bus.PSEL && apb_bus.PENABLE) begin
        PREADY = 1'b1;
        // Write
        if(apb_bus.PWRITE)
            // only ctrl and tx are writeable, else error
            case(apb_bus.PADDR[4:2])
                TX_DATA: begin
                    uart_regs_n[apb_bus.PADDR[4:2]] = apb_bus.PWDATA;
                    // notifiy fifo of write
                    tx_fifo_write_n = 1'b1;
                end
                CTRL, CLK_DIV: 
                    uart_regs_n[apb_bus.PADDR[4:2]] = apb_bus.PWDATA;                
                default:
                    PSLVERR = 1'b1;
            endcase
        // Read
        else begin
            PRDATA = uart_regs_q[apb_bus.PADDR[4:2]];
            if(apb_bus.PADDR[4:2] == RX_DATA) begin
                // notify fifo that we read a value
                rx_fifo_read = 1'b1;
            end
        end
    end

    // UART TX
    if(uart_regs_q[CTRL][TX_EN] && (!uart_regs_q[STATUS][TX_EMPTY])) begin
        tx_enable = 1'b1;
        if(tx_done) begin
            tx_enable = 1'b0;
            // notify fifo that we processed the data
            tx_fifo_read = 1'b1;
        end
    end

    // UART RX
    if((!uart_regs_q[STATUS][RX_FULL]) && uart_regs_q[CTRL][RX_EN]) begin
        if(rx_valid) begin
            rx_fifo_write_n = 1'b1;
        end
    end
end

always_ff @(posedge clk, negedge rstn)
begin
    if(!rstn) begin
        uart_regs_q[CLK_DIV] <= 32'hffffffff;
        uart_regs_q[CTRL]    <= 'b0;
        uart_regs_q[STATUS]  <= 'b0;
        uart_regs_q[TX_DATA] <= 'b0;
        uart_regs_q[RX_DATA] <= 'b0;
    end else begin
        uart_regs_q[CLK_DIV] <= uart_regs_n[CLK_DIV];
        uart_regs_q[CTRL]    <= uart_regs_n[CTRL];
        uart_regs_q[STATUS]  <= uart_regs_n[STATUS];
        uart_regs_q[TX_DATA] <= uart_regs_n[TX_DATA];
        uart_regs_q[RX_DATA] <= uart_regs_n[RX_DATA];

        // Fifos
        tx_fifo_write_q <= tx_fifo_write_n;
        rx_fifo_write_q <= rx_fifo_write_n;
    end
end

/* verilator lint_off PINMISSING */
/* verilator lint_off PINCONNECTEMPTY */
fifo #(
    .WIDTH      ( 8         ),
    .DEPTH      ( 32        ),
    .ALMOST_EN  ( 0         )
) rx_fifo (
    .clk_i      ( clk               ),
    .rstn_i     ( rstn              ),
    .din_i      ( rx_data           ),
    .we_i       ( rx_fifo_write_q   ),
    .re_i       ( rx_fifo_read      ),
    .dout_o     ( rx_data_read      ),
    .E_o        ( rx_empty          ),
    .F_o        ( rx_full           ),
    .AE_o       (),
    .AF_o       ()
);

fifo #(
    .WIDTH      ( 8         ),
    .DEPTH      ( 32        ),
    .ALMOST_EN  ( 0         )
) tx_fifo (
    .clk_i      ( clk                     ),
    .rstn_i     ( rstn                    ),
    .din_i      ( uart_regs_q[TX_DATA][7:0]),
    .we_i       ( tx_fifo_write_q         ),
    .re_i       ( tx_fifo_read            ),
    .dout_o     ( tx_data                 ),
    .E_o        ( tx_empty                ),
    .F_o        ( tx_full                 ),
    .AE_o       (),
    .AF_o       ()
);
/* verilator lint_on PINCONNECTEMPTY */
/* verilator lint_on PINMISSING */

uart_tx tx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn                      ),
    .clk_div_i  ( uart_regs_q[CLK_DIV]      ),
    .tx_data_i  ( tx_data                   ),
    .tx_valid_i ( tx_enable                 ),
    .tx_done_o  ( tx_done                   ),
    .parity_en_i( uart_regs_q[CTRL][PARITY_EN]),
    .tx_o       ( tx_o                      )
);

uart_rx rx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn                      ),
    .clk_div_i  ( uart_regs_q[CLK_DIV]      ),
    .rx_enable_i( uart_regs_q[CTRL][RX_EN]  ),
    .rx_data_o  ( rx_data                   ),
    .rx_valid_o ( rx_valid                  ),
    .rx_err_o   ( rx_parity_err             ),
    .parity_en_i( uart_regs_q[CTRL][PARITY_EN]),
    .rx_i       ( rx_i                      )
);

endmodule