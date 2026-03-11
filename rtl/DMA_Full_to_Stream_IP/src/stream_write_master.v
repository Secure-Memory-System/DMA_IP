`timescale 1ns / 1ps

module stream_write_master #(
    parameter integer C_M_AXIS_DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire reset_n,

    input  wire        i_start,
    input  wire [31:0] i_total_len,
    output reg         o_done,

    input  wire        i_fifo_empty,
    output wire        o_fifo_rd_en,
    input  wire [31:0] i_w_data,

    output wire [C_M_AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tlast
);

    localparam IDLE   = 2'b01;
    localparam ACTIVE = 2'b10;
    localparam BYTES_PER_BEAT = C_M_AXIS_DATA_WIDTH / 8;
    localparam WORDS_PER_BEAT = C_M_AXIS_DATA_WIDTH / 32;

    reg [1:0]  current_state, next_state;
    reg [31:0] r_remaining_bytes;

    reg [C_M_AXIS_DATA_WIDTH-1:0] pack_reg;
    reg [$clog2(WORDS_PER_BEAT):0] pack_cnt;
    reg pack_valid;

    wire fifo_pop = (current_state == ACTIVE) && 
                   (!i_fifo_empty) && 
                   (!pack_valid || m_axis_tready);

    assign o_fifo_rd_en  = fifo_pop;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            pack_reg   <= 0;
            pack_cnt   <= 0;
            pack_valid <= 0;
        end else begin
            if (current_state == IDLE) begin
                pack_cnt   <= 0;
                pack_valid <= 0;
            end else if (fifo_pop) begin
                pack_reg[32*pack_cnt +: 32] <= i_w_data;

                if (pack_cnt == WORDS_PER_BEAT - 1) begin
                    pack_cnt   <= 0;
                    pack_valid <= 1;
                end else begin
                    pack_cnt   <= pack_cnt + 1;
                    pack_valid <= 0;
                end
            end else if (pack_valid && m_axis_tready) begin
                pack_valid <= 0;
            end
        end
    end

    assign m_axis_tdata  = pack_reg;
    assign m_axis_tvalid = pack_valid;
    assign m_axis_tlast  = pack_valid && (r_remaining_bytes <= BYTES_PER_BEAT);

    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE:   if (i_start) next_state = ACTIVE;
            ACTIVE: if (m_axis_tlast && m_axis_tready) next_state = IDLE;
            default: next_state = IDLE;
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            current_state     <= IDLE;
            r_remaining_bytes <= 0;
            o_done            <= 0;
        end else begin
            current_state <= next_state;
            o_done        <= 0;

            case (current_state)
                IDLE: begin
                    if (i_start)
                        r_remaining_bytes <= i_total_len;
                end
                ACTIVE: begin
                    if (m_axis_tvalid && m_axis_tready) begin
                        if (r_remaining_bytes <= BYTES_PER_BEAT) begin
                            r_remaining_bytes <= 0;
                            o_done            <= 1;
                        end else begin
                            r_remaining_bytes <= r_remaining_bytes - BYTES_PER_BEAT;
                        end
                    end
                end
            endcase
        end
    end

endmodule
