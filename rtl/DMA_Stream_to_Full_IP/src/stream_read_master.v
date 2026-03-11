`timescale 1ns / 1ps

/**
 * stream_read_master
 * 기능: AXI4-Stream Slave로 데이터를 수신하여 내부 FIFO로 전달함.
 */
module stream_read_master #(
    parameter integer C_S_AXIS_DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire reset_n,

    input  wire        i_start,
    input  wire [31:0] i_total_len,
    output reg         o_done,

    input  wire        i_fifo_full,
    output wire        o_fifo_push,
    output wire [C_S_AXIS_DATA_WIDTH-1:0] o_r_data,

    input  wire [C_S_AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast
);

    localparam IDLE   = 2'b01;
    localparam ACTIVE = 2'b10;

    reg [1:0] current_state, next_state;

    localparam BYTES_PER_BEAT = C_S_AXIS_DATA_WIDTH / 8;

    reg [31:0] r_remaining_bytes;

    assign s_axis_tready = (current_state == ACTIVE) && (!i_fifo_full);
    assign o_fifo_push   = s_axis_tvalid && s_axis_tready;
    assign o_r_data      = s_axis_tdata;

    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (i_start) next_state = ACTIVE;
            end
            ACTIVE: begin
                if (s_axis_tvalid && s_axis_tready) begin
                    if (s_axis_tlast || (r_remaining_bytes <= BYTES_PER_BEAT))
                        next_state = IDLE;
                end
            end
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
                    if (i_start) begin
                        r_remaining_bytes <= i_total_len;
                    end
                end
                ACTIVE: begin
                    if (s_axis_tvalid && s_axis_tready) begin
                        if (s_axis_tlast || (r_remaining_bytes <= BYTES_PER_BEAT)) begin
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
