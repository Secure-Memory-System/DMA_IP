`timescale 1ns / 1ps

/**
 * Read_Master Module
 * 기능: AXI4-Full 프로토콜을 사용하여 외부 메모리로부터 데이터를 읽어 내부 FIFO로 전달함.
 * 특징: 4KB Boundary 제약 조건을 자동으로 계산하여 Burst를 분할 전송함.
 */
module read_master # (
    parameter integer C_M_AXI_ID_WIDTH   = 1,
    parameter integer C_M_AXI_ADDR_WIDTH = 32,
    parameter integer C_M_AXI_DATA_WIDTH = 32
)(
    input wire clk,
    input wire reset_n,

    // --- 사용자 제어 신호 (User Control) ---
    input wire i_start,
    input wire [31:0] i_src_addr,
    input wire [31:0] i_total_len,
    output reg o_read_done,

    // --- FIFO 인터페이스 ---
    input wire i_fifo_full,
    output wire o_fifo_push,
    output wire [31:0] o_r_data,

    // --- AXI4-Full: Read Address Channel (AR) ---
    output wire [C_M_AXI_ADDR_WIDTH-1 : 0] m_axi_araddr,
    output wire [7 : 0] m_axi_arlen,
    output wire [2 : 0] m_axi_arsize,
    output wire [1 : 0] m_axi_arburst,
    output wire m_axi_arvalid,
    input  wire m_axi_arready,

    // --- AXI4-Full: Read Data Channel (R) ---
    input  wire [C_M_AXI_DATA_WIDTH-1 : 0] m_axi_rdata,
    input  wire m_axi_rlast,
    input  wire m_axi_rvalid,
    output wire m_axi_rready
);

    localparam IDLE       = 3'b001;
    localparam ADDR_PHASE = 3'b010;
    localparam DATA_PHASE = 3'b100;

    reg [2:0] current_state, next_state;

    reg [31:0] r_current_addr;
    reg [31:0] r_remaining_bytes;
    reg [7:0]  r_burst_len;
    reg        arvalid_reg;

    wire [31:0] next_boundary_addr;
    wire [31:0] dist_to_boundary;
    wire [31:0] max_burst_bytes;
    wire [31:0] calc_len_bytes;
    wire [31:0] current_transfer_bytes;

    assign next_boundary_addr = (r_current_addr & 32'hFFFF_F000) + 32'h1000;
    assign dist_to_boundary   = next_boundary_addr - r_current_addr;
    assign max_burst_bytes    = (r_remaining_bytes > 64) ? 64 : r_remaining_bytes;
    assign calc_len_bytes     = (max_burst_bytes > dist_to_boundary) ? dist_to_boundary : max_burst_bytes;
    assign current_transfer_bytes = {22'd0, r_burst_len, 2'b00};

    assign m_axi_arsize  = 3'b010;
    assign m_axi_arburst = 2'b01;
    assign m_axi_araddr  = r_current_addr;
    assign m_axi_arvalid = arvalid_reg; 
    assign m_axi_arlen   = (calc_len_bytes[9:2] > 0) ? (calc_len_bytes[9:2] - 1) : 0;
    assign m_axi_rready  = (current_state == DATA_PHASE) && (!i_fifo_full);

    assign o_fifo_push = (m_axi_rvalid && m_axi_rready);
    assign o_r_data    = m_axi_rdata;

    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (i_start) next_state = ADDR_PHASE;
            end
            ADDR_PHASE: begin
                if (m_axi_arvalid && m_axi_arready) 
                    next_state = DATA_PHASE;
            end
            DATA_PHASE: begin
                if (m_axi_rlast && m_axi_rvalid && m_axi_rready) begin
                    if (r_remaining_bytes <= current_transfer_bytes)
                        next_state = IDLE;
                    else
                        next_state = ADDR_PHASE;
                end
            end
            default: next_state = IDLE;
        endcase
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n)
            arvalid_reg <= 0;
        else begin
            case (current_state)
                IDLE: begin
                    if (i_start) arvalid_reg <= 1;
                    else         arvalid_reg <= 0;
                end
                ADDR_PHASE: begin
                    if (arvalid_reg && m_axi_arready)
                        arvalid_reg <= 0;
                end
                DATA_PHASE: begin
                    if (m_axi_rlast && m_axi_rvalid && m_axi_rready) begin
                        if (r_remaining_bytes > current_transfer_bytes)
                            arvalid_reg <= 1;
                        else
                            arvalid_reg <= 0;
                    end
                end
                default: arvalid_reg <= 0;
            endcase
        end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            current_state     <= IDLE;
            r_current_addr    <= 0;
            r_remaining_bytes <= 0;
            r_burst_len       <= 0;
            o_read_done       <= 0;
        end else begin
            current_state <= next_state;

            case (current_state)
                IDLE: begin
                    if (i_start) begin
                        o_read_done <= 0;
                        r_current_addr    <= i_src_addr;
                        r_remaining_bytes <= i_total_len;
                    end
                end
                ADDR_PHASE: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        r_burst_len <= calc_len_bytes[9:2];
                    end
                end
                DATA_PHASE: begin
                    if (m_axi_rlast && m_axi_rvalid && m_axi_rready) begin
                        r_current_addr <= r_current_addr + current_transfer_bytes;
                        if (r_remaining_bytes > current_transfer_bytes)
                            r_remaining_bytes <= r_remaining_bytes - current_transfer_bytes;
                        else begin
                            r_remaining_bytes <= 0;
                            o_read_done       <= 1;
                        end
                    end
                end
            endcase
        end
    end
endmodule
