/*  This file is part of Darius_MiSTer.

    Darius_MiSTer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Darius_MiSTer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Darius_MiSTer.  If not, see <http://www.gnu.org/licenses/>.

    Author: Umberto Parisi (rmonic79)
    Version: 1.0
    Date: 2026

*/

// darius_fg_renderer — FG (text/HUD) layer renderer.
// 108×24 tilemap for score popups, INSERT COIN, attract text. Text ROM in
// local BRAM (zero SDRAM contention), read with 1 cycle of latency.
// Double line buffer for staggered prep/emit.
//
// Double buffer: display_buf_sel / prep_buf_sel (panel_renderer pattern).

module darius_fg_renderer (
	input  wire        clk,
	input  wire        reset,
	input  wire  [9:0] render_x,
	input  wire  [8:0] render_y,

	// FG RAM read (via Port B mux)
	output reg  [13:0] fg_ram_addr,
	input  wire [15:0] fg_ram_rdata,
	input  wire        fg_stall,

	// Text ROM download (from ioctl during ROM load)
	input  wire        dl_wr,        // write pulse
	input  wire [13:0] dl_addr,      // word address (0..16383)
	input  wire [15:0] dl_data,      // 16-bit interleaved word

	// Palette lookup
	output wire [10:0] pal_addr,
	input  wire [15:0] pal_data,

	// OSD adjustable offsets
	input  wire signed [9:0] fg_xoff, fg_yoff,

	// Pixel output
	output reg  [23:0] fg_rgb,
	output reg         fg_opaque
);

localparam H_ACTIVE = 10'd864;
localparam V_ACTIVE = 9'd224;
localparam Y_OFFSET = 9'd0;

// =====================================================================
// Text ROM in BRAM (16KB = 8192 × 16-bit).
// The original ROM file is 32KB but the upper half is 0xFF padding:
// fold bit 13 on download so only the lower half is stored in BRAM.
// =====================================================================
(* ramstyle = "M10K,no_rw_check" *) reg [15:0] text_rom [0:8191];

// Download write port: ignore writes to the upper (padding) half
always @(posedge clk) begin
	if (dl_wr && !dl_addr[13])
		text_rom[dl_addr[12:0]] <= dl_data;
end

// Read port (1-cycle latency)
reg [13:0] trom_rd_addr;
reg [15:0] trom_rd_data;
always @(posedge clk) begin
	trom_rd_data <= text_rom[trom_rd_addr[12:0]];
end

// =====================================================================
// Line buffers
// =====================================================================
(* ramstyle = "M10K,no_rw_check" *) reg [8:0] linebuf0 [0:1023];
(* ramstyle = "M10K,no_rw_check" *) reg [8:0] linebuf1 [0:1023];

reg       display_buf_sel;
reg       prep_buf_sel;

// =====================================================================
// Edge detection
// =====================================================================
reg [8:0] prev_render_y;
reg       prev_in_active;
wire      in_active = (render_y < V_ACTIVE);
wire      vblank_start = prev_in_active && !in_active;
wire      start_of_line = in_active && (render_y != prev_render_y);

// =====================================================================
// Display pipeline — timing fix 2026-04-08
// Critical path pre-fix: hc[] → render_x → linebuf → disp_raw (~11.3 ns)
// Fix: pre-register display address + prefetch +1 per compensare lo stadio
// extra (mimetico di darius_panel_renderer). Vedi docs/architecture/
// timing_fix_fg_pipeline.md per dettagli e criteri di verifica.
// =====================================================================
reg  [9:0] disp_rd_addr;
reg  [8:0] linebuf0_q, linebuf1_q;
reg        disp_valid_r0;

wire [9:0] disp_lookup_x = (render_x < (H_ACTIVE - 10'd1))
                            ? (render_x + 10'd1)
                            : (H_ACTIVE - 10'd1);

always @(posedge clk) begin
	// Stage 1: latch address + valid (rompe path combinatorio render_x → linebuf)
	disp_rd_addr  <= disp_lookup_x;
	disp_valid_r0 <= (render_x < H_ACTIVE) && in_active;
	// Stage 2: BRAM read with registered address
	linebuf0_q    <= linebuf0[disp_rd_addr];
	linebuf1_q    <= linebuf1[disp_rd_addr];
end

reg       disp_valid_r;
reg [8:0] disp_raw;
always @(posedge clk) begin
	// Stage 3: mux + latch (era lo stadio 1 originale)
	disp_valid_r <= disp_valid_r0;
	disp_raw     <= display_buf_sel ? linebuf1_q : linebuf0_q;
end

wire [8:0] disp_entry = disp_valid_r ? disp_raw : 9'd0;
wire [1:0] disp_pixel = disp_entry[1:0];
wire [6:0] disp_pal   = disp_entry[8:2];

assign pal_addr = {disp_pal, 2'b00, disp_pixel};

wire [4:0] pal_r = pal_data[4:0];
wire [4:0] pal_g = pal_data[9:5];
wire [4:0] pal_b = pal_data[14:10];

always @(posedge clk) begin
	fg_rgb <= {pal_r, pal_r[4:2], pal_g, pal_g[4:2], pal_b, pal_b[4:2]};
	fg_opaque <= (disp_pixel != 2'd0);
end

// =====================================================================
// FSM
// =====================================================================
localparam S_IDLE       = 4'd0;
localparam S_CLEAR      = 4'd1;
localparam S_READ_ATTR  = 4'd2;
localparam S_WAIT_ATTR  = 4'd3;
localparam S_LATCH_ATTR = 4'd4;
localparam S_READ_CODE  = 4'd5;
localparam S_WAIT_CODE  = 4'd6;
localparam S_LATCH_CODE = 4'd7;
localparam S_REQ_ROM    = 4'd8;  // set BRAM address
localparam S_WAIT_ROM   = 4'd9;  // allow BRAM output register to update
localparam S_LATCH_ROM  = 4'd10; // latch stable BRAM word
localparam S_DRAW       = 4'd11;
localparam S_NEXT_TILE  = 4'd12;

reg [3:0]  state;
reg [9:0]  clear_x;
reg [8:0]  prep_y;
reg [8:0]  prep_line_y;
reg        prep_done;
reg [6:0]  tile_col;
reg [15:0] cur_attr;
reg [15:0] cur_code;
reg [15:0] cur_romword;   // 16-bit tile pixel word from text_rom
reg        cur_flipx, cur_flipy;
reg [6:0]  cur_color;
reg [2:0]  draw_pix;


wire [5:0] tile_row = (prep_y + Y_OFFSET + fg_yoff) >> 3;
wire [2:0] fine_y   = (prep_y + Y_OFFSET + fg_yoff) & 3'b111;
wire [12:0] scan_tile_index = {tile_row, tile_col};

// Text ROM word address: code * 8 + fine_y
wire [13:0] rom_word_addr = {cur_code[10:0], fine_y ^ {3{cur_flipy}}};

function automatic [1:0] fg_pixel;
	input [15:0] word;
	input        flipx;
	input [2:0]  pix_idx;
	reg [2:0] bp;
	reg p0, p1;
	begin
		bp = flipx ? pix_idx : (3'd7 - pix_idx);
		p0 = word[bp];
		p1 = word[8 + bp];
		fg_pixel = {p1, p0};
	end
endfunction

reg [8:0] next_prep_y;

always @(posedge clk) begin
	if (reset) begin
		state           <= S_IDLE;
		display_buf_sel <= 1'b0;
		prep_buf_sel    <= 1'b1;
		prep_done       <= 1'b0;
		prep_line_y     <= 9'h1FF;
		prev_render_y   <= 9'h1FF;
		prev_in_active  <= 1'b0;
		next_prep_y     <= 9'd0;
	end else begin
		prev_in_active <= in_active;
		if (start_of_line)
			prev_render_y <= render_y;

		// Buffer swap at start_of_line if prep finished AND correct line
		if (start_of_line && prep_done && (prep_line_y == render_y)) begin
			display_buf_sel <= prep_buf_sel;
			prep_buf_sel    <= ~prep_buf_sel;
			prep_done       <= 1'b0;
			next_prep_y     <= render_y + 9'd1;
		end

		// VBlank: prepare line 0 for next frame
		if (vblank_start) begin
			next_prep_y <= 9'd0;
			prep_done   <= 1'b0;
		end

		if (!fg_stall) begin

		case (state)
			S_IDLE: begin
				if (!prep_done && next_prep_y < V_ACTIVE) begin
					prep_y   <= next_prep_y;
					clear_x  <= 10'd0;
					state    <= S_CLEAR;
				end
			end

			S_CLEAR: begin
				if (prep_buf_sel)
					linebuf1[clear_x] <= 9'd0;
				else
					linebuf0[clear_x] <= 9'd0;

				if (clear_x == H_ACTIVE - 10'd1) begin
					tile_col <= 7'd0;
					state    <= S_READ_ATTR;
				end else
					clear_x <= clear_x + 10'd1;
			end

			S_READ_ATTR: begin
				fg_ram_addr <= {1'b0, scan_tile_index};
				state <= S_WAIT_ATTR;
			end

			S_WAIT_ATTR: state <= S_LATCH_ATTR;

			S_LATCH_ATTR: begin
				cur_attr  <= fg_ram_rdata;
				cur_color <= fg_ram_rdata[6:0];
				cur_flipx <= fg_ram_rdata[14];
				cur_flipy <= fg_ram_rdata[15];
				fg_ram_addr <= {1'b1, scan_tile_index};
				state <= S_READ_CODE;
			end

			S_READ_CODE: state <= S_WAIT_CODE;

			S_WAIT_CODE: state <= S_LATCH_CODE;

			S_LATCH_CODE: begin
				cur_code <= fg_ram_rdata;
				if ((fg_ram_rdata & 16'h07FF) == 16'd0)
					state <= S_NEXT_TILE;
				else
					state <= S_REQ_ROM;
			end

			// BRAM text ROM read: set address, wait 1 cycle
			S_REQ_ROM: begin
				trom_rd_addr <= rom_word_addr;
				state <= S_WAIT_ROM;
			end

			S_WAIT_ROM: begin
				// Give the BRAM output register a full cycle to update
				state <= S_LATCH_ROM;
			end

			S_LATCH_ROM: begin
				cur_romword <= trom_rd_data;
				draw_pix <= 3'd0;
				state <= S_DRAW;
			end

			S_DRAW: begin
				reg [1:0] pixel;
				reg [9:0] draw_x;
				pixel = fg_pixel(cur_romword, cur_flipx, draw_pix);
				draw_x = {tile_col, 3'd0} + {7'd0, draw_pix} + fg_xoff;

				if (draw_x < H_ACTIVE && pixel != 2'd0) begin
					if (prep_buf_sel)
						linebuf1[draw_x] <= {cur_color, pixel};
					else
						linebuf0[draw_x] <= {cur_color, pixel};
				end

				if (draw_pix == 3'd7)
					state <= S_NEXT_TILE;
				else
					draw_pix <= draw_pix + 3'd1;
			end

			S_NEXT_TILE: begin
				if (tile_col == 7'd107) begin
					prep_done    <= 1'b1;
					prep_line_y  <= prep_y;
					state        <= S_IDLE;
				end else begin
					tile_col <= tile_col + 7'd1;
					state <= S_READ_ATTR;
				end
			end

			default: state <= S_IDLE;
		endcase
		end // if (!fg_stall)
	end
end

endmodule
