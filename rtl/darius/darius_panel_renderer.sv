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

// darius_panel_renderer — Tile renderer parametrico single-panel.
// Istanziato 3 volte in parallelo (L/C/R) per comporre lo schermo virtuale
// a 864 pixel tipico di Darius. Ogni instance renderizza H_PIXELS (288)
// pixel a partire da X_OFFSET. VRAM e palette replicate per instance
// (CPU writes fan out from parent).
//
// Double-width PC080SN layout:
//   VRAM $0000-$1FFF: attributes (color[8:0], flip[15:14])
//   VRAM $2000-$3FFF: tile codes ([13:0])
//   Tilemap: 128x64 tiles, 8x8 pixels each

module darius_panel_renderer
#(
	parameter [9:0] X_OFFSET = 10'd0,
	parameter [9:0] H_PIXELS = 10'd288,
	parameter       SS_IDX   = -1      // savestate: indice slave palette (pattern F2)
)
(
	input  wire        clk,
	input  wire        reset,

	// Savestate (pattern F2: ssbus dentro al chip) — palette locale
	ssbus_if.slave     ssbus,
	// Savestate: load registri scroll/ctrl al restore (da auto_save_adaptor nel top)
	input  wire        ss_regs_ld,
	input  wire [15:0] ss_xs0, ss_xs1, ss_ys0, ss_ys1, ss_c0, ss_c1,
	input  wire  [9:0] render_x,
	input  wire  [8:0] render_y,

	// CPU bus sniff (scroll/ctrl registers only)
	input  wire [23:0] cpu_bus_addr,
	input  wire        cpu_bus_asn,
	input  wire        cpu_bus_rnw,
	input  wire  [1:0] cpu_bus_dsn,
	input  wire [15:0] cpu_bus_wdata,
	input  wire        cpu_bus_cs,
	// Sub CPU palette write (direct from subcpu_map)
	input  wire        sub_pal_wr,
	input  wire [10:0] sub_pal_addr,
	input  wire [15:0] sub_pal_wdata,
	input  wire  [1:0] sub_pal_be,

	// Shared VRAM read (via arbiter)
	output reg  [14:0] vram_rd_addr,
	output reg         vram_rd_req,     // pulse when new address set
	input  wire [15:0] vram_rd_data,
	input  wire        vram_rd_valid,

	// Tile ROM via arbiter
	input  wire [31:0] tilerom_data,
	input  wire        tilerom_valid,
	output reg  [23:0] tilerom_addr,
	output reg         tilerom_req,

	// Scroll outputs (directly from registers, active on all instances)
	output wire [15:0] xscroll_l0, xscroll_l1,
	output wire [15:0] yscroll_l0, yscroll_l1,
	output wire [15:0] ctrl_l0, ctrl_l1,

	// Pixel output
	output wire [23:0] tile_rgb,
	output wire  [1:0] tile_prio,
	output wire        tile_opaque,

	// OSD adjustable offsets per layer
	input  wire signed [9:0] l0_xoff, l0_yoff,
	input  wire signed [9:0] l1_xoff, l1_yoff,

	// Debug: coordinate mapping probe (reuses existing outputs)
	output reg  [15:0] dbg_tile_code,
	output reg  [15:0] dbg_tile_attr,
	output reg  [31:0] dbg_tile_romdata
);

// =====================================================================
// CPU bus write detection
// =====================================================================
// VRAM writes handled externally (shared VRAM in dual68k_top)
wire yscroll_sel = (cpu_bus_addr >= 24'hD20000) && (cpu_bus_addr <= 24'hD20003);
wire xscroll_sel = (cpu_bus_addr >= 24'hD40000) && (cpu_bus_addr <= 24'hD40003);
wire ctrl_sel    = (cpu_bus_addr >= 24'hD50000) && (cpu_bus_addr <= 24'hD50003);
wire palette_sel = (cpu_bus_addr >= 24'hD80000) && (cpu_bus_addr <= 24'hD80FFF);

wire write_active = ~cpu_bus_asn && ~cpu_bus_rnw && cpu_bus_cs && (cpu_bus_dsn != 2'b11);
reg write_seen;
always @(posedge clk) begin
	if (reset) write_seen <= 1'b0;
	else if (!write_active) write_seen <= 1'b0;
	else write_seen <= 1'b1;
end
wire wr_pulse = write_active && !write_seen;

// =====================================================================
// Scroll / control registers
// =====================================================================
reg [15:0] xscroll [0:1];
reg [15:0] yscroll [0:1];
reg [15:0] ctrl    [0:1];
wire layer_sel = cpu_bus_addr[1];

always @(posedge clk) begin
	if (reset) begin
		xscroll[0] <= 0; xscroll[1] <= 0;
		yscroll[0] <= 0; yscroll[1] <= 0;
		ctrl[0]    <= 0; ctrl[1]    <= 0;
	end else if (ss_regs_ld) begin
		// restore savestate: ricarica i registri (CPU ferme)
		xscroll[0] <= ss_xs0; xscroll[1] <= ss_xs1;
		yscroll[0] <= ss_ys0; yscroll[1] <= ss_ys1;
		ctrl[0]    <= ss_c0;  ctrl[1]    <= ss_c1;
	end else if (wr_pulse) begin
		if (xscroll_sel) xscroll[layer_sel] <= cpu_bus_wdata;
		if (yscroll_sel) yscroll[layer_sel] <= cpu_bus_wdata;
		if (ctrl_sel)    ctrl[layer_sel]    <= cpu_bus_wdata;
	end
end
assign xscroll_l0 = xscroll[0]; assign xscroll_l1 = xscroll[1];
assign yscroll_l0 = yscroll[0]; assign yscroll_l1 = yscroll[1];
assign ctrl_l0 = ctrl[0]; assign ctrl_l1 = ctrl[1];

// =====================================================================
// VRAM — external shared (via arbiter), no local copy
// =====================================================================
// vram_rd_addr, vram_rd_data, vram_rd_valid come from ports above

// =====================================================================
// Palette RAM
// =====================================================================
wire pal_wr = wr_pulse && palette_sel;
wire [10:0] pal_cpu_addr = cpu_bus_addr[11:1];
wire  [1:0] pal_cpu_be = ~cpu_bus_dsn;

(* ramstyle = "M10K,no_rw_check", ram_init_file = "rtl/darius/pal_init_hi.mif" *) reg [7:0] pal_ram_hi [0:2047];
(* ramstyle = "M10K,no_rw_check", ram_init_file = "rtl/darius/pal_init_lo.mif" *) reg [7:0] pal_ram_lo [0:2047];
reg [15:0] pal_data;
reg [10:0] pal_lookup_addr;

// Savestate: adaptor sulla gamba SUB (durante SS le CPU sono ferme, la gamba
// è libera); gather-read sull'indirizzo muxato della porta di scrittura
// (stessa porta fisica), lookup renderer sull'altra porta. 2 porte = M10K TDP.
wire        sspl_we_lo, sspl_we_hi;
wire [10:0] sspl_addr;
wire [15:0] sspl_wdata;
reg  [15:0] pal_ss_q;
ss_ram16_adaptor #(.WIDTHAD(11), .SS_IDX(SS_IDX)) u_ss_pal (
	.clk(clk),
	.we_lo_in(sub_pal_wr & sub_pal_be[0]),
	.we_hi_in(sub_pal_wr & sub_pal_be[1]),
	.addr_in(sub_pal_addr),
	.wdata_in(sub_pal_wdata),
	.we_lo_out(sspl_we_lo), .we_hi_out(sspl_we_hi),
	.addr_out(sspl_addr), .wdata_out(sspl_wdata),
	.q_in(pal_ss_q),
	.ssbus(ssbus)
);

wire [10:0] pal_b_addr  = pal_wr ? pal_cpu_addr   : sspl_addr;
wire [15:0] pal_b_wdata = pal_wr ? cpu_bus_wdata  : sspl_wdata;
wire        pal_b_we_hi = pal_wr ? pal_cpu_be[1]  : sspl_we_hi;
wire        pal_b_we_lo = pal_wr ? pal_cpu_be[0]  : sspl_we_lo;

always @(posedge clk) begin
	if (pal_b_we_hi) pal_ram_hi[pal_b_addr] <= pal_b_wdata[15:8];
	if (pal_b_we_lo) pal_ram_lo[pal_b_addr] <= pal_b_wdata[7:0];
	pal_ss_q <= {pal_ram_hi[pal_b_addr], pal_ram_lo[pal_b_addr]};
	pal_data <= {pal_ram_hi[pal_lookup_addr], pal_ram_lo[pal_lookup_addr]};
end

// =====================================================================
// Line buffer renderer (prefetch from SDRAM)
// =====================================================================
localparam [9:0] LAST_PIXEL = H_PIXELS - 10'd1;
localparam V_ACTIVE = 9'd224;

// Panel-local coordinates
wire [9:0] local_x = render_x - X_OFFSET;
wire in_my_panel = (render_x >= X_OFFSET) && (render_x < (X_OFFSET + H_PIXELS));
wire in_active_area = in_my_panel && (render_y < V_ACTIVE);

reg  in_active_area_r;
reg  [8:0] prev_render_y;
wire start_of_line = in_active_area && (render_y != prev_render_y);

// Double-buffered line buffers: 14-bit entries {prio[1:0], opaque, pal_bank[6:0], pixel[3:0]}
// 512 entries (next power-of-2 above 288)
(* ramstyle = "no_rw_check" *) reg [13:0] linebuf0 [0:511];
(* ramstyle = "no_rw_check" *) reg [13:0] linebuf1 [0:511];
(* ramstyle = "no_rw_check" *) reg [13:0] linebuf0_q, linebuf1_q;

reg        display_buf_sel;
reg        display_line_valid;
reg [13:0] display_word;
reg  [8:0] display_rd_addr;
// Prefetch next pixel; last pixel wraps to itself (already displayed)
wire [8:0] display_lookup_x = (local_x[8:0] < LAST_PIXEL[8:0]) ? (local_x[8:0] + 9'd1) : LAST_PIXEL[8:0];

always @(posedge clk) begin
	linebuf0_q <= linebuf0[display_rd_addr];
	linebuf1_q <= linebuf1[display_rd_addr];
end

always @(*) begin
	if (!display_line_valid || !in_active_area)
		display_word = 14'd0;
	else if (display_buf_sel)
		display_word = linebuf1_q;
	else
		display_word = linebuf0_q;
end

// Prep state machine
reg        prep_buf_sel;
reg        prep_busy;
reg        prep_line_ready;
reg  [8:0] prep_line_y;
reg  [9:0] prep_out_x;
reg  [2:0] prep_fine_y;
reg  [2:0] prep_pix_idx;
reg [15:0] prep_attr;
reg [13:0] prep_code;
reg  [6:0] prep_pal_bank;
reg        prep_hflip;
reg [31:0] prep_tile_row;
reg        prep_linebuf_we;
reg  [8:0] prep_linebuf_addr;
reg [13:0] prep_linebuf_data;
reg        prep_layer;  // 0=Layer0 (background), 1=Layer1 (foreground)
wire [8:0] prep_next_line_y = (prep_line_y >= (V_ACTIVE - 9'd1)) ? 9'd0 : (prep_line_y + 9'd1);

// Latched scroll + ctrl per layer — captured at start of each line prep (like HBlank latch in real PC080SN)
reg  [9:0] prep_xscroll_lat  [0:1];
reg  [8:0] prep_yscroll_lat  [0:1];
reg        prep_ctrl_bit0_lat [0:1];

// Source pixel coordinates with scroll + panel offset (selects by current layer)
// MAME set_offsets(-16, 8): L0 xoff=-16, L1 xoff=+8
// MAME Y: L0 +8, L1 0
wire signed [9:0] layer_xoff_hw = (prep_layer == 1'b0) ? 10'sd0 : 10'sd0;
wire signed [8:0] layer_yoff_hw = (prep_layer == 1'b0) ?   9'sd0  : 9'sd0;
wire signed [9:0] layer_xoff = 10'sd0;
wire signed [9:0] layer_yoff = 10'sd0;
wire [9:0] prep_src_x = prep_out_x + X_OFFSET - prep_xscroll_lat[prep_layer] + layer_xoff_hw + layer_xoff;
wire [8:0] prep_src_y = prep_line_y - prep_yscroll_lat[prep_layer] + layer_yoff_hw + layer_yoff[8:0];
wire [12:0] prep_tile_index = {prep_src_y[8:3], prep_src_x[9:3]};
// VRAM address prefix: L0 attr=00/code=01, L1 attr=10/code=11
wire [1:0] vram_layer_attr = prep_layer ? 2'b10 : 2'b00;
wire [1:0] vram_layer_code = prep_layer ? 2'b11 : 2'b01;

// Tile pixel extraction (packed MSB: 2 pixels per byte, MSB first)
function automatic [3:0] get_pixel;
	input [31:0] row_data;
	input        hflip;
	input [2:0]  pix_idx;
	reg [4:0] shift;
	begin
		shift = hflip ? {pix_idx, 2'b00} : {(3'd7 - pix_idx), 2'b00};
		get_pixel = (row_data >> shift) & 4'hF;
	end
endfunction

// Prep pipeline states
localparam P_FETCH_TILE = 4'd0;
localparam P_WAIT_ATTR  = 4'd1;
localparam P_FETCH_CODE = 4'd2;
localparam P_WAIT_CODE  = 4'd3;
localparam P_READ_VRAM  = 4'd4;
localparam P_FETCH_ROM  = 4'd5;
localparam P_WAIT_ROM   = 4'd6;
localparam P_WRITE_PIX  = 4'd7;

reg [3:0] pipe_state;

always @(posedge clk) begin
	prep_linebuf_we <= 1'b0;
	vram_rd_req <= 1'b0;  // default: pulse off

	if (reset) begin
		vram_rd_req       <= 0;
		in_active_area_r  <= 0;
		prev_render_y     <= 9'h1FF;
		display_buf_sel   <= 0;
		display_line_valid <= 0;
		display_rd_addr   <= 0;
		prep_buf_sel      <= 1;
		prep_busy         <= 1;
		prep_line_ready   <= 0;
		prep_line_y       <= 0;
		prep_out_x        <= 0;
		pipe_state        <= P_FETCH_TILE;
		tilerom_req       <= 0;
		tilerom_addr      <= 0;
		prep_tile_row     <= 0;
		prep_attr         <= 0;
		prep_code         <= 0;
		prep_pal_bank     <= 0;
		prep_hflip        <= 0;
		prep_pix_idx      <= 0;
		prep_fine_y       <= 0;
		prep_layer        <= 0;
		prep_xscroll_lat[0]  <= 0; prep_xscroll_lat[1]  <= 0;
		prep_yscroll_lat[0]  <= 0; prep_yscroll_lat[1]  <= 0;
		prep_ctrl_bit0_lat[0] <= 0; prep_ctrl_bit0_lat[1] <= 0;
	end else begin
		in_active_area_r <= in_active_area;
		if (start_of_line) prev_render_y <= render_y;

		if (in_active_area)
			display_rd_addr <= display_lookup_x;
		else
			display_rd_addr <= 0;

		// Start of visible line: swap buffers if prep matches display
		if (start_of_line) begin
			// Latch scroll + ctrl for both layers at line boundary (matches real PC080SN HBlank latch)
			prep_xscroll_lat[0]   <= xscroll[0][9:0]; prep_xscroll_lat[1]   <= xscroll[1][9:0];
			prep_yscroll_lat[0]   <= yscroll[0][8:0]; prep_yscroll_lat[1]   <= yscroll[1][8:0];
			prep_ctrl_bit0_lat[0] <= ctrl[0][0];      prep_ctrl_bit0_lat[1] <= ctrl[1][0];
			if (prep_line_ready && (prep_line_y == render_y)) begin
				display_buf_sel    <= prep_buf_sel;
				display_line_valid <= 1'b1;
				prep_buf_sel       <= ~prep_buf_sel;
				prep_line_y        <= prep_next_line_y;
				prep_out_x         <= 0;
				prep_busy          <= 1;
				prep_line_ready    <= 0;
				prep_layer         <= 0;
				pipe_state         <= P_FETCH_TILE;
			end else begin
				display_line_valid <= 1'b0;
				prep_line_y        <= (render_y >= (V_ACTIVE - 9'd1)) ? 9'd0 : (render_y + 9'd1);
				prep_out_x         <= 0;
				prep_busy          <= 1;
				prep_line_ready    <= 0;
				prep_layer         <= 0;
				pipe_state         <= P_FETCH_TILE;
				tilerom_req        <= 0;
			end
		end

		// Prep pipeline
		if (prep_busy) begin
			case (pipe_state)
				P_FETCH_TILE: begin
					prep_fine_y <= prep_src_y[2:0];
					prep_pix_idx <= prep_src_x[2:0];
					vram_rd_addr <= {vram_layer_attr, prep_tile_index};
					vram_rd_req <= 1'b1;
					pipe_state <= P_WAIT_ATTR;
				end

				P_WAIT_ATTR: begin
					if (vram_rd_valid)
						pipe_state <= P_FETCH_CODE;
				end

				P_FETCH_CODE: begin
					prep_attr <= vram_rd_data;
					vram_rd_addr <= {vram_layer_code, prep_tile_index};
					vram_rd_req <= 1'b1;
					pipe_state <= P_WAIT_CODE;
				end

				P_WAIT_CODE: begin
					if (vram_rd_valid)
						pipe_state <= P_READ_VRAM;
				end

				P_READ_VRAM: begin
					prep_code <= vram_rd_data[13:0];
					prep_pal_bank <= prep_attr[6:0];
					prep_hflip <= prep_attr[14] ^ prep_ctrl_bit0_lat[prep_layer];
					pipe_state <= P_FETCH_ROM;
				end

				P_FETCH_ROM: begin
					if (prep_code == 14'd0 || prep_code >= 14'd12288) begin
						prep_tile_row <= 32'd0;
						pipe_state <= P_WRITE_PIX;
					end else begin
						tilerom_addr <= {5'd0, prep_code, (prep_fine_y ^ {3{prep_attr[15] ^ prep_ctrl_bit0_lat[prep_layer]}}), 2'b00};
						tilerom_req <= 1'b1;
						pipe_state <= P_WAIT_ROM;
					end
				end

				P_WAIT_ROM: begin
					if (tilerom_valid) begin
						tilerom_req <= 1'b0;
						prep_tile_row <= tilerom_data;
						pipe_state <= P_WRITE_PIX;
					end
				end

				P_WRITE_PIX: begin
					// Layer 0: always write. Layer 1: only opaque pixels (compositing)
					if (prep_layer == 1'b0 || get_pixel(prep_tile_row, prep_hflip, prep_pix_idx) != 4'd0) begin
						prep_linebuf_we <= 1'b1;
						prep_linebuf_addr <= prep_out_x[8:0];
						// prio: L0=01, L2=10 (matches MAME priority bitmap values)
						prep_linebuf_data <= {(prep_layer ? 2'b10 : 2'b01),
							(get_pixel(prep_tile_row, prep_hflip, prep_pix_idx) != 4'd0),
							prep_pal_bank,
							get_pixel(prep_tile_row, prep_hflip, prep_pix_idx)};
					end

					if (prep_out_x == LAST_PIXEL) begin
						if (prep_layer == 1'b0) begin
							// Layer 0 done → start Layer 1 on same line buffer
							prep_layer  <= 1'b1;
							prep_out_x  <= 10'd0;
							pipe_state  <= P_FETCH_TILE;
						end else begin
							// Both layers done → line complete
							prep_layer  <= 1'b0;
							prep_busy   <= 0;
							prep_line_ready <= 1;
						end
					end else begin
						prep_out_x <= prep_out_x + 10'd1;
						if (prep_pix_idx == 3'd7)
							pipe_state <= P_FETCH_TILE;
						else
							prep_pix_idx <= prep_pix_idx + 3'd1;
					end
				end

				default: pipe_state <= P_FETCH_TILE;
			endcase
		end

		// Write to line buffer
		if (prep_linebuf_we) begin
			if (prep_buf_sel)
				linebuf1[prep_linebuf_addr] <= prep_linebuf_data;
			else
				linebuf0[prep_linebuf_addr] <= prep_linebuf_data;
		end
	end
end

// =====================================================================
// Display output: line buffer -> palette -> RGB
// =====================================================================
wire [3:0] disp_pixel = display_word[3:0];
wire [6:0] disp_pal   = display_word[10:4];
wire       disp_opaque = display_word[11];
wire [1:0] disp_prio   = display_word[13:12];

always @(posedge clk) begin
	pal_lookup_addr <= {disp_pal, disp_pixel};
end

// Delay opaque/prio by 2 clocks to align with pal_data:
//   Cycle N:   display_word available (combinatorial from line buffer)
//   Cycle N+1: pal_lookup_addr registered
//   Cycle N+2: pal_data registered (BRAM read)
// So opaque/prio need 2 stages to match.
reg        disp_opaque_d, disp_opaque_dd;
reg  [1:0] disp_prio_d,   disp_prio_dd;
always @(posedge clk) begin
	disp_opaque_d  <= disp_opaque;
	disp_prio_d    <= disp_prio;
	disp_opaque_dd <= disp_opaque_d;
	disp_prio_dd   <= disp_prio_d;
end

// xBGR555 -> RGB888
wire [7:0] out_r = {pal_data[4:0],  pal_data[4:2]};
wire [7:0] out_g = {pal_data[9:5],  pal_data[9:7]};
wire [7:0] out_b = {pal_data[14:10], pal_data[14:12]};

assign tile_rgb    = {out_r, out_g, out_b};
assign tile_opaque = disp_opaque_dd;
assign tile_prio   = disp_prio_dd;

// Debug: coordinate mapping probe
// Captures the mapping triad at out_x=0 and out_x=112 on first target line
// Uses existing dbg_tile_code/attr/romdata outputs (already wired to overlay)
// Format:
//   dbg_tile_code   = {6'b0, src_x[9:0]}  at out_x=0
//   dbg_tile_attr   = {3'b0, tile_index[12:0]} at out_x=0
//   dbg_tile_romdata = {src_y[8:0], prep_line_y[8:0], code_at_0[13:0]}
reg dbg_captured;
reg [13:0] dbg_code_at_0;
wire dbg_target_line = (prep_line_y >= 9'd32) && (prep_line_y <= 9'd39);
always @(posedge clk) begin
	if (reset || (start_of_line && render_y == 9'd0)) begin
		dbg_captured     <= 0;
		dbg_tile_code    <= 16'd0;
		dbg_tile_attr    <= 16'd0;
		dbg_tile_romdata <= 32'd0;
		dbg_code_at_0    <= 14'd0;
	end else if (!dbg_captured && prep_busy && prep_layer == 1'b0 && dbg_target_line) begin
		// Capture src_x, src_y, tile_index at first pixel of target line
		if (pipe_state == P_FETCH_TILE && prep_out_x == 10'd0) begin
			dbg_tile_code <= {6'b0, prep_src_x};
			dbg_tile_attr <= {3'b0, prep_tile_index};
		end
		// Capture tile code when it arrives
		if (pipe_state == P_READ_VRAM && prep_out_x <= 10'd7) begin
			dbg_code_at_0 <= vram_rd_data[13:0];
		end
		// Finalize when we advance past first tile
		if (pipe_state == P_FETCH_TILE && prep_out_x == 10'd8) begin
			dbg_tile_romdata <= {prep_src_y, prep_line_y[8:0], dbg_code_at_0};
			dbg_captured <= 1;
		end
	end
end


endmodule
