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

// darius_sprite_renderer — Sprite renderer.
// Legge entry sprite da local RAM (snooped dal CPU bus), fetch pixel dalla
// sprite ROM via SDRAM, disegna in line buffer. Supporta flip X/Y, priority,
// tiles 16x16 4bpp.
//
// Sprite RAM format (MAME, 4 words per entry at $E00100):
//   word[0]: Y position → sy = (256 - data) & 0x1FF
//   word[1]: X position → sx = data & 0x3FF
//   word[2]: tile code[12:0], flipX[14], flipY[15]
//   word[3]: priority[7], color[6:0]
//
// Sprite tiles: 16x16, 4bpp, 128 bytes each (32 SDRAM words)
// Layout: 4 quadrants (TL, TR, BL, BR), each 8x8, 4 planes
// SDRAM word = {plane3[7:0], plane2[7:0], plane1[7:0], plane0[7:0]}
// draw_sprites called with x_offs=xoffs (scroll-dependent), y_offs=-8

module darius_sprite_renderer #(
	parameter SS_IDX = -1      // savestate: indice slave OBJ RAM locale (pattern F2)
) (
	input  wire        clk,
	input  wire        reset,
	// Savestate (pattern F2: ssbus dentro al chip) — OBJ RAM locale snooped
	ssbus_if.slave     ssbus,
	input  wire  [9:0] render_x,
	input  wire  [8:0] render_y,

	// X offset (scroll-dependent, from MAME draw_sprites x_offs parameter)
	input  wire  [9:0] x_offset,

	// Main CPU bus snoop (sprite RAM writes at $E00100-$E00FFF)
	input  wire [23:0] cpu_bus_addr,
	input  wire        cpu_bus_asn,
	input  wire        cpu_bus_rnw,
	input  wire  [1:0] cpu_bus_dsn,
	input  wire [15:0] cpu_bus_wdata,
	input  wire        cpu_bus_cs,

	// Sub CPU bus snoop (also writes sprite RAM)
	input  wire [23:0] sub_bus_addr,
	input  wire        sub_bus_asn,
	input  wire        sub_bus_rnw,
	input  wire  [1:0] sub_bus_dsn,
	input  wire [15:0] sub_bus_wdata,
	input  wire        sub_bus_cs,

	// Sprite ROM via SDRAM (32-bit reads)
	input  wire [31:0] spriterom_data,
	input  wire        spriterom_valid,
	output reg  [23:0] spriterom_addr,
	output reg         spriterom_req,

	// Palette lookup (shared with tile palette)
	input  wire [15:0] pal_data,       // xBGR555 from palette RAM
	output reg  [10:0] pal_lookup_addr, // {color[6:0], pixel[3:0]}

	// Pixel output
	// OSD adjustable offsets
	input  wire signed [9:0] spr_xoff, spr_yoff,

	output wire [23:0] sprite_rgb,
	output wire  [1:0] sprite_prio,
	output wire        sprite_opaque,
	output wire [12:0] dbg_disp_word,

	// Follow-cam: tracking navicella
	output reg   [9:0] ship_x,      // ultima X valida (coord. schermo)
	output reg         ship_commit  // pulse 1 clk a ogni aggiornamento
);

localparam H_ACTIVE = 10'd864;
localparam V_ACTIVE = 9'd224;
localparam Y_OFFSET = 9'd32;  // y_offs = -8 from MAME + north adjust

// =====================================================================
// CPU bus write detection (main + sub)
// =====================================================================
// Main CPU snoop
wire main_sprite_sel = (cpu_bus_addr >= 24'hE00100) && (cpu_bus_addr <= 24'hE00FFF);
wire main_write_active = ~cpu_bus_asn && ~cpu_bus_rnw && cpu_bus_cs && (cpu_bus_dsn != 2'b11);
reg main_write_seen;
always @(posedge clk) begin
	if (reset) main_write_seen <= 1'b0;
	else if (!main_write_active) main_write_seen <= 1'b0;
	else main_write_seen <= 1'b1;
end
wire main_wr_pulse = main_write_active && !main_write_seen;
wire main_spr_wr = main_wr_pulse && main_sprite_sel;

// Sub CPU snoop
wire sub_sprite_sel = (sub_bus_addr >= 24'hE00100) && (sub_bus_addr <= 24'hE00FFF);
wire sub_write_active = ~sub_bus_asn && ~sub_bus_rnw && sub_bus_cs && (sub_bus_dsn != 2'b11);
reg sub_write_seen;
always @(posedge clk) begin
	if (reset) sub_write_seen <= 1'b0;
	else if (!sub_write_active) sub_write_seen <= 1'b0;
	else sub_write_seen <= 1'b1;
end
wire sub_wr_pulse = sub_write_active && !sub_write_seen;
wire sub_spr_wr = sub_wr_pulse && sub_sprite_sel;

// Mux: main has priority if both write same cycle (unlikely)
wire spr_wr = main_spr_wr || sub_spr_wr;

// =====================================================================
// Local sprite RAM (snooped, 1920 words = 480 sprites × 4 words)
// =====================================================================
// Sprite RAM word address: $E00100[11:1]=0x080, $E00FFF[11:1]=0x7FF
// Subtract base 0x080 to get 0-based index (0 to 0x77F = 1919)
wire [10:0] main_spr_word_addr = cpu_bus_addr[11:1] - 11'h080;
wire [10:0] sub_spr_word_addr  = sub_bus_addr[11:1] - 11'h080;
wire [10:0] spr_wr_addr  = main_spr_wr ? main_spr_word_addr : sub_spr_word_addr;
wire [15:0] spr_wr_data  = main_spr_wr ? cpu_bus_wdata : sub_bus_wdata;
wire [1:0]  spr_wr_be    = main_spr_wr ? ~cpu_bus_dsn : ~sub_bus_dsn;
(* ramstyle = "no_rw_check" *) reg [7:0] spr_ram_hi [0:2047];
(* ramstyle = "no_rw_check" *) reg [7:0] spr_ram_lo [0:2047];
// DOUBLE BUFFER OBJ RAM (portato da Darius2NinjaWarriors): la CPU (main+sub)
// scrive la primaria (live); il renderer legge la copia "frozen". Copia atomica
// live->frozen durante il vblank -> il frame disegna uno stato COERENTE congelato
// a inizio frame -> nessuna race mid-frame (aggiornamento CPU intraframe non
// spezza piu' le scanline: appare dal frame successivo, come sull'HW Taito).
(* ramstyle = "no_rw_check" *) reg [7:0] spr_ram_hi_frozen [0:2047];
(* ramstyle = "no_rw_check" *) reg [7:0] spr_ram_lo_frozen [0:2047];
reg  [7:0] spr_rdata_hi, spr_rdata_lo;
reg [10:0] spr_rd_addr;
wire [15:0] spr_rdata = {spr_rdata_hi, spr_rdata_lo};

// Savestate: adaptor sulla porta di scrittura snoop (CPU ferme durante SS);
// gather-read sull'indirizzo muxato della stessa porta. FSM di scan legge
// dall'altra porta: 2 porte totali = M10K TDP.
wire        ssob_we_lo, ssob_we_hi;
wire [10:0] ssob_addr;
wire [15:0] ssob_wdata;
reg  [15:0] spr_ram_ss_q;
ss_ram16_adaptor #(.WIDTHAD(11), .SS_IDX(SS_IDX)) u_ss_objram (
	.clk(clk),
	.we_lo_in(spr_wr & spr_wr_be[0]),
	.we_hi_in(spr_wr & spr_wr_be[1]),
	.addr_in(spr_wr_addr),
	.wdata_in(spr_wr_data),
	.we_lo_out(ssob_we_lo), .we_hi_out(ssob_we_hi),
	.addr_out(ssob_addr), .wdata_out(ssob_wdata),
	.q_in(spr_ram_ss_q),
	.ssbus(ssbus)
);

// Port A LIVE: scrittura CPU (snoop via adaptor) + gather-read savestate.
// Il savestate legge/scrive la LIVE (lo stato "vero" del gioco).
always @(posedge clk) begin
	if (ssob_we_hi) spr_ram_hi[ssob_addr] <= ssob_wdata[15:8];
	if (ssob_we_lo) spr_ram_lo[ssob_addr] <= ssob_wdata[7:0];
	spr_ram_ss_q <= {spr_ram_hi[ssob_addr], spr_ram_lo[ssob_addr]};
end

// Copy LIVE -> FROZEN durante vblank (render_y >= V_ACTIVE). Un'entry/clk,
// 0..2047. All'ingresso vblank riparte; al reset copy_done=0 forza la prima
// copia completa prima di partire in active. Vblank Darius ~38 righe x ~6108
// clk >> 2048 -> copia sempre completata prima del display.
reg [10:0] copy_idx;
reg        copy_done;
reg  [8:0] prev_render_y_copy;
wire vblank_copy = (render_y >= V_ACTIVE);
always @(posedge clk) begin
	if (reset) begin
		copy_idx <= 11'd0;
		copy_done <= 1'b0;
		prev_render_y_copy <= 9'h1FF;
	end else begin
		prev_render_y_copy <= render_y;
		// ingresso vblank: (ri)parte la copia
		if (vblank_copy && !(prev_render_y_copy >= V_ACTIVE)) begin
			copy_idx  <= 11'd0;
			copy_done <= 1'b0;
		end
		if (!copy_done && vblank_copy) begin
			if (copy_idx == 11'd2047) copy_done <= 1'b1;
			else                      copy_idx  <= copy_idx + 11'd1;
		end
	end
end

// Port A FROZEN: copy write live->frozen in vblank.
// Port B FROZEN: il renderer (FSM scan) legge da qui (no race con CPU writes).
always @(posedge clk) begin
	if (!copy_done) spr_ram_hi_frozen[copy_idx] <= spr_ram_hi[copy_idx];
	spr_rdata_hi <= spr_ram_hi_frozen[spr_rd_addr];
end
always @(posedge clk) begin
	if (!copy_done) spr_ram_lo_frozen[copy_idx] <= spr_ram_lo[copy_idx];
	spr_rdata_lo <= spr_ram_lo_frozen[spr_rd_addr];
end

// [PERF] max slot OBJ RAM scritto dalla CPU: lo scan parte da qui invece che da
// 479 -> scan piu' corto -> draw finisce prima -> meno sfori (= meno linee ripetute).
// Monotono (OBJ riscritta ogni frame). NON cambia cosa viene disegnato.
reg [8:0] max_idx_used;
always @(posedge clk) begin
	if (reset) max_idx_used <= 9'd0;
	else if (spr_wr && (spr_wr_addr[10:2] > max_idx_used))
		max_idx_used <= spr_wr_addr[10:2];
end

// =====================================================================
// Follow-cam: snoop entry navicella
// =====================================================================
// Nave = slot 7 ($E00138), code $3E5-$3E7 (rotazione fiamma).
// Verificato: MAME attract (slot 7 = code $3E6 128/128 frame demo) +
// Darius-Native main_annotated.md ("La nave è a $E00138, code $3E6").
// Word addr 0-based (base $080): X = $E0013A -> 11'h01D, code = $E0013C -> 11'h01E.
// Commit order-independent (clear di frame scrive X=$380/code=0 prima del
// rewrite): X candidata filtrata dal valore park $380 (>=880 = offscreen),
// commit su write X con code valido O su write code valido con X plausibile.
localparam [10:0] SHIP_X_WADDR    = 11'h01D;
localparam [10:0] SHIP_CODE_WADDR = 11'h01E;

reg [9:0] ship_cand_x;
reg       ship_code_ok;
wire      wr_x_plausible  = spr_wr_data[9:0] < 10'd880;
wire      cand_plausible  = ship_cand_x < 10'd880;
wire      wr_code_is_ship = (spr_wr_data[12:0] >= 13'h03E5) && (spr_wr_data[12:0] <= 13'h03E7);

always @(posedge clk) begin
	ship_commit <= 1'b0;
	if (reset) begin
		ship_cand_x  <= 10'd880;
		ship_code_ok <= 1'b0;
		ship_x       <= 10'd432;
	end else if (spr_wr) begin
		if (spr_wr_addr == SHIP_X_WADDR) begin
			ship_cand_x <= spr_wr_data[9:0];
			if (ship_code_ok && wr_x_plausible) begin
				ship_x      <= spr_wr_data[9:0];
				ship_commit <= 1'b1;
			end
		end
		if (spr_wr_addr == SHIP_CODE_WADDR) begin
			ship_code_ok <= wr_code_is_ship;
			if (wr_code_is_ship && cand_plausible) begin
				ship_x      <= ship_cand_x;
				ship_commit <= 1'b1;
			end
		end
	end
end

// =====================================================================
// Sprite line buffer (double-buffered)
// =====================================================================
// Entry: {priority[1:0], color[6:0], pixel[3:0]} = 13 bits, 0 = transparent
(* ramstyle = "no_rw_check" *) reg [12:0] spr_lb0 [0:1023];
(* ramstyle = "no_rw_check" *) reg [12:0] spr_lb1 [0:1023];
reg [12:0] spr_lb0_q, spr_lb1_q;

reg        spr_disp_sel;     // which buffer is being displayed
reg [12:0] spr_disp_word;
reg  [9:0] spr_disp_addr;

always @(posedge clk) begin
	spr_lb0_q <= spr_lb0[spr_disp_addr];
	spr_lb1_q <= spr_lb1[spr_disp_addr];
end

// Line buffer write
reg        lb_we;
reg  [9:0] lb_waddr;
reg [12:0] lb_wdata;
reg        lb_buf_sel;  // which buffer is being written (opposite of display)

always @(posedge clk) begin
	if (lb_we) begin
		if (lb_buf_sel)
			spr_lb1[lb_waddr] <= lb_wdata;
		else
			spr_lb0[lb_waddr] <= lb_wdata;
	end
end

// Display read
wire in_active = (render_x < H_ACTIVE) && (render_y < V_ACTIVE);
always @(posedge clk) begin
	spr_disp_addr <= (render_x < H_ACTIVE - 10'd1) ? render_x + 10'd1 : 10'd0;
end
// Combinatorial mux — matches panel renderer pipeline (no extra register stage)
always @(*) begin
	if (!in_active)
		spr_disp_word = 13'd0;
	else if (spr_disp_sel)
		spr_disp_word = spr_lb1_q;
	else
		spr_disp_word = spr_lb0_q;
end

// =====================================================================
// Pixel extraction: bit-per-plane (MAME method)
// =====================================================================
// Pixel extraction: bit-per-plane
//
// BYTE ORDER — HARDWARE VALIDATED (2026-03-31)
// The theoretical model of the MRA->WIDE=1->SDRAM->bridge path predicted:
//   tile_data = {a96_46, a96_47, a96_44, a96_45}
// But real MiSTer hardware produces a different byte order in row_data.
// This was confirmed by:
//   - Debug overlay showing PA=646 (pixel 6) where MAME expects pixel 5
//   - Systematic HW testing of 8+ permutations
//   - PA lookup live showing correct bank 0x64x entries with this mapping
// The effective byte layout observed on hardware is:
//   [31:24] = plane2 byte    [23:16] = plane3 byte
//   [15:8]  = plane0 byte    [7:0]   = plane1 byte
// The root cause is likely in how the HPS/framework packs MRA interleave
// output="32" bytes into WIDE=1 16-bit transfers — the two central bytes
// of the 32-bit word arrive swapped relative to the theoretical model.
//
// MAME pixel = {plane3_bit, plane2_bit, plane1_bit, plane0_bit}
function automatic [3:0] spr_get_pixel;
	input [31:0] row_data;
	input        hflip;
	input  [2:0] pix_idx;
	reg [2:0] bp;
	reg [7:0] p0, p1, p2, p3;
	begin
		bp = 3'd7 - pix_idx;   // flip handled by draw_x, not bit extraction
		p0 = row_data[23:16];  // plane 0 — original mapping (MRA reordered)
		p1 = row_data[7:0];    // plane 1
		p2 = row_data[31:24];  // plane 2
		p3 = row_data[15:8];   // plane 3
		spr_get_pixel = { p3[bp], p2[bp], p1[bp], p0[bp] };  // original nibble order
	end
endfunction

// =====================================================================
// SCAN-OMBRA + FIFO (portato da NightSlashers boogwings_sprites, adattato a
// Darius): la scansione della OBJ RAM (Y/X/code/attr per ~480 slot) e' una FSM
// SEPARATA che accoda i MATCH in una FIFO, in PARALLELO al draw. Prima lo scan
// era dentro la FSM di draw (seriale: 3 clk/slot solo per testare Y, 9 se in
// banda) -> su frame densi il draw sforava la scanline -> il double-buffer
// perdeva new_line -> LINEA RIPETUTA. Con la scan-ombra il draw drena solo la
// FIFO e finisce in tempo -> new_line mai perso -> niente ripetizione.
//
// SINCRONIA INVARIATA: il double-buffer resta ancorato a render_y/new_line
// (come prima, a Darius NON si sfasa). Cambia SOLO la sorgente degli sprite del
// draw: FIFO invece di scan inline. Pixel/priorita' bit-identici (stesso ordine
// max_idx_used->0, stesso spr_get_pixel, stesso line buffer).
// =====================================================================
reg  [8:0] prev_render_y;
wire new_line = in_active && (render_y != prev_render_y);

// ---- FSM di DRAW (consuma la FIFO) ----
localparam D_IDLE      = 3'd0;
localparam D_CLEAR     = 3'd1;
localparam D_POP       = 3'd2;   // preleva un match dalla FIFO
localparam D_FETCH_ROM = 3'd3;
localparam D_WAIT_ROM  = 3'd4;
localparam D_DRAW      = 3'd5;
localparam D_NEXT      = 3'd6;

reg  [2:0] draw_state;
reg  [9:0] clear_addr;
reg  [8:0] prep_line_y;    // line being prepared (dal draw: render_y+1 su new_line)

// ---- FSM di SCAN (riempie la FIFO) ----
localparam SC_IDLE    = 4'd0;
localparam SC_READ_Y  = 4'd1;
localparam SC_WAIT_Y  = 4'd2;
localparam SC_LATCH_Y = 4'd3;
localparam SC_READ_X  = 4'd4;
localparam SC_LATCH_X = 4'd5;
localparam SC_READ_CODE = 4'd6;
localparam SC_LATCH_CODE = 4'd7;
localparam SC_READ_ATTR = 4'd8;   // = wait attr
localparam SC_PUSH    = 4'd9;

reg  [3:0] sc_state;
reg  [8:0] sc_idx;         // sprite index (max_idx_used .. 0)
reg  [8:0] sc_line_y;      // linea che lo scan sta preparando (= prep_line_y latchata a start)
reg        sc_run;         // scan attivo per la linea corrente

// registri di decode dello scan (equivalenti ai vecchi cur_*)
reg signed [10:0] sc_sx;
reg [12:0] sc_code;
reg        sc_flipx, sc_flipy;
reg  [6:0] sc_color;
reg        sc_prio;
reg  [3:0] sc_row_diff_hit;
reg  [3:0] sc_draw_row;

// ---- FIFO match scan->draw ----
// entry = {sx[10:0], code[12:0], flipx, color[6:0], prio, draw_row[3:0]} = 37 bit.
// MLAB (come NS): la FIFO scan->render in M10K aveva race read-during-write sul
// ferro; MLAB (LUTRAM write-through) la elimina. 64 entry >> sprite/linea reali.
localparam FF_W = 37;
(* ramstyle = "MLAB" *) reg [FF_W-1:0] mfifo [0:63];
reg  [6:0] mf_w, mf_r;     // 7-bit (bit alto = wrap) su 64 entry
reg [FF_W-1:0] mf_q_r;
reg        mf_empty_d;
wire       mf_empty = (mf_w == mf_r);
wire       mf_full  = (mf_w[6] != mf_r[6]) && (mf_w[5:0] == mf_r[5:0]);

// Draw state
reg signed [10:0] cur_sx;
reg [12:0] cur_code;
reg        cur_flipx;
reg  [6:0] cur_color;
reg        cur_prio;
reg  [3:0] draw_row_in_spr;
reg [31:0] cur_romdata;
reg  [3:0] draw_pix;       // pixel counter within row (0-15)

// FIFO read registrata (MLAB): mf_q_r valido perche' mf_r fermo da >=1 ciclo;
// mf_empty_d (ritardato) = guardia read-during-write (pop solo entry scritte >=1 ck).
always @(posedge clk) begin
	mf_q_r     <= mfifo[mf_r[5:0]];
	mf_empty_d <= mf_empty;
end

// =====================================================================
// FSM di SCAN (scan-ombra): scandisce la OBJ RAM, accoda i match in FIFO.
// Usa la porta di lettura OBJ RAM (spr_rd_addr/spr_rdata). Gira in parallelo
// alla FSM di draw. Ordine max_idx_used->0 = identico a prima (first-wins).
// =====================================================================
always @(posedge clk) begin
	if (reset) begin
		sc_state <= SC_IDLE;
		sc_idx   <= 0;
		sc_run   <= 1'b0;
		mf_w     <= 7'd0;
	end else begin
		// Start scan quando il draw inizia a preparare una nuova linea (D_CLEAR).
		// sc_line_y = prep_line_y latchata; mf_w azzerato = FIFO ripartita per la linea.
		if (new_line) begin
			sc_line_y <= (render_y >= V_ACTIVE - 9'd1) ? 9'd0 : render_y + 9'd1;
			sc_idx    <= max_idx_used;
			mf_w      <= 7'd0;
			sc_run    <= 1'b1;
			sc_state  <= SC_READ_Y;
		end else begin
			case (sc_state)
				SC_IDLE: ; // attende new_line

				SC_READ_Y: begin
					spr_rd_addr <= {sc_idx, 2'b00};
					sc_state    <= SC_WAIT_Y;
				end

				SC_WAIT_Y: sc_state <= SC_LATCH_Y;  // BRAM latency

				SC_LATCH_Y: begin
					reg [8:0] sy_calc;
					reg [8:0] row_diff;
					sy_calc = (9'd256 - spr_rdata[8:0] - 9'd16 + spr_yoff[8:0]) & 9'h1FF;
					row_diff = (sc_line_y - sy_calc) & 9'h1FF;
					if (row_diff < 9'd16) begin
						sc_row_diff_hit <= row_diff[3:0];
						spr_rd_addr     <= {sc_idx, 2'b01};
						sc_state        <= SC_READ_X;
					end else begin
						// non in banda Y: prossimo slot
						if (sc_idx == 9'd0) begin sc_run <= 1'b0; sc_state <= SC_IDLE; end
						else begin sc_idx <= sc_idx - 9'd1; sc_state <= SC_READ_Y; end
					end
				end

				SC_READ_X: sc_state <= SC_LATCH_X;

				SC_LATCH_X: begin
					begin
						reg signed [10:0] sx_raw;
						sx_raw = {1'b0, spr_rdata[9:0]} - {1'b0, x_offset} + spr_xoff;
						sc_sx <= (sx_raw > 11'sd900) ? (sx_raw - 11'sd1024) : sx_raw;
					end
					spr_rd_addr <= {sc_idx, 2'b10};
					sc_state    <= SC_READ_CODE;
				end

				SC_READ_CODE: sc_state <= SC_LATCH_CODE;

				SC_LATCH_CODE: begin
					sc_code  <= spr_rdata[12:0];
					sc_flipx <= spr_rdata[14];
					sc_flipy <= spr_rdata[15];
					spr_rd_addr <= {sc_idx, 2'b11};
					sc_state <= SC_READ_ATTR;
				end

				SC_READ_ATTR: sc_state <= SC_PUSH;  // wait attr word

				SC_PUSH: begin
					reg [3:0] draw_row;
					sc_color <= spr_rdata[6:0];
					sc_prio  <= spr_rdata[7];
					draw_row = sc_flipy ? (4'd15 - sc_row_diff_hit) : sc_row_diff_hit;
					// push solo se code!=0 e c'e' spazio; entry = {sx,code,flipx,color,prio,row}
					if (sc_code != 13'd0 && !mf_full) begin
						mfifo[mf_w[5:0]] <= {sc_sx, sc_code, sc_flipx, spr_rdata[6:0], spr_rdata[7], draw_row};
						mf_w <= mf_w + 7'd1;
					end
					// se FIFO piena e code!=0: stallo qui finche' il draw drena
					if (sc_code != 13'd0 && mf_full) begin
						sc_state <= SC_PUSH;
					end else begin
						if (sc_idx == 9'd0) begin sc_run <= 1'b0; sc_state <= SC_IDLE; end
						else begin sc_idx <= sc_idx - 9'd1; sc_state <= SC_READ_Y; end
					end
				end

				default: sc_state <= SC_IDLE;
			endcase
		end
	end
end

// scan_done = lo scan ha finito TUTTI gli slot per la linea corrente.
wire scan_done = ~sc_run;

// =====================================================================
// FSM di DRAW: consuma la FIFO, fetch ROM, disegna nel double-buffer.
// Display/sincronia INVARIATI: swap a new_line, line buffer 2-buffer.
// =====================================================================
always @(posedge clk) begin
	lb_we <= 1'b0;
	spriterom_req <= 1'b0;

	if (reset) begin
		prev_render_y <= 9'h1FF;
		draw_state    <= D_IDLE;
		clear_addr    <= 0;
		spr_disp_sel  <= 0;
		lb_buf_sel    <= 1;
		prep_line_y   <= 0;
		mf_r          <= 7'd0;
	end else begin
		if (new_line) prev_render_y <= render_y;

		// new_line: swap buffer + reset draw per la nuova linea (come prima, ma
		// ora anche se il draw non aveva finito -> nessun new_line perso).
		if (new_line) begin
			spr_disp_sel <= lb_buf_sel;
			lb_buf_sel   <= ~lb_buf_sel;
			prep_line_y  <= (render_y >= V_ACTIVE - 9'd1) ? 9'd0 : render_y + 9'd1;
			clear_addr   <= 0;
			mf_r         <= 7'd0;   // riparte a drenare la FIFO della nuova linea
			draw_state   <= D_CLEAR;
		end else begin
			case (draw_state)
				D_IDLE: ; // attende new_line

				// Clear the write buffer
				D_CLEAR: begin
					lb_we    <= 1'b1;
					lb_waddr <= clear_addr;
					lb_wdata <= 13'd0;
					if (clear_addr == H_ACTIVE - 10'd1)
						draw_state <= D_POP;
					else
						clear_addr <= clear_addr + 10'd1;
				end

				// Pop next match from FIFO (guardia RDW: mf_empty_d).
				D_POP: begin
					if (!mf_empty && !mf_empty_d) begin
						cur_sx          <= mf_q_r[36:26];
						cur_code        <= mf_q_r[25:13];
						cur_flipx       <= mf_q_r[12];
						cur_color       <= mf_q_r[11:5];
						cur_prio        <= mf_q_r[4];
						draw_row_in_spr <= mf_q_r[3:0];
						mf_r  <= mf_r + 7'd1;
						draw_state <= D_FETCH_ROM;
					end else if (mf_empty && scan_done) begin
						// FIFO vuota e scan finito: linea completa, attende new_line
						draw_state <= D_IDLE;
					end
					// FIFO vuota ma scan in corso: attendi qui
				end

				// Fetch left half (byte addr = code*128 + quadrant + row*4)
				D_FETCH_ROM: begin
					spriterom_addr <= {cur_code, draw_row_in_spr[3], 1'b0, draw_row_in_spr[2:0], 2'b00};
					spriterom_req  <= 1'b1;
					draw_pix       <= 0;
					draw_state     <= D_WAIT_ROM;
				end

				D_WAIT_ROM: begin
					if (spriterom_valid) begin
						cur_romdata <= spriterom_data;
						draw_state  <= D_DRAW;
					end
				end

				// Draw 8 pixels from current ROM word (bit-per-plane, MAME method)
				D_DRAW: begin
					reg [3:0] pixel;
					reg signed [10:0] draw_x;

					pixel = spr_get_pixel(cur_romdata, cur_flipx, draw_pix[2:0]);
					draw_x = cur_flipx ? (cur_sx + (11'sd15 - {7'd0, draw_pix})) : (cur_sx + {7'd0, draw_pix});

					if (pixel != 4'd0 && draw_x >= 0 && draw_x < $signed({1'b0, H_ACTIVE})) begin
						lb_we    <= 1'b1;
						lb_waddr <= draw_x[9:0];
						lb_wdata <= {1'b0, cur_prio, cur_color, pixel};
					end

					if (draw_pix == 4'd7) begin
						// Fetch right half
						spriterom_addr <= {cur_code, draw_row_in_spr[3], 1'b1, draw_row_in_spr[2:0], 2'b00};
						spriterom_req  <= 1'b1;
						draw_pix       <= 4'd8;
						draw_state     <= D_WAIT_ROM;
					end else if (draw_pix == 4'd15) begin
						draw_state <= D_NEXT;
					end else begin
						draw_pix <= draw_pix + 4'd1;
					end
				end

				// Advance to next match
				D_NEXT: draw_state <= D_POP;

				default: draw_state <= D_IDLE;
			endcase
		end
	end
end

// =====================================================================
// Output: line buffer → palette → RGB
// =====================================================================
wire [3:0] disp_pixel = spr_disp_word[3:0];
wire [6:0] disp_color = spr_disp_word[10:4];
wire       disp_prio  = spr_disp_word[11];
wire       disp_hit   = (disp_pixel != 4'd0) && in_active;

// Register palette address (1 cycle latency)
always @(posedge clk) begin
	pal_lookup_addr <= {disp_color, disp_pixel};
end

// Delay opaque/prio by 2 clocks to align with pal_data:
//   Cycle N:   spr_disp_word available (combinatorial)
//   Cycle N+1: pal_lookup_addr registered here
//   Cycle N+2: sprite_pal_data registered in darius_dual68k_top.sv
// So opaque/prio need 2 stages, not 1.
reg        disp_hit_d,  disp_hit_dd;
reg        disp_prio_d, disp_prio_dd;
always @(posedge clk) begin
	disp_hit_d   <= disp_hit;
	disp_prio_d  <= disp_prio;
	disp_hit_dd  <= disp_hit_d;
	disp_prio_dd <= disp_prio_d;
end

// xBGR555 → RGB888
wire [7:0] out_r = {pal_data[4:0],  pal_data[4:2]};
wire [7:0] out_g = {pal_data[9:5],  pal_data[9:7]};
wire [7:0] out_b = {pal_data[14:10], pal_data[14:12]};

assign sprite_rgb    = {out_r, out_g, out_b};
assign sprite_prio   = {1'b0, disp_prio_dd};
assign sprite_opaque = disp_hit_dd;
assign dbg_disp_word = spr_disp_word;

endmodule
