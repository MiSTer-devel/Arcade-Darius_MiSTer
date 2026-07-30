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

// darius_followcam — single-screen view that follows the ship (CRT friendly).
// Post-compositor stage: buffers the full composite line (864 px, xBGR555
// lossless) and re-reads it with a camera offset + INTEGER pixel repeat.
// The internal render stays 864 slots; in follow the DISPLAY uses a WIDER
// active window (timing regenerated here, same 15.7kHz line rate) so on a
// CRT the image genuinely grows:
//   view 00 = triple  : full bypass (864 slots, render timing)
//   view 01 = W288 x4 : 1152 active slots (48.0us, panel proportions)
//   view 10 = W320 x4 : 1280 active slots (53.3us, "pushed 4:3")
//   view 11 = W432 x3 : 1296 active slots (54.0us, 1.5 screens)
// The capture samples the RGB at the SAME instant the framework samples it
// (on the ce_pix edge): any upstream pipeline (2-clk palette, panel-change
// warm-up) is already stable — the buffer holds EXACTLY what is seen in
// triple, with no spurious columns at the panel boundaries.
// Camera: target = ship centered, deadzone + slew, updated at end of VBlank.
// Ship absent (title/death): easing toward the playfield center.

module darius_followcam (
	input  wire       clk,
	input  wire       reset,
	input  wire [2:0] view,        // OSD: 0=triple, 1=288, 2=320, 3=432,
	                               //      4=screen jump (288, camera a scatti)
	input  wire       wide3,       // OSD: triple CRT wide (solo con view=0):
	                               // ri-emette la linea a 16MHz (/6) — 864 attivi
	                               // su 1018 slot = 54us, repeat 1:1 intero

	input  wire       ce_pix,
	input  wire       HBlank,      // timing RENDER (864 attivi su 1527)
	input  wire       HSync,
	input  wire       VBlank,
	input  wire [8:0] render_y,
	input  wire [9:0] render_x,

	// Tracking nave (da darius_sprite_renderer)
	input  wire [9:0] ship_x,      // ultima X commitata (coord. schermo 0..863)
	input  wire       ship_commit, // pulse 1 clk a ogni commit

	// Composito completo dal compositor
	input  wire [7:0] r_in, g_in, b_in,

	output wire [7:0] r_out, g_out, b_out,
	// Timing display (in follow: finestra allargata; in triple: passthrough)
	output wire       HBlank_out,
	output wire       HSync_out,
	// pixel-ce dello stream in uscita (= ce_pix, o 16MHz in triple wide)
	output wire       pix_ce,
	// Coordinata X per il pause_overlay (display slot in follow, render_x in triple)
	output wire [9:0] ovl_x
);

localparam [10:0] H_TOTAL = 11'd1527;

// view è quasi-statico (OSD): doppia registrazione per tagliare il path
// hps_io.status -> camera/contatori (setup fail @96MHz altrimenti)
// Classi larghezza: 288 (view 1 e 4-jump), 320 (view 2), 432 (view 3)
reg [2:0]  view_r;
reg        wide3_r;
reg        en;                   // follow attivo (view != 0)
reg        w3;                   // triple wide attivo (view==0 && wide3)
reg        out_en;               // en | w3: display rigenerato
reg        jump;                 // view 4: camera a scatti per pannello
reg [1:0]  rpt;                  // repeat mondo - 1 (3 = x4, 2 = x3, 0 = x1 wide3)
reg [9:0]  half_w, cam_max, cam_center;
reg [10:0] disp_act;             // slot attivi display
reg [10:0] hs_start, hs_end;     // finestra HSync display
wire v_w288 = (view_r == 3'd1) || (view_r == 3'd4);
always @(posedge clk) begin
	view_r     <= view;
	wide3_r    <= wide3;
	en         <= (view_r != 3'd0);
	w3         <= (view_r == 3'd0) && wide3_r;
	out_en     <= (view_r != 3'd0) || ((view_r == 3'd0) && wide3_r);
	jump       <= (view_r == 3'd4);
	rpt        <= (view_r == 3'd0) ? 2'd0 :          // wide3: repeat x1
	              (view_r == 3'd3) ? 2'd2 : 2'd3;
	half_w     <= v_w288           ? 10'd144 :
	              (view_r == 3'd2) ? 10'd160 : 10'd216;
	cam_max    <= v_w288           ? 10'd576 :
	              (view_r == 3'd2) ? 10'd544 : 10'd432;
	cam_center <= v_w288           ? 10'd288 :
	              (view_r == 3'd2) ? 10'd272 : 10'd216;
	// wide3: 864 attivi su 1018 slot @16MHz — BP corto = immagine a sinistra,
	// fine-tuning con CRT Adjust. Sync 75 slot (~4.7us).
	disp_act   <= (view_r == 3'd0) ? 11'd864 :
	              v_w288           ? 11'd1152 :
	              (view_r == 3'd2) ? 11'd1280 : 11'd1296;
	hs_start   <= (view_r == 3'd0) ? 11'd904 :
	              v_w288           ? 11'd1264 :
	              (view_r == 3'd2) ? 11'd1328 : 11'd1336;
	hs_end     <= (view_r == 3'd0) ? 11'd979 :
	              v_w288           ? 11'd1414 :
	              (view_r == 3'd2) ? 11'd1478 : 11'd1486;
end

// ce a 16MHz (/6 di clk) per wide3, agganciato in fase a inizio linea render.
// 6108 clk/linea / 6 = 1018 slot esatti — nessuna deriva.
reg [2:0] div6;
reg       ce6;
wire      slot_ce = w3 ? ce6 : ce_pix;
// pixel-ce dello stream in uscita (per CE_PIXEL e crt_adjust a valle)
assign pix_ce = slot_ce;

// =====================================================================
// Presenza nave: accumulata per frame (commit in qualsiasi punto del frame)
// =====================================================================
reg vb_d;
wire vb_end = vb_d & ~VBlank;   // fine vblank = inizio frame attivo
reg seen;
reg [9:0] ship_x_lat;

always @(posedge clk) begin
	vb_d <= VBlank;
	if (reset) begin
		seen       <= 1'b0;
		ship_x_lat <= 10'd432;
	end else begin
		if (ship_commit) begin
			seen       <= 1'b1;
			ship_x_lat <= ship_x;
		end
		if (vb_end) seen <= 1'b0;
	end
end

// =====================================================================
// Camera: desired = clamp(ship_x - W/2, 0, 864-W); deadzone 4 px,
// step proporzionale max 4 px/frame (nave max ~3 px/frame).
// Aritmetica PIPELINATA su 4 stadi (setup fail se in 1 ciclo): serve solo
// una volta a frame (vb_end), gli stadi sono stabili da migliaia di cicli.
// =====================================================================
reg [9:0] cam;

reg signed [11:0] des_raw_r;
reg [9:0]  desired_r;
reg signed [11:0] err_r;
reg [11:0] abs_r;
reg [9:0]  step_r;

always @(posedge clk) begin
	des_raw_r <= {2'b00, ship_x_lat} - {2'b00, half_w};
	desired_r <= !seen                  ? cam_center :
	             (des_raw_r < 12'sd0)   ? 10'd0 :
	             (des_raw_r > $signed({2'b00, cam_max})) ? cam_max :
	             des_raw_r[9:0];
	err_r     <= {2'b00, desired_r} - {2'b00, cam};
	abs_r     <= err_r[11] ? (~err_r + 12'd1) : err_r;
	step_r    <= (abs_r >= 12'd64) ? 10'd4 :
	             (abs_r >= 12'd16) ? 10'd2 :
	             (abs_r >  12'd4)  ? 10'd1 : 10'd0;

	if (reset)
		cam <= 10'd288;
	else if (vb_end)
		cam <= jump      ? panel_px :                    // screen jump: snap secco
		       (cam > cam_max) ? cam_max :               // clamp su cambio W
		       err_r[11] ? (cam - step_r) : (cam + step_r);
end

// ── Screen Jump (view 4): camera a scatti sul pannello della nave ─────────
// Isteresi ±16 px oltre il confine pannello (niente flip-flop ai bordi).
// Nave assente: pannello centrale.
reg [1:0] panel;      // 0=sinistro, 1=centro, 2=destro
reg [9:0] panel_px;   // panel * 288
always @(posedge clk) begin
	if (reset) begin
		panel    <= 2'd1;
		panel_px <= 10'd288;
	end else begin
		panel_px <= {panel, 8'd0} + {panel, 5'd0};   // panel*256 + panel*32 = *288
		if (vb_end) begin
			if (!seen)
				panel <= 2'd1;
			else if ((panel != 2'd2) && (ship_x_lat > panel_px + 10'd303))  // 288+16
				panel <= panel + 2'd1;
			else if ((panel != 2'd0) && (panel_px > 10'd16) && (ship_x_lat < panel_px - 10'd16))
				panel <= panel - 2'd1;
		end
	end
end

// =====================================================================
// Line buffer composito completo, double-buffer per parità di linea.
// xBGR555 (5 bit/canale) = lossless: RGB888 a monte è espansione 5->8.
// =====================================================================
(* ramstyle = "no_rw_check" *) reg [14:0] lb0 [0:1023];
(* ramstyle = "no_rw_check" *) reg [14:0] lb1 [0:1023];

reg line_par;

// edge detect su HBlank render
reg hb_d;
wire hb_rise = HBlank & ~hb_d;
wire hb_fall = ~HBlank & hb_d;
always @(posedge clk) begin
	hb_d <= HBlank;
	if (reset)
		line_par <= 1'b0;
	else if (hb_rise & ~VBlank)   // fine linea attiva render
		line_par <= ~line_par;
end

// write side — campiona al ce_pix edge, ESATTAMENTE come il framework:
// al ce edge che porta hc a x+1, l'RGB pre-edge è il pixel x definitivo
// (tutte le pipeline a monte, palette e warm-up cambio pannello, sono
// stabili). widx conta i pixel scritti dalla partenza linea: gli edge
// hc->1 .. hc->864 catturano i pixel 0..863.
wire [14:0] pix_in = {r_in[7:3], g_in[7:3], b_in[7:3]};
reg [9:0] widx;
always @(posedge clk) begin
	if (HBlank) widx <= 10'd0;
	else if (ce_pix && !VBlank && widx < 10'd864) begin
		if (line_par) lb1[widx] <= pix_in;
		else          lb0[widx] <= pix_in;
		widx <= widx + 10'd1;
	end
end

// =====================================================================
// Display timing follow: contatore di slot agganciato all'inizio della
// linea attiva render (hb_fall = slot 0). Stessa line rate del render,
// finestra attiva più larga, HSync ricentrato.
// =====================================================================
// Divisore /6 per wide3: fase agganciata a inizio linea render.
always @(posedge clk) begin
	if (hb_fall) begin
		div6 <= 3'd0;
		ce6  <= 1'b0;
	end else begin
		div6 <= (div6 == 3'd5) ? 3'd0 : div6 + 3'd1;
		ce6  <= (div6 == 3'd4);   // pulse su slot boundary /6
	end
end

reg [10:0] hc2;
always @(posedge clk) begin
	if (hb_fall)
		hc2 <= 11'd0;             // slot 0 corrente
	else if (slot_ce)
		hc2 <= (hc2 == H_TOTAL - 11'd1) ? 11'd0 : hc2 + 11'd1;
end

wire disp_hb = (hc2 >= disp_act);
wire disp_hs = ~((hc2 >= hs_start) && (hc2 < hs_end));  // attivo basso come render

// Parità di lettura latciata a FINE display attivo (slot disp_act, dopo il
// toggle di line_par allo slot 864): line_par cambia mentre il display attivo
// prosegue oltre — la lettura resta sul buffer della linea precedente per
// tutta la linea, e la nuova parità è pronta PRIMA dello slot 0 successivo.
reg rd_par;
reg dhb_d;
always @(posedge clk) begin
	dhb_d <= disp_hb;
	if (disp_hb & ~dhb_d) rd_par <= ~line_par;
end

// =====================================================================
// Read side: repeat INTERO — x4 (x3 in Wide). Prefetch +1 slot
// (stesso pattern dei renderer: dato pronto al ce edge).
// =====================================================================
reg  [1:0] wcnt;
reg  [9:0] idx;
reg  [9:0] rd_addr;
reg [14:0] lb0_q, lb1_q;

wire [9:0] idx_next = (wcnt == rpt) ? (idx + 10'd1) : idx;

always @(posedge clk) begin
	if (disp_hb) begin
		wcnt <= 2'd0;
		idx  <= 10'd0;
	end else if (slot_ce) begin
		if (wcnt == rpt) begin
			wcnt <= 2'd0;
			idx  <= idx + 10'd1;
		end else
			wcnt <= wcnt + 2'd1;
	end
	rd_addr <= (w3 ? 10'd0 : cam) + (disp_hb ? 10'd0 : idx_next);
	lb0_q   <= lb0[rd_addr];
	lb1_q   <= lb1[rd_addr];
end

wire [14:0] pix_rd = rd_par ? lb1_q : lb0_q;

// =====================================================================
// Output: espansione 5->8 (replica MSB), linea 0 nera (contiene l'ultima
// linea del frame precedente), bypass totale se view=Triple
// =====================================================================
wire disp_active = ~disp_hb & ~VBlank;
wire blank0 = (render_y == 9'd0) | ~disp_active;
wire [7:0] fr = {pix_rd[14:10], pix_rd[14:12]};
wire [7:0] fg = {pix_rd[9:5],   pix_rd[9:7]};
wire [7:0] fb = {pix_rd[4:0],   pix_rd[4:2]};

assign r_out = !out_en ? r_in : (blank0 ? 8'd0 : fr);
assign g_out = !out_en ? g_in : (blank0 ? 8'd0 : fg);
assign b_out = !out_en ? b_in : (blank0 ? 8'd0 : fb);

assign HBlank_out = out_en ? disp_hb : HBlank;
assign HSync_out  = out_en ? disp_hs : HSync;
assign ovl_x      = out_en ? ((hc2 > 11'd1023) ? 10'd1023 : hc2[9:0]) : render_x;

endmodule
