/*  This file is part of Darius_MiSTer.  GPL-3.
    Author: Umberto Parisi (rmonic79)

    audio_mixer_gain — per-channel gain, isolated from the mixer and the chips.

    Each channel has a DEFAULT (localparam, a SINGLE PLACE below) and a 4-bit
    OSD selector that picks a MULTIPLIER RELATIVE to the default:

        coeff_out = DEFAULT * osd_mul(sel) >> 8      (8.8 fixed, 100% = 256)

    - The default is the channel's reference number (decided by US, not by the
      user). Changing it here moves the channel; all levels scale with it
      because they are relative (no absolute value anywhere else).
    - The osd_mul table covers ALL 16 levels even if the OSD exposes only a
      subset: changing which entries are shown does not touch the math.
    - sel = 0 (Default) returns the pure default -> mixer BIT-IDENTICAL to the
      behaviour without this module. Trace-free removal: drop the instance and
      put the fixed numbers back into the mixer's stage 0b.

    Note: the chips (jt03/jt5205) are NOT touched: the gain is downstream, on
    the mix coefficient.
*/

module audio_mixer_gain (
	// selettori OSD, uno per canale (4 bit = 16 livelli)
	input  wire [3:0] sel_fm0,
	input  wire [3:0] sel_fm1,
	input  wire [3:0] sel_p0a, sel_p0b, sel_p0c,   // SSG-A ch A/B/C
	input  wire [3:0] sel_p1a, sel_p1b, sel_p1c,   // SSG-B ch A/B/C
	input  wire [3:0] sel_msm,

	// coefficienti di mix scalati. Larghezza dimensionata per default x max
	// moltiplicatore (200% = x2) SENZA saturare qui: la saturazione avviene
	// SOLO al clamp finale (stage 3), cosi il rapporto tra i canali resta
	// identico a qualunque gain -> trasparente. FM max 154*2=308 (9b, uso 15b
	// margine), PSG max 5284*2=10568 (14b), MSM max 4071*2=8142 (14b).
	output wire [15:0] coeff_fm0,
	output wire [15:0] coeff_fm1,
	output wire [15:0] coeff_p0a, coeff_p0b, coeff_p0c,
	output wire [15:0] coeff_p1a, coeff_p1b, coeff_p1c,
	output wire [15:0] coeff_msm
);

// =====================================================================
//  DEFAULT PER CANALE — UNICO PUNTO DA MODIFICARE
//  Valori iniziali = coefficienti MAME-ratio attuali del mixer
//  (FM:PSG:MSM = 0.60:0.08:1.00). Alzare/abbassare qui = riferimento nuovo,
//  tutto il resto scala di conseguenza.
// =====================================================================
// Default x4 vs coeff MAME originali (154/5284/4071): base forte (oltre il
// 200% della taratura precedente). Il clamp finale (stage 3, su registri
// larghi) fa da limiter sui picchi estremi; il worst-case reale di gioco e'
// lontano dal worst-case teorico, quindi il percepito sale davvero. Rapporti
// FM:PSG:MSM IDENTICI (stesso fattore x4 su tutti). Le % OSD scalano da qui.
// Alzare/abbassare ancora = SOLO questi 9 numeri (un posto). 16-bit per
// contenere PSG 5284*4=21136 (oltre i 14b).
localparam [15:0] DEF_FM0 = 16'd616;     // FM chip 0 (BGM)   154  x4
localparam [15:0] DEF_FM1 = 16'd616;     // FM chip 1 (BGM)
localparam [15:0] DEF_P0A = 16'd21136;   // SSG-A ch A        5284 x4
localparam [15:0] DEF_P0B = 16'd21136;   // SSG-A ch B
localparam [15:0] DEF_P0C = 16'd21136;   // SSG-A ch C
localparam [15:0] DEF_P1A = 16'd21136;   // SSG-B ch A
localparam [15:0] DEF_P1B = 16'd21136;   // SSG-B ch B
localparam [15:0] DEF_P1C = 16'd21136;   // SSG-B ch C
localparam [15:0] DEF_MSM = 16'd16284;   // MSM5205 ADPCM     4071 x4

// =====================================================================
//  MOLTIPLICATORE RELATIVO — 8.8 fixed, 100% = 256. Tabella COMPLETA a 16
//  livelli (contemplati tutti, esposti a discrezione dell'OSD).
//  sel 0 = Default (256 = ×1 esatto -> canale invariato).
// =====================================================================
// Ordine voci CONF_STR (P2O[..]): 0=Default 1=Mute 2=25 3=50 4=75 5=125
// 6=150 7=200 (%). Relativi al default (256=x1.00). Niente "100%" (= Default).
//
// scale16 = DEFAULT * mul >> 8. DEF e' COSTANTE (localparam) e i mul sono
// costanti -> gli 8 risultati per canale sono COSTANTI note a compile-time.
// Le pre-calcolo con la macro SCALED (valutata dal compilatore in piena
// larghezza int -> NESSUN troncamento) e faccio un MUX 8:1 di costanti:
// ZERO DSP (prima 9 moltiplicatori). Nota bug evitato: scrivere
// `def_v * mul >> 8` inline in Verilog tronca il prodotto a 16b (larghezza =
// max operandi) -> qui i prodotti sono costanti localparam, calcolati a 32b.
`define SCALED(D,M) (16'(((D) * (M)) >> 8))

// mux 8:1 dei livelli per un canale (D = default costante del canale)
function automatic [15:0] scale_ch;
	input integer d;
	input [3:0]   sel;
	case (sel)
		4'd0: scale_ch = `SCALED(d, 256);  // Default
		4'd1: scale_ch = 16'd0;            // Mute
		4'd2: scale_ch = `SCALED(d, 64);   // 25%
		4'd3: scale_ch = `SCALED(d, 128);  // 50%
		4'd4: scale_ch = `SCALED(d, 192);  // 75%
		4'd5: scale_ch = `SCALED(d, 320);  // 125%
		4'd6: scale_ch = `SCALED(d, 384);  // 150%
		4'd7: scale_ch = `SCALED(d, 512);  // 200%
		default: scale_ch = `SCALED(d, 256);
	endcase
endfunction

assign coeff_fm0 = scale_ch(DEF_FM0, sel_fm0);
assign coeff_fm1 = scale_ch(DEF_FM1, sel_fm1);
assign coeff_p0a = scale_ch(DEF_P0A, sel_p0a);
assign coeff_p0b = scale_ch(DEF_P0B, sel_p0b);
assign coeff_p0c = scale_ch(DEF_P0C, sel_p0c);
assign coeff_p1a = scale_ch(DEF_P1A, sel_p1a);
assign coeff_p1b = scale_ch(DEF_P1B, sel_p1b);
assign coeff_p1c = scale_ch(DEF_P1C, sel_p1c);
assign coeff_msm = scale_ch(DEF_MSM, sel_msm);

endmodule
