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

//============================================================================
//  ss_ym_shadow — YM2203 savestate WITHOUT touching the chips (shadow + replay).
//
//  Principle (Darius rule): jt03 stays vanilla. Here we:
//    1. SNOOP the Z80A->YM writes into a per-chip shadow RAM
//       (256 registers + 3 key-on slots per channel, addresses 0x100-0x102);
//    2. SAVE/RESTORE the shadow via ssbus (ss_ram_adaptor pattern);
//    3. on restore (end of stream, CPUs halted) an INJECTOR replays the
//       registers on the chip's normal bus. The chip is NOT reset: the
//       non-replayed state (prescaler etc.) stays alive. The fnum pairs
//       respect the latch order (A4 before A0, etc.). replay_busy holds the
//       pause high until the replay is finished.
//
//  Savestate removal: delete this file + the [SS-HOOK] points in the audio
//  module -> core identical to the baseline.
//============================================================================

`timescale 1ns / 1ps

module ss_ym_shadow #(
	parameter SS_IDX_SH1 = -1,
	parameter SS_IDX_SH2 = -1
) (
	input  wire       clk,
	input  wire       reset,
	// [FIX ottava] tick cen a cui il chip campiona il write (ce_4m_ym):
	// il replay deve presentare ogni write per 1 SOLO ce, come il chip reale.
	input  wire       ce_ym,

	// Snoop dal modulo audio ([SS-HOOK])
	input  wire       ym1_wr,
	input  wire       ym2_wr,
	input  wire       a0,
	input  wire [7:0] wdata,

	// Avvio replay: livello di ss_mem_read (stream restore in corso);
	// il replay parte sul fronte di discesa (stream concluso, CPU ferme).
	input  wire       ss_mem_read,
	output wire       replay_busy,

	// Injector verso il modulo audio ([SS-HOOK])
	output wire       rp_active,
	output reg        rp_cs1,
	output reg        rp_cs2,
	output reg        rp_a0,
	output reg  [7:0] rp_data,
	output reg        rp_wr,

	// Slave savestate (uno per chip)
	ssbus_if.slave    ssb1,
	ssbus_if.slave    ssb2
);

// =====================================================================
// Snoop: latch indice + mappatura scritture nella shadow.
// Key-on (reg 0x28): una entry per canale a 0x100+ch, cosi' il replay
// ripristina lo stato key di TUTTI i canali (l'ultima 0x28 da sola no).
// =====================================================================
reg [7:0] idx1, idx2;
always @(posedge clk) begin
	if (reset) begin
		idx1 <= 8'd0;
		idx2 <= 8'd0;
	end else begin
		if (ym1_wr && !a0) idx1 <= wdata;
		if (ym2_wr && !a0) idx2 <= wdata;
	end
end

// indirizzo shadow della scrittura dati corrente (idempotente sul burst)
wire [8:0] snoop1_addr = (idx1 == 8'h28) ? {7'h40, wdata[1:0]} : {1'b0, idx1};
wire       snoop1_wren = ym1_wr && a0 && !(idx1 == 8'h28 && wdata[1:0] == 2'b11);
wire [8:0] snoop2_addr = (idx2 == 8'h28) ? {7'h40, wdata[1:0]} : {1'b0, idx2};
wire       snoop2_wren = ym2_wr && a0 && !(idx2 == 8'h28 && wdata[1:0] == 2'b11);

// [FIX ottava] latch_fnum del chip (jt12_mmr.v:178) e' STATO INTERNO condiviso:
// block+fnum_hi latchato dall'ULTIMA scrittura A4/A5/A6/AC/AD/AE, consumato al
// commit A0/A1/A2. NON e' un registro indirizzabile -> senza questo snoop non
// verrebbe salvato. Al restore un driver che scrive fnum_lo senza riscrivere il
// block committerebbe con latch_fnum residuo (0) -> block sbagliato -> ottava sotto.
// Qui memorizzo l'ultimo block scritto in una cella shadow dedicata (0x103), che
// il replay ripristina come ultimissima op (vedi seq_reg / FSM). Reg A4/A5/A6 =
// idx 0xA4/0xA5/0xA6; AC/AD/AE = 0xAC/0xAD/0xAE. Il latch prende din[5:0].
wire is_ablk1 = (idx1==8'hA4)||(idx1==8'hA5)||(idx1==8'hA6)||(idx1==8'hAC)||(idx1==8'hAD)||(idx1==8'hAE);
wire is_ablk2 = (idx2==8'hA4)||(idx2==8'hA5)||(idx2==8'hA6)||(idx2==8'hAC)||(idx2==8'hAD)||(idx2==8'hAE);
wire       snoop1_lf_wren = ym1_wr && a0 && is_ablk1;
wire       snoop2_lf_wren = ym2_wr && a0 && is_ablk2;

// =====================================================================
// Shadow RAM per chip (512x8: 0x000-0x0FF registri, 0x100-0x102 key-on)
// interposta con ss_ram_adaptor (gather/scatter dal framework).
// =====================================================================
reg [7:0] sh1_ram [0:511];
reg [7:0] sh2_ram [0:511];
reg [7:0] sh1_q, sh2_q;

wire       sh1_we, sh2_we;
wire [8:0] sh1_addr, sh2_addr;
wire [7:0] sh1_wdata, sh2_wdata;

ss_ram_adaptor #(.WIDTH(8), .WIDTHAD(9), .SS_IDX(SS_IDX_SH1)) u_ss_sh1 (
	.clk(clk),
	.wren_in(snoop1_wren),
	.addr_in(snoop1_addr),
	.wdata_in(wdata),
	.wren_out(sh1_we),
	.addr_out(sh1_addr),
	.wdata_out(sh1_wdata),
	.q_in(sh1_q),
	.ssbus(ssb1)
);

ss_ram_adaptor #(.WIDTH(8), .WIDTHAD(9), .SS_IDX(SS_IDX_SH2)) u_ss_sh2 (
	.clk(clk),
	.wren_in(snoop2_wren),
	.addr_in(snoop2_addr),
	.wdata_in(wdata),
	.wren_out(sh2_we),
	.addr_out(sh2_addr),
	.wdata_out(sh2_wdata),
	.q_in(sh2_q),
	.ssbus(ssb2)
);

// [FIX ottava] cattura ritardata di 1 clk della scrittura latch_fnum in cella
// 0x103. Lo snoop del block (A4/A5/A6/AC/AD/AE) scrive gia' 0x0A4.. via adaptor
// nel ciclo corrente; il ciclo DOPO scriviamo lo stesso dato anche a 0x103, cosi'
// 0x103 = ULTIMO block scritto in ordine temporale = latch_fnum del chip. Nessun
// conflitto con l'adaptor: durante lo snoop live l'adaptor scrive 1 sola cella/ciclo
// e la 2a scrittura cade il ciclo successivo (snoop non back-to-back sul bus Z80).
reg       lf1_pend, lf2_pend;
reg [7:0] lf1_data, lf2_data;
always @(posedge clk) begin
	if (reset) begin lf1_pend <= 1'b0; lf2_pend <= 1'b0; end
	else begin
		lf1_pend <= snoop1_lf_wren; lf1_data <= wdata;
		lf2_pend <= snoop2_lf_wren; lf2_data <= wdata;
	end
end

// Porta RAM: write dall'adaptor; read muxata replay/adaptor (mai in
// conflitto: durante il replay lo ssbus e' idle, durante gather il
// replay e' fermo). [FIX ottava] +scrittura 0x103 (latch_fnum) il ciclo dopo lo snoop.
reg [8:0] rp_rd_addr;
always @(posedge clk) begin
	if (sh1_we)        sh1_ram[sh1_addr] <= sh1_wdata;
	else if (lf1_pend) sh1_ram[9'h103]   <= lf1_data;   // [FIX ottava] latch_fnum
	sh1_q <= sh1_ram[replay_busy ? rp_rd_addr : sh1_addr];
	if (sh2_we)        sh2_ram[sh2_addr] <= sh2_wdata;
	else if (lf2_pend) sh2_ram[9'h103]   <= lf2_data;   // [FIX ottava] latch_fnum
	sh2_q <= sh2_ram[replay_busy ? rp_rd_addr : sh2_addr];
end

// =====================================================================
// Sequenza di replay: indice -> registro YM (0x1FF = fine).
// PSG 00-0D, timer 24-27, op 30-9F, ch B0-B2, fnum in ordine latch,
// key-on per canale (shadow 0x100-0x102 -> scrittura su reg 0x28).
// =====================================================================
// Codifica cur[8:0]:
//   cur[8]=0 -> registro diretto cur[7:0], dato dallo shadow
//   cur[8]=1 & cur[7:0]<0x03 -> KEY-ON canale cur[1:0]: scrive 0x28, dato = shadow[0x100+ch]
//   cur[8]=1 & cur[7:0]>=0x10 -> KEY-OFF canale cur[1:0]: scrive 0x28, dato = {4'd0,ch} (opmask 0)
// Il KEY-OFF PRECEDE il key-on: garantisce il fronte di salita del key-on
// (jt12_eg: attacco solo su !keyon_last && keyon). Senza, un canale gia'
// keyato pre-load non retriggera -> nota muta finche' il driver non ri-keya.
localparam SEQ_LAST = 8'd153;
function [8:0] seq_reg(input [7:0] i);
	begin
		if      (i <= 8'd13)  seq_reg = {1'b0, 4'h0, i[3:0]};          // 00-0D
		else if (i <= 8'd17)  seq_reg = {1'b0, 8'h24 + (i - 8'd14)};// 24-27
		else if (i <= 8'd129) seq_reg = {1'b0, 8'h30 + (i - 8'd18)};   // 30-9F
		else if (i <= 8'd132) seq_reg = {1'b0, 8'hB0 + (i - 8'd130)};  // B0-B2
		else case (i)                                                   // fnum: latch prima del commit
			8'd133: seq_reg = 9'h0A4;  8'd134: seq_reg = 9'h0A0;
			8'd135: seq_reg = 9'h0A5;  8'd136: seq_reg = 9'h0A1;
			8'd137: seq_reg = 9'h0A6;  8'd138: seq_reg = 9'h0A2;
			8'd139: seq_reg = 9'h0AC;  8'd140: seq_reg = 9'h0A8;
			8'd141: seq_reg = 9'h0AD;  8'd142: seq_reg = 9'h0A9;
			8'd143: seq_reg = 9'h0AE;  8'd144: seq_reg = 9'h0AA;
			// KEY-OFF preventivo ch0-2 (0x110+ch): forza keyon_last -> 0
			8'd145: seq_reg = 9'h110;  8'd146: seq_reg = 9'h111;
			8'd147: seq_reg = 9'h112;
			// KEY-ON ch0-2 (0x100+ch): fronte reale 0->1 -> attacco envelope
			8'd148: seq_reg = 9'h100;  8'd149: seq_reg = 9'h101;
			8'd150: seq_reg = 9'h102;
			// IOA/IOB (0x0E/0x0F): volumi analogici esterni (def_vol_lut)
			8'd151: seq_reg = 9'h00E;  8'd152: seq_reg = 9'h00F;
			// [FIX ottava] ULTIMA op: ripristina latch_fnum del chip scrivendo A4
			// con il dato salvato in 0x103 (ultimo block scritto pre-save). Cosi'
			// un driver post-restore che aggiorna fnum_lo senza riscrivere il block
			// committa con latch_fnum CORRETTO -> nessuna ottava. Marker 0x1FE =
			// "restore latch": registro 0xA4 (solo indirizzo, latcha), dato da 0x103.
			8'd153: seq_reg = 9'h1FE;
			default: seq_reg = 9'h1FF;
		endcase
	end
endfunction

// Durante la pausa ce_cnt non viene azzerato: ce_4m_raw pulsa al wrap del
// contatore (ogni 128 clk). Il BUSY YM dura 32 cen -> 4096 clk: attesa
// 16384 clk tra le operazioni bus = margine 4x.
localparam [13:0] GAP = 14'd16383;

reg  [2:0] rp_state;      // 0 idle, 7 warmup, 1 addr, 2 gap, 3 rd shadow, 4 data, 5 gap, 6 next
reg  [7:0] rp_i;          // indice sequenza
reg        rp_chip;       // 0 = YM1, 1 = YM2
reg [13:0] rp_gap;
reg        mem_read_d;
reg        rp_run;
// [FIX ottava] il chip registra il write ad ogni ce_ym in cui cs&wr sono alti.
// Con cs&wr tenuti ~12k clk il chip vede lo stesso write ~511 volte -> i
// registri edge-sensitive (block/fnum, key-on) si corrompono -> ottava sotto.
// rp_seen alto = il chip ha gia' campionato 1 volta -> rilasciare subito il bus.
reg        rp_seen;

assign replay_busy = rp_run;
assign rp_active   = rp_run;

wire [8:0] cur = seq_reg(rp_i);
// [FIX ottava] marker 0x1FE = restore latch_fnum: scrive registro 0xA4 (latcha),
// dato letto da shadow 0x103 (ultimo block pre-save). NON e' un key-on/off.
wire       is_lfrestore = (cur == 9'h1FE);
// registro YM da indirizzare: key-entry (0x100+, escluso marker) -> 0x28;
// marker -> 0xA4; resto -> registro diretto.
wire [7:0] cur_reg  = is_lfrestore ? 8'hA4 : (cur[8] ? 8'h28 : cur[7:0]);
// key-off entry = cur[8] & cur[4] (0x110+ch); il marker NON e' key-off.
wire       cur_koff = cur[8] & cur[4] & ~is_lfrestore;
// dato per l'entry corrente: key-off -> {4'd0,ch} (opmask 0); resto -> shadow.
wire [7:0] cur_data = cur_koff ? {5'd0, cur[2:0]} : (rp_chip ? sh2_q : sh1_q);

always @(posedge clk) begin
	if (reset) begin
		rp_state <= 3'd0; rp_run <= 1'b0; rp_wr <= 1'b0;
		rp_cs1 <= 1'b0; rp_cs2 <= 1'b0; rp_a0 <= 1'b0; rp_data <= 8'd0;
		rp_i <= 8'd0; rp_chip <= 1'b0; rp_gap <= 14'd0;
		rp_rd_addr <= 9'd0; mem_read_d <= 1'b0; rp_seen <= 1'b0;
	end else begin
		mem_read_d <= ss_mem_read;
		case (rp_state)
			// idle: parte sul fronte di discesa dello stream restore
			3'd0: begin
				rp_wr <= 1'b0; rp_cs1 <= 1'b0; rp_cs2 <= 1'b0;
				if (mem_read_d && !ss_mem_read) begin
					rp_run   <= 1'b1;
					rp_i     <= 8'd0;
					rp_chip  <= 1'b0;
					rp_gap   <= GAP;
					rp_state <= 3'd7;
				end
			end
			// warmup: attesa iniziale (il cen YM riparte col replay attivo)
			3'd7: begin
				if (rp_gap == 14'd0)
					rp_state <= 3'd1;
				else
					rp_gap <= rp_gap - 14'd1;
			end
			// scrittura indirizzo (a0=0)
			3'd1: begin
				rp_a0   <= 1'b0;
				rp_data <= cur_reg;
				rp_cs1  <= ~rp_chip;
				rp_cs2  <=  rp_chip;
				rp_wr   <= 1'b1;
				rp_seen <= 1'b0;   // [FIX ottava] nuovo write: azzera il conteggio ce
				rp_gap  <= GAP;
				rp_state <= 3'd2;
			end
			// gap post-indirizzo. [FIX ottava] il bus scende NON piu' a un clk
			// fisso ma dopo che il chip ha campionato ESATTAMENTE 1 volta (1 ce_ym):
			// rp_seen si alza al primo ce_ym con bus alto, poi il clk dopo si
			// rilascia il bus -> il chip vede 1 solo write (come reale). Il resto
			// del GAP resta come attesa BUSY (bus gia' rilasciato).
			3'd2: begin
				if (ce_ym) rp_seen <= 1'b1;
				if (rp_seen || rp_gap == 14'd0) begin  // fallback: giu' comunque a fine gap
					rp_wr <= 1'b0; rp_cs1 <= 1'b0; rp_cs2 <= 1'b0;
				end
				if (rp_gap == 14'd0) begin
					rp_rd_addr <= is_lfrestore ? 9'h103 : cur;  // [FIX ottava] latch da 0x103
					rp_state   <= 3'd3;
				end else
					rp_gap <= rp_gap - 14'd1;
			end
			// lettura shadow (1 clk di latenza)
			3'd3: rp_state <= 3'd4;
			// scrittura dato (a0=1)
			3'd4: begin
				rp_a0   <= 1'b1;
				rp_data <= cur_data;   // key-off: {4'd0,ch}; resto: shadow
				rp_cs1  <= ~rp_chip;
				rp_cs2  <=  rp_chip;
				rp_wr   <= 1'b1;
				rp_seen <= 1'b0;   // [FIX ottava] nuovo write: azzera il conteggio ce
				rp_gap  <= GAP;
				rp_state <= 3'd5;
			end
			// gap post-dato. [FIX ottava] bus giu' dopo 1 solo ce_ym (vedi state 2).
			3'd5: begin
				if (ce_ym) rp_seen <= 1'b1;
				if (rp_seen || rp_gap == 14'd0) begin  // fallback: giu' comunque a fine gap
					rp_wr <= 1'b0; rp_cs1 <= 1'b0; rp_cs2 <= 1'b0;
				end
				if (rp_gap == 14'd0)
					rp_state <= 3'd6;
				else
					rp_gap <= rp_gap - 14'd1;
			end
			// prossima entry / prossimo chip / fine
			3'd6: begin
				if (rp_i == SEQ_LAST) begin
					if (rp_chip == 1'b0) begin
						rp_chip  <= 1'b1;
						rp_i     <= 8'd0;
						rp_state <= 3'd1;
					end else begin
						rp_run   <= 1'b0;
						rp_state <= 3'd0;
					end
				end else begin
					rp_i     <= rp_i + 8'd1;
					rp_state <= 3'd1;
				end
			end
			default: rp_state <= 3'd0;
		endcase
	end
end

endmodule
