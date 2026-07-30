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

// darius_audio_z80 — Sottosistema audio.
// Z80 #1 (main audio): YM2203 ×2 + PC060HA slave + ROM bank-switched.
// Z80 #2 (ADPCM): MSM5205 ADPCM, ROM-only (no RAM).
// Memory maps e I/O per comunicazione con Main CPU via PC060HA.
//
// MAME memory maps:
//   Z80 #1: 0000-3FFF ROM (fixed), 4000-7FFF ROM (banked, 4×16KB),
//           8000-8FFF RAM (4KB), 9000-9001 YM2203#1, A000-A001 YM2203#2,
//           B000 PC060HA port_w, B001 PC060HA comm R/W,
//           C000/C400/C800/CC00/D000 pan regs, D400 ADPCM cmd, DC00 bank switch
//   Z80 #2: 0000-FFFF ROM (flat, no RAM), I/O port 00 = ADPCM command from Z80#1

module darius_audio_z80 (
	input  wire        clk,          // 96 MHz system clock
	input  wire        reset,
	input  wire        pause,        // halt Z80+YM when high (keeps sync with main)
	input  wire  [1:0] clk_sel,      // 00=4MHz, 01=8MHz, 10=2MHz, 11=1MHz

	// ROM download (ioctl) — scrive le ROM Z80 in DDR3 durante il load MRA
	input  wire        ioctl_download,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,
	output wire        ioctl_wait,    // stallo HPS finche' la write DDR3 e' pendente

	// DDRAM HPS (passthrough dal top, adapter istanziato qui dentro)
	input  wire        DDRAM_BUSY,
	output wire  [7:0] DDRAM_BURSTCNT,
	output wire [28:0] DDRAM_ADDR,
	input  wire [63:0] DDRAM_DOUT,
	input  wire        DDRAM_DOUT_READY,
	output wire        DDRAM_RD,
	output wire [63:0] DDRAM_DIN,
	output wire  [7:0] DDRAM_BE,
	output wire        DDRAM_WE,

	// PC060HA sound side — directly to/from PC060HA
	output wire        snd_cs,        // CS active during B000-B001 access
	output wire        snd_addr,      // 0=port (B000), 1=comm (B001)
	output wire        snd_wr,
	output wire        snd_rd,
	output wire  [7:0] snd_wdata,
	input  wire  [7:0] snd_rdata,
	input  wire        snd_nmi_n,     // NMI from PC060HA (active low)
	input  wire        snd_reset_in,  // reset from PC060HA

	// Audio output
	output reg signed [15:0] audio_l,
	output reg signed [15:0] audio_r,

	// =====================================================================
	// [SS-HOOK] Savestate — unici punti di aggancio del modulo audio.
	// Rimozione: cancellare questo blocco porte e i punti [SS-HOOK] sotto;
	// nel top: ss_zram_we/addr/wdata = ss_zram_cpu_*, ss_amisc_ld = 0.
	// Chip (T80/jt03/jt5205) NON toccati.
	// =====================================================================
	output wire        ss_zram_cpu_we,    // lato CPU della porta write ZRAM
	output wire [11:0] ss_zram_cpu_addr,
	output wire  [7:0] ss_zram_cpu_wdata,
	input  wire        ss_zram_we,        // lato RAM (interposto dal top)
	input  wire [11:0] ss_zram_addr,
	input  wire  [7:0] ss_zram_wdata,
	output wire  [7:0] ss_zram_q,
	input  wire        ss_amisc_ld,       // load registri misc (solo restore)
	input  wire [55:0] ss_amisc_in,
	output wire [55:0] ss_amisc_out,
	// [SS-HOOK] Fase B: snoop scritture YM + injector replay (chip vanilla).
	// Rimozione: ss_ymrp_active=0, ss_ymrp_* qualsiasi.
	output wire        ss_ym1_wr,         // scrittura Z80A->YM1 in corso
	output wire        ss_ym2_wr,
	output wire        ss_ym_a0,          // A0/dato condivisi (bus Z80A)
	output wire  [7:0] ss_ym_wdata,
	input  wire        ss_ymrp_active,    // 1 = injector pilota i bus YM
	input  wire        ss_ymrp_cs1,
	input  wire        ss_ymrp_cs2,
	input  wire        ss_ymrp_a0,
	input  wire  [7:0] ss_ymrp_data,
	input  wire        ss_ymrp_wr,
	output wire        ss_ce_ym,          // [FIX ottava] tick campionamento jt03 (ce_4m_ym)
	// [SS-HOOK] Fase C: CPU Z80 instrumentate (tv80s auto_ss, 358 bit x2).
	// Rimozione SS: adaptor nel top a riposo -> auto_ss_wr=0 = trasparente.
	input  wire [357:0] ss_z80a_ssin,
	output wire [357:0] ss_z80a_ssout,
	input  wire         ss_z80a_sswr,
	input  wire [357:0] ss_z80b_ssin,
	output wire [357:0] ss_z80b_ssout,
	input  wire         ss_z80b_sswr,
	// OSD mixer gain: 9 selettori 4-bit (sel=0 Default -> mixer invariato)
	input  wire [3:0]  mix_sel_fm0, mix_sel_fm1,
	input  wire [3:0]  mix_sel_p0a, mix_sel_p0b, mix_sel_p0c,
	input  wire [3:0]  mix_sel_p1a, mix_sel_p1b, mix_sel_p1c,
	input  wire [3:0]  mix_sel_msm,
	// [SS-HOOK] impulso a fine restore: azzera le fasi dei clock-enable audio
	// (ce_cnt/msm_ce_cnt) -> ripartenza deterministica. A 0 = trasparente.
	input  wire        ss_restore_release
);

// =====================================================================
// Clock enable: selectable Z80 speed from 96 MHz
// =====================================================================
reg [6:0] ce_cnt;
reg [6:0] ce_div;

always @(*) case (clk_sel)
	2'd0: ce_div = 7'd24;   // 96/24 = 4 MHz (original)
	2'd1: ce_div = 7'd12;   // 96/12 = 8 MHz
	2'd2: ce_div = 7'd48;   // 96/48 = 2 MHz
	2'd3: ce_div = 7'd96;   // 96/96 = 1 MHz
endcase

wire ce_4m_raw   = (ce_cnt == ce_div - 7'd1);
wire ce_4m_n_raw = (ce_cnt == (ce_div >> 1) - 7'd1);
wire ce_4m   = ce_4m_raw   & ~pause;
wire ce_4m_n = ce_4m_n_raw & ~pause;
// [SS-HOOK] cen YM vivo durante il replay (Z80 restano fermi su ce_4m)
wire ce_4m_ym = ce_4m_raw & (~pause | ss_ymrp_active);
assign ss_ce_ym = ce_4m_ym;   // [FIX ottava] esposto al replay per limitare il write a 1 ce

always @(posedge clk) begin
	if (reset)
		ce_cnt <= 0;
	else if (ss_restore_release)   // [SS-HOOK] fase deterministica post-restore
		ce_cnt <= 0;
	else
		ce_cnt <= ce_4m ? 7'd0 : ce_cnt + 7'd1;
end

// =====================================================================
// Audio ROM in DDR3 (Fase 1 refactoring — pattern Darius2/Sorgelig).
// Download ioctl -> write port adapter; fetch Z80 -> read port con stallo
// wait_n (req/ack toggle). Cache 8-byte + prefetch nell'adapter: il codice
// sequenziale gira quasi sempre in hit, i salti costano ~10-20 clk @96MHz,
// ben dentro il ciclo Z80 a 4MHz (24 clk). Libera ~128KB di M10K.
//
// Mappa DDR3 (byte addr, offset 0x30000000 lo mette l'adapter):
//   0x000000-0x00FFFF: Z80A ROM (64KB, bank-switched lato Z80)
//   0x010000-0x01FFFF: Z80B ROM (64KB flat)
// =====================================================================
localparam [26:0] Z80A_ROM_BASE = 27'h1C8000;
localparam [26:0] Z80B_ROM_BASE = 27'h1D8000;

wire z80a_rom_dl = ioctl_download && ioctl_wr &&
                   (ioctl_addr >= Z80A_ROM_BASE) && (ioctl_addr < Z80B_ROM_BASE);
wire z80b_rom_dl = ioctl_download && ioctl_wr &&
                   (ioctl_addr >= Z80B_ROM_BASE) && (ioctl_addr < (Z80B_ROM_BASE + 27'h10000));

// Write side: word write (WIDE=1) all'offset DDR3 della ROM
reg         ddr_we_req = 0;
wire        ddr_we_ack;
reg  [27:0] ddr_wraddr;
reg  [15:0] ddr_wdata;

wire [16:0] z80a_rom_dl_off = ioctl_addr - Z80A_ROM_BASE;
wire [16:0] z80b_rom_dl_off = ioctl_addr - Z80B_ROM_BASE;

// Edge detect su ioctl_wr (pattern Darius2): 1 toggle per word scritta.
// Solo Z80#1 (musica) va in DDR3; Z80#2 (FX) va in BRAM (blocco sotto).
reg ioctl_wr_prev = 1'b0;
always @(posedge clk) begin
	ioctl_wr_prev <= ioctl_wr;
	if (ioctl_wr && !ioctl_wr_prev && z80a_rom_dl) begin
		ddr_wraddr <= {12'd0, z80a_rom_dl_off[15:0]};
		ddr_wdata  <= ioctl_dout;
		ddr_we_req <= ~ddr_we_req;
	end
end
assign ioctl_wait = (ddr_we_req != ddr_we_ack);

wire [7:0] z80a_rom_q;   // dal read port 1 dell'adapter (musica, resta in DDR3)

// =====================================================================
// Z80 #2 ROM (FX/ADPCM) in BRAM M10K — spostata FUORI dalla DDR3 per
// eliminare la contesa DDR3 (video+audio+savestate): latenza fissa, nessun
// wait, il feed campioni ADPCM non glitcha piu' sotto carico. 64KB flat.
// Download: ioctl scrive word (16b) -> 2 byte in BRAM. Runtime: read byte.
// =====================================================================
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80b_rom_lo [0:32767];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] z80b_rom_hi [0:32767];
wire [14:0] z80b_rom_dl_word = z80b_rom_dl_off[15:1];   // indice word 0..32767
always @(posedge clk) begin
	if (ioctl_wr && !ioctl_wr_prev && z80b_rom_dl) begin
		z80b_rom_lo[z80b_rom_dl_word] <= ioctl_dout[7:0];
		z80b_rom_hi[z80b_rom_dl_word] <= ioctl_dout[15:8];
	end
end
reg  [7:0] z80b_rom_q;   // lettura BRAM (registrata, latenza 1 clk)
always @(posedge clk)
	z80b_rom_q <= z80b_addr[0] ? z80b_rom_hi[z80b_addr[15:1]]
	                           : z80b_rom_lo[z80b_addr[15:1]];

// =====================================================================
// Z80 #1 — Main audio CPU
// =====================================================================
wire [15:0] z80a_addr;
wire  [7:0] z80a_dout;
reg   [7:0] z80a_din;
wire        z80a_mreq_n, z80a_iorq_n, z80a_rd_n, z80a_wr_n;
wire        z80a_m1_n, z80a_rfsh_n;
wire        z80a_wait_n;         // stallo su fetch ROM DDR3 pendente (assegnato sotto)

// Forward declaration for YM2203 IRQ
wire       ym1_irq_n;

// PC060HA NMI: active low, directly from PC060HA.
// tv80s ri-arma l'edge-detect NMI (Oldnmi_n) solo se vede nmi_n ALTO in un
// tick ce_4m; T80pa lo faceva a clk pieno. Tra l'ack dello Z80 (snd_full->0,
// nmi_n sale) e il comando successivo del 68k (nmi_n scende) la finestra alta
// puo' durare <24 clk (68k su griglia /12, comandi back-to-back PAN+SE):
// finestra persa -> edge del comando dopo mai visto -> comando musicale perso
// (frase/strumento mai avviato). Tutto e' derivato dallo stesso 96MHz ->
// race deterministica -> mancano sempre le stesse frasi. Fix speculare a
// msm_nmi_hold: ogni fase alta catturata a clk pieno e presentata per almeno
// un tick ce_4m prima di mostrare il basso (= equivalenza T80pa; +1 tick di
// latenza NMI, 250ns, irrilevante). Nessun edge spurio: scatta solo se
// snd_nmi_n e' realmente basso (comando pendente).
reg nmi_rearm_pend;
always @(posedge clk) begin
	if (z80a_reset)     nmi_rearm_pend <= 1'b0;
	else if (snd_nmi_n) nmi_rearm_pend <= 1'b1;
	else if (ce_4m)     nmi_rearm_pend <= 1'b0;
end
wire z80a_nmi_n = snd_nmi_n | nmi_rearm_pend;

// YM2203 #1 IRQ → Z80 #1 INT
wire z80a_int_n = ym1_irq_n;

// Z80 #1 reset: global reset OR PC060HA sub-reset (MAME: only main audio CPU)
wire z80a_reset = reset | snd_reset_in;

// tv80s instrumentato auto_ss (savestate rodato F2/NightSlashers). cen singolo:
// avanza 1 T-state per ce_4m, ESATTAMENTE come T80pa (CEN_p clocca il core,
// CEN_n era solo ausiliario per gli strobe interni). Mode=0 obbligatorio
// (default di tv80s; NON overridare) = M1 a 4 T-state + refresh come Z80 reale.
tv80s z80_audio (
	.reset_n (~z80a_reset),
	.clk     (clk),
	.cen     (ce_4m),
	.wait_n  (z80a_wait_n),
	.int_n   (z80a_int_n),
	.nmi_n   (z80a_nmi_n),
	.busrq_n (1'b1),
	.m1_n    (z80a_m1_n),
	.mreq_n  (z80a_mreq_n),
	.iorq_n  (z80a_iorq_n),
	.rd_n    (z80a_rd_n),
	.wr_n    (z80a_wr_n),
	.rfsh_n  (z80a_rfsh_n),
	.halt_n  (),
	.busak_n (),
	.A       (z80a_addr),
	.di      (z80a_din),
	.dout    (z80a_dout),
	.auto_ss_in  (ss_z80a_ssin),
	.auto_ss_out (ss_z80a_ssout),
	.auto_ss_wr  (ss_z80a_sswr)
);

// --- Bank switch register ---
reg [1:0] audio_bank;
always @(posedge clk) begin
	if (reset)
		audio_bank <= 2'd0;
	else if (ss_amisc_ld)   // [SS-HOOK]
		audio_bank <= ss_amisc_in[55:54];
	else if (!z80a_mreq_n && !z80a_wr_n && z80a_rfsh_n && z80a_addr == 16'hDC00)
		audio_bank <= z80a_dout[1:0];
end

// --- ADPCM command register (Z80#1 → Z80#2) ---
reg [7:0] adpcm_cmd;
wire      z80b_reads_cmd;  // forward declaration

always @(posedge clk) begin
	if (reset)
		adpcm_cmd <= 8'd0;
	else if (ss_amisc_ld)   // [SS-HOOK]
		adpcm_cmd <= ss_amisc_in[53:46];
	else if (!z80a_mreq_n && !z80a_wr_n && z80a_rfsh_n && z80a_addr == 16'hD400)
		adpcm_cmd <= z80a_dout;
end

// --- Pan registers (stub — just capture, no routing yet) ---
reg [7:0] pan_fm0, pan_fm1, pan_psg0, pan_psg1, pan_da;
always @(posedge clk) begin
	if (reset) begin
		pan_fm0 <= 8'h80; pan_fm1 <= 8'h80;
		pan_psg0 <= 8'h80; pan_psg1 <= 8'h80; pan_da <= 8'h88;  // nibble L=8, R=8 (centered)
	end else if (ss_amisc_ld) begin   // [SS-HOOK]
		pan_fm0  <= ss_amisc_in[45:38];
		pan_fm1  <= ss_amisc_in[37:30];
		pan_psg0 <= ss_amisc_in[29:22];
		pan_psg1 <= ss_amisc_in[21:14];
		pan_da   <= ss_amisc_in[13:6];
	end else if (!z80a_mreq_n && !z80a_wr_n && z80a_rfsh_n) begin
		case (z80a_addr)
			16'hC000: pan_fm0  <= z80a_dout;
			16'hC400: pan_fm1  <= z80a_dout;
			16'hC800: pan_psg0 <= z80a_dout;
			16'hCC00: pan_psg1 <= z80a_dout;
			16'hD000: pan_da   <= z80a_dout;
		endcase
	end
end

// --- Z80 #1 RAM (4KB) ---
reg  [7:0] z80a_ram [0:4095];
reg  [7:0] z80a_ram_q;
wire       z80a_ram_sel = !z80a_mreq_n && (z80a_addr[15:12] == 4'h8);

// [SS-HOOK] porta write ZRAM esposta al top (a riposo: filo dritto dal top).
// & ~pause: strobe di write congelato a pausa calata riscriverebbe la RAM
// appena restorata -> driver musicale corrotto -> musica non riparte al
// restore. Fix verbatim NS/BW (ns_audio_z80.sv:214, is_ram & mem_wr & ~pause).
assign ss_zram_cpu_we    = z80a_ram_sel && !z80a_wr_n && ~pause;
assign ss_zram_cpu_addr  = z80a_addr[11:0];
assign ss_zram_cpu_wdata = z80a_dout;
assign ss_zram_q         = z80a_ram_q;

always @(posedge clk) begin
	if (ss_zram_we)
		z80a_ram[ss_zram_addr] <= ss_zram_wdata;
	z80a_ram_q <= z80a_ram[ss_zram_addr];
end

// --- Z80 #1 ROM read (DDR3, stallo wait_n con toggle req/ack) ---
wire z80a_rom_sel = !z80a_mreq_n && !z80a_rd_n && z80a_rfsh_n && (z80a_addr < 16'h8000);
wire [15:0] z80a_rom_addr_calc = (z80a_addr < 16'h4000) ? z80a_addr : {audio_bank, z80a_addr[13:0]};

reg         z80a_rd_req = 0;
wire        z80a_rd_ack;
reg  [15:0] z80a_rom_addr_lat;
reg         z80a_rom_sel_prev;
always @(posedge clk) begin
	if (reset) begin
		z80a_rd_req       <= 1'b0;
		z80a_rom_sel_prev <= 1'b0;
		z80a_rom_addr_lat <= 16'd0;
	end else begin
		z80a_rom_sel_prev <= z80a_rom_sel;
		if (z80a_rom_sel && !z80a_rom_sel_prev) begin
			z80a_rom_addr_lat <= z80a_rom_addr_calc;
			z80a_rd_req       <= ~z80a_rd_req;
		end
	end
end

// --- Z80 #1 address decode — PC060HA (B000=port, B001=comm) ---
wire z80a_pc060_sel = !z80a_mreq_n && z80a_rfsh_n &&
                      (z80a_addr == 16'hB000 || z80a_addr == 16'hB001);

assign snd_cs    = z80a_pc060_sel;
assign snd_addr  = z80a_addr[0];
assign snd_wr    = z80a_pc060_sel && !z80a_wr_n;
assign snd_rd    = z80a_pc060_sel && !z80a_rd_n;
assign snd_wdata = z80a_dout;

// --- Z80 #1 YM2203 x2 (jt03) ---
wire z80a_ym1_sel = !z80a_mreq_n && z80a_rfsh_n && (z80a_addr[15:1] == 15'h4800);  // 9000-9001
wire z80a_ym2_sel = !z80a_mreq_n && z80a_rfsh_n && (z80a_addr[15:1] == 15'h5000);  // A000-A001
wire z80a_ym_sel  = z80a_ym1_sel | z80a_ym2_sel;

// [FIX crackle — timing tv80s verificato] Il tv80s tiene wr_n basso per 2 tick
// di cen (T2+T3, wr_n registrato basso in tstate[1] e tstate[2] — vedi
// tv80_auto_ss.v:4180). jt03 (write=!cs&!wr, registra su ce_4m_ym) vedeva OGNI
// write 2 volte -> sotto carico il 2o tick catturava un bus in transizione ->
// registro YM corrotto -> crackle. T80pa (vecchio) aveva timing diverso, 1 vista.
// FIX: bloccare la SECONDA vista. Il dato Z80 e' gia' stabile in T2 (registrato
// dallo Z80 sullo stesso cen). ym_wr_seen si arma DOPO che jt03 ha registrato il
// write al 1o ce_4m_ym; da li' maschera wr_n verso jt03 fino al rilascio dello
// Z80. jt03 registra 1 sola volta con dato stabile. Chip NON toccati.
// Verificato in sim col timing reale: 1 write, dato stabile (scratchpad).
reg ym_wr_seen;
always @(posedge clk) begin
	if (reset)
		ym_wr_seen <= 1'b0;
	else if (z80a_wr_n)                          // Z80 ha rilasciato wr -> riarma
		ym_wr_seen <= 1'b0;
	else if (ce_4m_ym & z80a_ym_sel & ~z80a_wr_n) // jt03 ha appena registrato -> blocca il 2o tick
		ym_wr_seen <= 1'b1;
end
// wr_n verso jt03: dopo la 1a vista -> alto (non registra il 2o tick).
wire z80a_ym_wr_n = z80a_wr_n | ym_wr_seen;

// [SS-HOOK] snoop scritture YM = write EFFETTIVO verso jt03 (1 vista, dato
// stabile): lo shadow/replay riproduce lo stesso dato al restore.
assign ss_ym1_wr   = z80a_ym1_sel && !z80a_ym_wr_n;
assign ss_ym2_wr   = z80a_ym2_sel && !z80a_ym_wr_n;
assign ss_ym_a0    = z80a_addr[0];
assign ss_ym_wdata = z80a_dout;

wire [7:0] ym1_dout, ym2_dout;
wire signed [15:0] ym1_snd, ym2_snd;
wire signed [15:0] ym1_fm, ym2_fm;
wire [9:0] ym1_psg_snd, ym2_psg_snd;
wire [7:0] ym1_psg_a, ym1_psg_b, ym1_psg_c;
wire [7:0] ym2_psg_a, ym2_psg_b, ym2_psg_c;
wire [7:0] ym1_ioa_out, ym1_iob_out;
wire [7:0] ym2_ioa_out, ym2_iob_out;
wire       ym1_ioa_oe, ym1_iob_oe;
wire       ym2_ioa_oe, ym2_iob_oe;

jt03 u_ym1 (
	.rst    (reset),
	.clk    (clk),
	.cen    (ce_4m_ym),   // [SS-HOOK] = ce_4m a riposo
	.din    (ss_ymrp_active ? ss_ymrp_data : z80a_dout),         // [SS-HOOK]
	.addr   (ss_ymrp_active ? ss_ymrp_a0   : z80a_addr[0]),      // [SS-HOOK]
	.cs_n   (ss_ymrp_active ? ~ss_ymrp_cs1 : ~z80a_ym1_sel),     // [SS-HOOK]
	.wr_n   (ss_ymrp_active ? ~ss_ymrp_wr  : z80a_ym_wr_n),      // [FIX crackle] 1 vista
	.dout   (ym1_dout),
	.irq_n  (ym1_irq_n),
	.IOA_in (ym1_ioa_oe ? ym1_ioa_out : 8'hFF),
	.IOB_in (ym1_iob_oe ? ym1_iob_out : 8'hFF),
	.IOA_out(ym1_ioa_out), .IOB_out(ym1_iob_out),
	.IOA_oe (ym1_ioa_oe), .IOB_oe (ym1_iob_oe),
	.psg_A  (ym1_psg_a), .psg_B  (ym1_psg_b), .psg_C  (ym1_psg_c),
	.fm_snd (ym1_fm),
	.psg_snd(ym1_psg_snd),
	.snd    (ym1_snd),
	.snd_sample(),
	.debug_view()
);

jt03 u_ym2 (
	.rst    (reset),
	.clk    (clk),
	.cen    (ce_4m_ym),   // [SS-HOOK] = ce_4m a riposo
	.din    (ss_ymrp_active ? ss_ymrp_data : z80a_dout),         // [SS-HOOK]
	.addr   (ss_ymrp_active ? ss_ymrp_a0   : z80a_addr[0]),      // [SS-HOOK]
	.cs_n   (ss_ymrp_active ? ~ss_ymrp_cs2 : ~z80a_ym2_sel),     // [SS-HOOK]
	.wr_n   (ss_ymrp_active ? ~ss_ymrp_wr  : z80a_ym_wr_n),      // [FIX crackle] 1 vista
	.dout   (ym2_dout),
	.irq_n  (),
	.IOA_in (ym2_ioa_oe ? ym2_ioa_out : 8'hFF),
	.IOB_in (ym2_iob_oe ? ym2_iob_out : 8'hFF),
	.IOA_out(ym2_ioa_out), .IOB_out(ym2_iob_out),
	.IOA_oe (ym2_ioa_oe), .IOB_oe (ym2_iob_oe),
	.psg_A  (ym2_psg_a), .psg_B  (ym2_psg_b), .psg_C  (ym2_psg_c),
	.fm_snd (ym2_fm),
	.psg_snd(ym2_psg_snd),
	.snd    (ym2_snd),
	.snd_sample(),
	.debug_view()
);

// =====================================================================
// Volume registers from YM2203 I/O ports (MAME: write_portA0/B0/A1/B1)
// =====================================================================
// def_vol lookup: 100 / 10^((32 - i*32/15) / 20) for i=0..15
// Precomputed as 7-bit values (0..100)
function automatic [6:0] def_vol_lut;
	input [3:0] idx;
	case (idx)
		4'd0:  def_vol_lut = 7'd0;    // -inf dB
		4'd1:  def_vol_lut = 7'd2;
		4'd2:  def_vol_lut = 7'd3;
		4'd3:  def_vol_lut = 7'd4;
		4'd4:  def_vol_lut = 7'd6;
		4'd5:  def_vol_lut = 7'd8;
		4'd6:  def_vol_lut = 7'd11;
		4'd7:  def_vol_lut = 7'd14;
		4'd8:  def_vol_lut = 7'd19;
		4'd9:  def_vol_lut = 7'd25;
		4'd10: def_vol_lut = 7'd33;
		4'd11: def_vol_lut = 7'd44;
		4'd12: def_vol_lut = 7'd58;
		4'd13: def_vol_lut = 7'd75;
		4'd14: def_vol_lut = 7'd89;
		4'd15: def_vol_lut = 7'd100;
	endcase
endfunction

// Volume directly from YM2203 I/O port outputs (combinatorial, no extra registers)
// MAME: write_portA0 = YM1 IOA, write_portB0 = YM1 IOB, etc.
wire [6:0] vol_fm0   = def_vol_lut(ym1_ioa_out[3:0]);
wire [6:0] vol_psg0a = def_vol_lut(ym1_ioa_out[7:4]);
wire [6:0] vol_psg0b = def_vol_lut(ym1_iob_out[7:4]);
wire [6:0] vol_psg0c = def_vol_lut(ym1_iob_out[3:0]);
wire [6:0] vol_fm1   = def_vol_lut(ym2_ioa_out[3:0]);
wire [6:0] vol_psg1a = def_vol_lut(ym2_ioa_out[7:4]);
wire [6:0] vol_psg1b = def_vol_lut(ym2_iob_out[7:4]);
wire [6:0] vol_psg1c = def_vol_lut(ym2_iob_out[3:0]);

// --- Z80 #1 data bus mux (combinatorial — BRAM already has 1-cycle registered output) ---
always @(*) begin
	if (z80a_rom_sel)
		z80a_din = z80a_rom_q;
	else if (z80a_ram_sel)
		z80a_din = z80a_ram_q;
	else if (z80a_ym1_sel)
		z80a_din = ym1_dout;
	else if (z80a_ym2_sel)
		z80a_din = ym2_dout;
	else if (z80a_pc060_sel)
		z80a_din = snd_rdata;
	else
		z80a_din = 8'hFF;
end

// =====================================================================
// Z80 #2 — ADPCM CPU (ROM only, no RAM)
// =====================================================================
wire [15:0] z80b_addr;
wire  [7:0] z80b_dout;
reg   [7:0] z80b_din;
wire        z80b_mreq_n, z80b_iorq_n, z80b_rd_n, z80b_wr_n;
wire        z80b_m1_n;
wire        z80b_wait_n;         // stallo su fetch ROM DDR3 pendente (assegnato sotto)

// --- MSM5205 (ADPCM) ---
reg [7:0] msm_ce_cnt;
wire      msm_cen = (msm_ce_cnt == 8'd249);
always @(posedge clk) begin
	if (reset) msm_ce_cnt <= 0;
	else if (ss_restore_release) msm_ce_cnt <= 0;   // [SS-HOOK] fase deterministica post-restore
	else msm_ce_cnt <= msm_cen ? 8'd0 : msm_ce_cnt + 8'd1;
end

// NMI enable gate + data write (MAME: port 0x00=nmi_disable, 0x01=nmi_enable, 0x02=adpcm_data)
reg       msm_nmi_en;
wire      msm_irq;
wire      msm_vclk;
wire signed [11:0] msm_snd;

wire z80b_io_wr = !z80b_iorq_n && !z80b_wr_n;
wire z80b_nmi_dis  = z80b_io_wr && z80b_addr[7:0] == 8'h00;
wire z80b_nmi_en   = z80b_io_wr && z80b_addr[7:0] == 8'h01;
wire z80b_adpcm_wr = z80b_io_wr && z80b_addr[7:0] == 8'h02;

always @(posedge clk) begin
	if (reset)
		msm_nmi_en <= 1'b0;
	else if (ss_amisc_ld)   // [SS-HOOK]
		msm_nmi_en <= ss_amisc_in[0];
	else begin
		if (z80b_nmi_dis) msm_nmi_en <= 1'b0;
		if (z80b_nmi_en)  msm_nmi_en <= 1'b1;
	end
end

// MSM5205 data + reset (MAME: m_msm->data_w(data), m_msm->reset_w(!(data & 0x20)))
reg [3:0] msm_din;
reg       msm_reset;
always @(posedge clk) begin
	if (reset) begin
		msm_din   <= 4'd0;
		msm_reset <= 1'b1;
	end else if (ss_amisc_ld) begin   // [SS-HOOK]
		msm_din   <= ss_amisc_in[5:2];
		msm_reset <= ss_amisc_in[1];
	end else if (z80b_adpcm_wr) begin
		msm_din   <= z80b_dout[3:0];
		msm_reset <= !(z80b_dout[5]);
	end
end

// [SS-HOOK] snapshot registri misc verso l'adaptor nel top (56 bit)
assign ss_amisc_out = {audio_bank, adpcm_cmd, pan_fm0, pan_fm1, pan_psg0,
                       pan_psg1, pan_da, msm_din, msm_reset, msm_nmi_en};

jt5205 u_msm5205 (
	.rst    (msm_reset),
	.clk    (clk),
	.cen    (msm_cen),
	.sel    (2'b10),    // S1=1, S0=0 → divide by 48 → 384K/48 = 8KHz (matches MAME)
	.din    (msm_din),
	.sound  (msm_snd),
	.sample (),
	.irq    (msm_irq),
	.vclk_o (msm_vclk)
);

// msm_irq (jt5205 irq = cen_lo) e' un impulso di 1 clk a 96MHz ogni 12000 clk
// (8kHz). tv80s campiona nmi_n solo su cen (ce_4m, 1 clk su 24); T80pa lo
// campionava a clk pieno (T80.vhd edge-detect fuori dal gate CEN) e lo
// catturava sempre. 12000 mod 24 = 0 -> l'impulso e' phase-locked al tick
// ce_4m: fase fissata al reset, se sbagliata l'NMI e' persa ad ogni campione
// (FX muti dal boot, deterministico; il restore la spostava via msm_reset).
// Stretch: latch a clk pieno, rilasciato al primo ce_4m (il tick in cui
// tv80s campiona). Un NMI per impulso, si pulisce da solo entro 1 tick.
reg msm_nmi_hold;
always @(posedge clk) begin
	if (reset)                      msm_nmi_hold <= 1'b0;
	else if (msm_irq && msm_nmi_en) msm_nmi_hold <= 1'b1;
	else if (ce_4m)                 msm_nmi_hold <= 1'b0;
end

wire z80b_rfsh_n;
tv80s z80_adpcm (
	.reset_n (~reset),
	.clk     (clk),
	.cen     (ce_4m),
	.wait_n  (z80b_wait_n),
	.int_n   (1'b1),
	.nmi_n   (~((msm_irq & msm_nmi_en) | msm_nmi_hold)),
	.busrq_n (1'b1),
	.m1_n    (z80b_m1_n),
	.mreq_n  (z80b_mreq_n),
	.iorq_n  (z80b_iorq_n),
	.rd_n    (z80b_rd_n),
	.wr_n    (z80b_wr_n),
	.rfsh_n  (z80b_rfsh_n),
	.halt_n  (),
	.busak_n (),
	.A       (z80b_addr),
	.di      (z80b_din),
	.dout    (z80b_dout),
	.auto_ss_in  (ss_z80b_ssin),
	.auto_ss_out (ss_z80b_ssout),
	.auto_ss_wr  (ss_z80b_sswr)
);

// --- Z80 #2 ROM read: da BRAM (latenza fissa 1 clk, nessuno stallo) ---
// z80b_rom_q e' registrata dalla BRAM (blocco sopra). A 4MHz (24 clk/ciclo Z80)
// 1 clk di latenza e' ampiamente coperto -> wait_n sempre alto (vedi sotto).
wire z80b_rom_sel = !z80b_mreq_n && !z80b_rd_n && z80b_rfsh_n;

// --- Z80 #2 I/O port 0x00: read ADPCM command from Z80 #1 ---
assign z80b_reads_cmd = !z80b_iorq_n && !z80b_rd_n && z80b_addr[7:0] == 8'h00;

// --- Z80 #2 I/O read: port 0x00=cmd, 0x02/0x03=0 (MAME), else 0xFF ---
wire z80b_io_rd = !z80b_iorq_n && !z80b_rd_n;
wire z80b_io_rd_02 = z80b_io_rd && (z80b_addr[7:0] == 8'h02 || z80b_addr[7:0] == 8'h03);

// --- Z80 #2 data bus mux (combinatorial) ---
always @(*) begin
	if (z80b_rom_sel)
		z80b_din = z80b_rom_q;
	else if (z80b_reads_cmd)
		z80b_din = adpcm_cmd;
	else if (z80b_io_rd_02)
		z80b_din = 8'h00;
	else
		z80b_din = 8'hFF;
end

// =====================================================================
// =====================================================================
// Audio sample rate generation (192kHz for JTFRAME filters)
// =====================================================================
reg [8:0] sample_cnt;
wire sample_192k = (sample_cnt == 9'd499);  // 96MHz / 500 = 192kHz
always @(posedge clk) begin
	if (reset) sample_cnt <= 9'd0;
	else sample_cnt <= sample_192k ? 9'd0 : sample_cnt + 9'd1;
end

// =====================================================================
// Per-channel analog reconstruction (JTFRAME modules)
// =====================================================================
// DC removal for PSG (unsigned 8-bit → signed 8-bit)
wire signed [7:0] psg1a_dc, psg1b_dc, psg1c_dc;
wire signed [7:0] psg2a_dc, psg2b_dc, psg2c_dc;

jtframe_dcrm #(.SW(8)) u_dc_p1a(.rst(reset),.clk(clk),.sample(sample_192k),.din(ym1_psg_a),.dout(psg1a_dc));
jtframe_dcrm #(.SW(8)) u_dc_p1b(.rst(reset),.clk(clk),.sample(sample_192k),.din(ym1_psg_b),.dout(psg1b_dc));
jtframe_dcrm #(.SW(8)) u_dc_p1c(.rst(reset),.clk(clk),.sample(sample_192k),.din(ym1_psg_c),.dout(psg1c_dc));
jtframe_dcrm #(.SW(8)) u_dc_p2a(.rst(reset),.clk(clk),.sample(sample_192k),.din(ym2_psg_a),.dout(psg2a_dc));
jtframe_dcrm #(.SW(8)) u_dc_p2b(.rst(reset),.clk(clk),.sample(sample_192k),.din(ym2_psg_b),.dout(psg2b_dc));
jtframe_dcrm #(.SW(8)) u_dc_p2c(.rst(reset),.clk(clk),.sample(sample_192k),.din(ym2_psg_c),.dout(psg2c_dc));

// Pole filters currently bypassed: DC-removed PSG and raw FM/MSM are
// fed directly to the mixer. May be revisited in future versions.
wire signed [15:0] ym1_fm_f = ym1_fm;
wire signed [15:0] ym2_fm_f = ym2_fm;
wire signed [7:0] ym1_psg_a_f = psg1a_dc;
wire signed [7:0] ym1_psg_b_f = psg1b_dc;
wire signed [7:0] ym1_psg_c_f = psg1c_dc;
wire signed [7:0] ym2_psg_a_f = psg2a_dc;
wire signed [7:0] ym2_psg_b_f = psg2b_dc;
wire signed [7:0] ym2_psg_c_f = psg2c_dc;
wire signed [11:0] msm_f = msm_snd;

// =====================================================================
// Audio mixer — MAME-ratio-matched, 3-stage pipeline
// =====================================================================
// Coefficients scaled for jt03 integer output ranges (not MAME floats).
// FM:PSG:MSM ratios match MAME route gains (0.60 : 0.08 : 1.00).
// Division by 65536 (>>>16) instead of /10000. No post-gain ×4.
// Pipeline: stage 1 = products, stage 2 = sum, stage 3 = >>>16 + clamp

// --- Stage 0: pv quantization (MAME-exact: (pan * vol) >> 8) — REGISTERED ---
reg [6:0] pv_fm0_l, pv_fm0_r, pv_fm1_l, pv_fm1_r;
reg [6:0] pv_p0a_l, pv_p0a_r, pv_p0b_l, pv_p0b_r, pv_p0c_l, pv_p0c_r;
reg [6:0] pv_p1a_l, pv_p1a_r, pv_p1b_l, pv_p1b_r, pv_p1c_l, pv_p1c_r;
reg [6:0] pv_da_l, pv_da_r;

always @(posedge clk) begin
	pv_fm0_l <= ({7'd0, pan_fm0} * {8'd0, vol_fm0}) >> 8;
	pv_fm0_r <= ({7'd0, 8'd255 - pan_fm0} * {8'd0, vol_fm0}) >> 8;
	pv_fm1_l <= ({7'd0, pan_fm1} * {8'd0, vol_fm1}) >> 8;
	pv_fm1_r <= ({7'd0, 8'd255 - pan_fm1} * {8'd0, vol_fm1}) >> 8;
	pv_p0a_l <= ({7'd0, pan_psg0} * {8'd0, vol_psg0a}) >> 8;
	pv_p0a_r <= ({7'd0, 8'd255 - pan_psg0} * {8'd0, vol_psg0a}) >> 8;
	pv_p0b_l <= ({7'd0, pan_psg0} * {8'd0, vol_psg0b}) >> 8;
	pv_p0b_r <= ({7'd0, 8'd255 - pan_psg0} * {8'd0, vol_psg0b}) >> 8;
	pv_p0c_l <= ({7'd0, pan_psg0} * {8'd0, vol_psg0c}) >> 8;
	pv_p0c_r <= ({7'd0, 8'd255 - pan_psg0} * {8'd0, vol_psg0c}) >> 8;
	pv_p1a_l <= ({7'd0, pan_psg1} * {8'd0, vol_psg1a}) >> 8;
	pv_p1a_r <= ({7'd0, 8'd255 - pan_psg1} * {8'd0, vol_psg1a}) >> 8;
	pv_p1b_l <= ({7'd0, pan_psg1} * {8'd0, vol_psg1b}) >> 8;
	pv_p1b_r <= ({7'd0, 8'd255 - pan_psg1} * {8'd0, vol_psg1b}) >> 8;
	pv_p1c_l <= ({7'd0, pan_psg1} * {8'd0, vol_psg1c}) >> 8;
	pv_p1c_r <= ({7'd0, 8'd255 - pan_psg1} * {8'd0, vol_psg1c}) >> 8;
	pv_da_l  <= def_vol_lut(pan_da[3:0]);
	pv_da_r  <= def_vol_lut(pan_da[7:4]);
end

// --- Stage 0b: coefficients (coeff × pv) — REGISTERED ---
// Ratios FM:PSG:MSM match MAME route gains (0.60 : 0.08 : 1.00).
// I coefficienti (154/5284/4071 di default) arrivano ora dal modulo isolato
// audio_mixer_gain: default per canale + scaling relativo da OSD. A sel=0
// (Default) coeff_* == valori fissi originali -> mixer BIT-IDENTICO.
// I default vanno cambiati SOLO in audio_mixer_gain.sv (un posto).
wire [15:0] g_fm0, g_fm1;
wire [15:0] g_p0a, g_p0b, g_p0c, g_p1a, g_p1b, g_p1c, g_msm;
audio_mixer_gain u_mixer_gain (
	.sel_fm0(mix_sel_fm0), .sel_fm1(mix_sel_fm1),
	.sel_p0a(mix_sel_p0a), .sel_p0b(mix_sel_p0b), .sel_p0c(mix_sel_p0c),
	.sel_p1a(mix_sel_p1a), .sel_p1b(mix_sel_p1b), .sel_p1c(mix_sel_p1c),
	.sel_msm(mix_sel_msm),
	.coeff_fm0(g_fm0), .coeff_fm1(g_fm1),
	.coeff_p0a(g_p0a), .coeff_p0b(g_p0b), .coeff_p0c(g_p0c),
	.coeff_p1a(g_p1a), .coeff_p1b(g_p1b), .coeff_p1c(g_p1c),
	.coeff_msm(g_msm)
);

// c_* = g(16b) * pv(7b) -> max 23 bit (42272*99=4.184.928). Larghezza [22:0]
// uniforme, contiene default x4 x 200% x pv_max senza troncare (troncare qui
// alterava i rapporti). g e pv-pad entrambi 16b per il prodotto.
reg [22:0] c_fm0_l, c_fm0_r, c_fm1_l, c_fm1_r;
reg [22:0] c_p0a_l, c_p0a_r, c_p0b_l, c_p0b_r, c_p0c_l, c_p0c_r;
reg [22:0] c_p1a_l, c_p1a_r, c_p1b_l, c_p1b_r, c_p1c_l, c_p1c_r;
reg [22:0] c_msm_l, c_msm_r;

always @(posedge clk) begin
	c_fm0_l <= g_fm0 * {9'd0, pv_fm0_l};
	c_fm0_r <= g_fm0 * {9'd0, pv_fm0_r};
	c_fm1_l <= g_fm1 * {9'd0, pv_fm1_l};
	c_fm1_r <= g_fm1 * {9'd0, pv_fm1_r};
	c_p0a_l <= g_p0a * {9'd0, pv_p0a_l};
	c_p0a_r <= g_p0a * {9'd0, pv_p0a_r};
	c_p0b_l <= g_p0b * {9'd0, pv_p0b_l};
	c_p0b_r <= g_p0b * {9'd0, pv_p0b_r};
	c_p0c_l <= g_p0c * {9'd0, pv_p0c_l};
	c_p0c_r <= g_p0c * {9'd0, pv_p0c_r};
	c_p1a_l <= g_p1a * {9'd0, pv_p1a_l};
	c_p1a_r <= g_p1a * {9'd0, pv_p1a_r};
	c_p1b_l <= g_p1b * {9'd0, pv_p1b_l};
	c_p1b_r <= g_p1b * {9'd0, pv_p1b_r};
	c_p1c_l <= g_p1c * {9'd0, pv_p1c_l};
	c_p1c_r <= g_p1c * {9'd0, pv_p1c_r};
	c_msm_l <= g_msm * {9'd0, pv_da_l};
	c_msm_r <= g_msm * {9'd0, pv_da_r};
end

// --- Stage 1: per-channel products (registered) ---
// c_* ora [22:0] (23b). Prodotto campione(signed) × c(23u):
// FM:  16s × 23u → signed [38:0]
// PSG:  9s × 23u → signed [31:0]
// MSM: 12s × 23u → signed [34:0]
reg signed [38:0] s1_fm0_l, s1_fm0_r, s1_fm1_l, s1_fm1_r;
reg signed [31:0] s1_p0a_l, s1_p0a_r, s1_p0b_l, s1_p0b_r, s1_p0c_l, s1_p0c_r;
reg signed [31:0] s1_p1a_l, s1_p1a_r, s1_p1b_l, s1_p1b_r, s1_p1c_l, s1_p1c_r;
reg signed [34:0] s1_msm_l, s1_msm_r;

always @(posedge clk) begin
	s1_fm0_l <= ym1_fm_f * $signed({1'b0, c_fm0_l});
	s1_fm0_r <= ym1_fm_f * $signed({1'b0, c_fm0_r});
	s1_fm1_l <= ym2_fm_f * $signed({1'b0, c_fm1_l});
	s1_fm1_r <= ym2_fm_f * $signed({1'b0, c_fm1_r});
	s1_p0a_l <= ym1_psg_a_f * $signed({1'b0, c_p0a_l});
	s1_p0a_r <= ym1_psg_a_f * $signed({1'b0, c_p0a_r});
	s1_p0b_l <= ym1_psg_b_f * $signed({1'b0, c_p0b_l});
	s1_p0b_r <= ym1_psg_b_f * $signed({1'b0, c_p0b_r});
	s1_p0c_l <= ym1_psg_c_f * $signed({1'b0, c_p0c_l});
	s1_p0c_r <= ym1_psg_c_f * $signed({1'b0, c_p0c_r});
	s1_p1a_l <= ym2_psg_a_f * $signed({1'b0, c_p1a_l});
	s1_p1a_r <= ym2_psg_a_f * $signed({1'b0, c_p1a_r});
	s1_p1b_l <= ym2_psg_b_f * $signed({1'b0, c_p1b_l});
	s1_p1b_r <= ym2_psg_b_f * $signed({1'b0, c_p1b_r});
	s1_p1c_l <= ym2_psg_c_f * $signed({1'b0, c_p1c_l});
	s1_p1c_r <= ym2_psg_c_f * $signed({1'b0, c_p1c_r});
	s1_msm_l <= msm_f * $signed({1'b0, c_msm_l});
	s1_msm_r <= msm_f * $signed({1'b0, c_msm_r});
end

// --- Stage 2: sum (registered) ---
// s1: FM [38:0], PSG [31:0], MSM [34:0]. Sign-extend tutti a [40:0] e somma
// in [40:0] (9 addendi, margine per worst-case x4 x200%).
wire signed [40:0] sum_l_w =
	{{2{s1_fm0_l[38]}}, s1_fm0_l} + {{2{s1_fm1_l[38]}}, s1_fm1_l} +
	{{9{s1_p0a_l[31]}}, s1_p0a_l} + {{9{s1_p0b_l[31]}}, s1_p0b_l} +
	{{9{s1_p0c_l[31]}}, s1_p0c_l} + {{9{s1_p1a_l[31]}}, s1_p1a_l} +
	{{9{s1_p1b_l[31]}}, s1_p1b_l} + {{9{s1_p1c_l[31]}}, s1_p1c_l} +
	{{6{s1_msm_l[34]}}, s1_msm_l};
wire signed [40:0] sum_r_w =
	{{2{s1_fm0_r[38]}}, s1_fm0_r} + {{2{s1_fm1_r[38]}}, s1_fm1_r} +
	{{9{s1_p0a_r[31]}}, s1_p0a_r} + {{9{s1_p0b_r[31]}}, s1_p0b_r} +
	{{9{s1_p0c_r[31]}}, s1_p0c_r} + {{9{s1_p1a_r[31]}}, s1_p1a_r} +
	{{9{s1_p1b_r[31]}}, s1_p1b_r} + {{9{s1_p1c_r[31]}}, s1_p1c_r} +
	{{6{s1_msm_r[34]}}, s1_msm_r};

reg signed [40:0] s2_sum_l, s2_sum_r;
always @(posedge clk) begin
	s2_sum_l <= sum_l_w;
	s2_sum_r <= sum_r_w;
end

// --- Stage 3: >>>16 + SOFT-CLIP (registered) ---
// Volumi (default x4) invariati sotto TH1. Sopra, comprime dolce (>>2 poi >>4)
// verso il fondo scala. DIMENSIONATO sul WORST-CASE REALE della somma di TUTTI
// i canali (FM x2 + PSG x6 + MSM, x4): div puo' arrivare a ~135665 -> con
// questa curva -> ~27979 (< 32767), NESSUNA onda quadra. Il soft-clip
// precedente saturava oltre 100k -> con tanti suoni sommati (135k) tagliava
// netto = onda quadra (bug). Clamp finale solo oltre ~200k (impossibile).
localparam signed [24:0] TH1  = 25'sd16000;
localparam signed [24:0] TH2  = 25'sd40000;
localparam signed [24:0] YTH2 = 25'sd22000;   // TH1 + (TH2-TH1)>>2
wire signed [24:0] div_l = s2_sum_l >>> 16;
wire signed [24:0] div_r = s2_sum_r >>> 16;

function automatic signed [15:0] soft_clip;
	input signed [24:0] x;
	reg  [24:0] a;      // magnitudine
	reg  [24:0] y;
	begin
		a = x[24] ? (~x + 25'sd1) : x;         // abs
		if      (a <= TH1) y = a;
		else if (a <= TH2) y = TH1  + ((a - TH1) >>> 2);   // /4
		else               y = YTH2 + ((a - TH2) >>> 4);   // /16, coda lunga
		if (y > 25'sd32767) y = 25'sd32767;    // hard limit finale (solo >200k, impossibile)
		soft_clip = x[24] ? -y[15:0] : y[15:0];
	end
endfunction

always @(posedge clk) begin
	audio_l <= soft_clip(div_l);
	audio_r <= soft_clip(div_r);
end

// =====================================================================
// Adapter DDR3 (Sorgelig pattern, portato da Darius2).
// Port 1 = Z80A ROM, Port 2 = Z80B ROM. Port 3/4 e copy port inutilizzati.
// Stesso clock (96MHz) su tutto: nessun CDC.
// =====================================================================
assign z80a_wait_n = (z80a_rd_req == z80a_rd_ack);
assign z80b_wait_n = 1'b1;   // Z80#2 ROM in BRAM: sempre pronta, nessuno stallo

darius_ddram u_ddram (
	.DDRAM_CLK       (clk),
	.DDRAM_BUSY      (DDRAM_BUSY),
	.DDRAM_BURSTCNT  (DDRAM_BURSTCNT),
	.DDRAM_ADDR      (DDRAM_ADDR),
	.DDRAM_DOUT      (DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_RD        (DDRAM_RD),
	.DDRAM_DIN       (DDRAM_DIN),
	.DDRAM_BE        (DDRAM_BE),
	.DDRAM_WE        (DDRAM_WE),

	.wraddr  (ddr_wraddr),
	.din     (ddr_wdata),
	.we_byte (1'b0),
	.we_req  (ddr_we_req),
	.we_ack  (ddr_we_ack),

	.rdaddr  ({12'd0, z80a_rom_addr_lat}),
	.dout    (z80a_rom_q),
	.rd_req  (z80a_rd_req),
	.rd_ack  (z80a_rd_ack),

	.rdaddr2 (28'd0),   // Z80#2 ROM spostata in BRAM: porta 2 DDR3 inutilizzata
	.dout2   (),
	.rd_req2 (1'b0),
	.rd_ack2 (),

	.rdaddr3 (28'd0),
	.dout3   (),
	.rd_req3 (1'b0),
	.rd_ack3 (),

	.rdaddr4 (28'd0),
	.dout4   (),
	.rd_req4 (1'b0),
	.rd_ack4 (),

	.cpaddr  (28'd0),
	.cpdout  (),
	.cpwr    (),
	.cpreq   (1'b0),
	.cpbusy  ()
);

endmodule
