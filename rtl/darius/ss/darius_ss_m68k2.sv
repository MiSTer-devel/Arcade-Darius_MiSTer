/*  This file is part of Darius_MiSTer.
    GPL-3.
    Original author: Martin Donlon (wickerwaka) — Arcade-TaitoF2 savestate system.
    BoogieWings adaptation: Umberto Parisi (rmonic79).
    TWO-68000 (main+sub) variant for Darius: Umberto Parisi (rmonic79), 2026.
*/

//============================================================================
//  darius_ss_m68k2 — savestate of TWO 68000s (main + sub) via mini-handler.
//
//  Derived 1:1 from ss_m68k (BoogieWings/F2): same FSM, same states, but
//  every phase waits for BOTH CPUs:
//    SAVE:    IRQ7 forced to both -> each runs the mini-handler that pushes
//             its registers onto its OWN stack (its own work RAM, saved via
//             the adaptor) and writes its OWN SSP to SS_SCRATCH (0xFF0000,
//             intercepted per-bus) -> 2 SSPs captured -> memory_stream.
//    RESTORE: memory_stream reloads RAM + 2 SSPs -> reset to both with a
//             per-CPU custom reset vector -> handler restart + rte.
//
//  The mini-handler and the vector are IDENTICAL to the original; the only
//  structural difference is the duplicated bus observation/override and the
//  joint wait (both_*). SS_GLOB saves 2 32-bit values (SSP m/s).
//============================================================================

`timescale 1ns / 1ps

module darius_ss_m68k2 #(
	parameter integer SS_GLOB_IDX = 0
) (
	input  wire        clk,
	input  wire        ce_cpu,        // clock-enable lento per il reset counter

	// Bus 68000 MAIN (osservato)
	input  wire [23:0] m_word_addr,
	input  wire [1:0]  m_ds_n,
	input  wire        m_rw,          // 1=read
	input  wire [2:0]  m_fc,
	input  wire        m_iack_n,
	input  wire [15:0] m_data_out,

	// Bus 68000 SUB (osservato)
	input  wire [23:0] s_word_addr,
	input  wire [1:0]  s_ds_n,
	input  wire        s_rw,
	input  wire [2:0]  s_fc,
	input  wire        s_iack_n,
	input  wire [15:0] s_data_out,

	// Trigger (da savestate_ui)
	input  wire        do_save,
	input  wire        do_restore,
	input  wire        paused_real,   // pausa REALE frame-aligned (paused_safe registrato)
	input  wire        vblank_area,   // 1 = render_y in vblank; il save aspetta il FALLING
	input  wire        ext_save_ready, // handler esterni (Z80) pronti: gate pre-stream (1 se non usato)

	// Verso memory_stream (via save_state_data)
	output reg         ss_mem_write,
	output reg         ss_mem_read,
	input  wire        ss_busy,
	input  wire        slot_empty,   // [FIX slot vuoto] load su sotto-slot mai scritto -> non ripartire

	// Stato globale (2 SSP) via ssbus
	ssbus_if.slave     ss_glob,

	// Override verso le CPU (per-CPU)
	output wire        m_din_en,
	output wire [15:0] m_din_data,
	output wire        s_din_en,
	output wire [15:0] s_din_data,
	output wire        ss_irq,        // IPL7 a ENTRAMBE
	output wire        ss_reset,      // reset a ENTRAMBE (restore)
	output wire        ss_pause,
	output wire        ss_cpu_exec,
	output wire        ss_restore_done,
	output reg  [3:0]  ss_state_out
);

localparam [23:0] SS_SCRATCH = 24'hFF0000;

// Mini-handler 68K identico a ss_m68k (F2.sv:187-205)
reg [15:0] ss_irq_handler [0:15];
initial begin
	ss_irq_handler[ 0] = 16'h48e7; ss_irq_handler[ 1] = 16'hfffe; // movem.l d0-d7/a0-a6,-(a7)
	ss_irq_handler[ 2] = 16'h4e6e;                                // move.l usp,a6
	ss_irq_handler[ 3] = 16'h2f0e;                                // move.l a6,-(a7)
	ss_irq_handler[ 4] = 16'h4df9; ss_irq_handler[ 5] = 16'h00ff; ss_irq_handler[ 6] = 16'h0000; // lea 0xff0000,a6
	ss_irq_handler[ 7] = 16'h2c8f;                                // move.l a7,(a6)
	ss_irq_handler[ 8] = 16'h2c5f;                                // move.l (a7)+,a6
	ss_irq_handler[ 9] = 16'h4e66;                                // move.l a6,usp
	ss_irq_handler[10] = 16'h4cdf; ss_irq_handler[11] = 16'h7fff; // movem.l (a7)+,d0-d7/a0-a6
	ss_irq_handler[12] = 16'h4e73;                                // rte
	ss_irq_handler[13] = 16'h0000;
	ss_irq_handler[14] = 16'h0000;
	ss_irq_handler[15] = 16'h0000;
end

reg [31:0] ssp_saved_m, ssp_saved_s;
reg [31:0] ssp_rest_m,  ssp_rest_s;
reg [15:0] rst_vec_m [0:3];
reg [15:0] rst_vec_s [0:3];

// Stato globale via ssbus: 2 elementi 32-bit (addr 0 = main, addr 1 = sub)
always @(posedge clk) begin
	ss_glob.setup(SS_GLOB_IDX, 2, 2);
	if (ss_glob.access(SS_GLOB_IDX)) begin
		if (ss_glob.read)
			ss_glob.read_response(SS_GLOB_IDX, {32'd0, ss_glob.addr[0] ? ssp_saved_s : ssp_saved_m});
		else if (ss_glob.write) begin
			if (ss_glob.addr[0]) ssp_rest_s <= ss_glob.data[31:0];
			else                 ssp_rest_m <= ss_glob.data[31:0];
			ss_glob.write_ack(SS_GLOB_IDX);
		end
	end
end

// FSM identica a ss_m68k, con attese congiunte
localparam [3:0]
	SST_IDLE               = 4'd0,
	SST_SAVE_WAIT_PAUSE    = 4'd8,
	SST_SAVE_WAIT_IRQ      = 4'd1,
	SST_SAVE_WAIT_SSP      = 4'd2,
	SST_SAVE_WAIT_WRITE    = 4'd3,
	SST_SAVE_WAIT_EXIT     = 4'd4,
	SST_RESTORE_WAIT_PAUSE = 4'd9,
	SST_RESTORE_WAIT_READ  = 4'd5,
	SST_RESTORE_HOLD_RST   = 4'd6,
	SST_RESTORE_WAIT_RST   = 4'd7;

reg [3:0] ss_state = SST_IDLE;
reg [4:0] ss_reset_counter;
reg [3:0] ss_state_d;

// Flag per-CPU dentro le fasi congiunte
reg m_irq_taken,  s_irq_taken;    // IACK visto
reg m_ssp_done,   s_ssp_done;     // SSP catturato
reg m_exited,     s_exited;       // tornata a codice normale

wire ss_override = (ss_state != SST_IDLE);
wire m_ds_active = ~&m_ds_n;
wire s_ds_active = ~&s_ds_n;

wire m_saven  = (m_word_addr[23:8] == 16'hFF00);
wire m_resetn = (m_word_addr[23:4] == 20'h00000);
wire s_saven  = (s_word_addr[23:8] == 16'hFF00);
wire s_resetn = (s_word_addr[23:4] == 20'h00000);

wire ss_save_active    = (ss_state == SST_SAVE_WAIT_IRQ) || (ss_state == SST_SAVE_WAIT_SSP) ||
                         (ss_state == SST_SAVE_WAIT_EXIT);
wire ss_restore_active = (ss_state == SST_RESTORE_HOLD_RST) || (ss_state == SST_RESTORE_WAIT_RST);

// Override data-in per-CPU (handler / vettore autovector liv7 / reset-vector)
wire m_sel_save = ss_override & m_ds_active & m_saven;
wire m_sel_vec  = ss_override & m_ds_active & (m_word_addr[23:2] == 22'h00001F);
wire m_sel_rst  = ss_override & m_ds_active & m_resetn;
wire s_sel_save = ss_override & s_ds_active & s_saven;
wire s_sel_vec  = ss_override & s_ds_active & (s_word_addr[23:2] == 22'h00001F);
wire s_sel_rst  = ss_override & s_ds_active & s_resetn;

assign m_din_en   = m_sel_save | m_sel_vec | m_sel_rst;
assign m_din_data = m_sel_save ? ss_irq_handler[m_word_addr[4:1]] :
                    m_sel_rst  ? rst_vec_m[m_word_addr[2:1]] :
                    m_sel_vec  ? (m_word_addr[1] ? 16'h0000 : 16'h00FF) : 16'h0000;
assign s_din_en   = s_sel_save | s_sel_vec | s_sel_rst;
assign s_din_data = s_sel_save ? ss_irq_handler[s_word_addr[4:1]] :
                    s_sel_rst  ? rst_vec_s[s_word_addr[2:1]] :
                    s_sel_vec  ? (s_word_addr[1] ? 16'h0000 : 16'h00FF) : 16'h0000;

assign ss_irq      = (ss_state == SST_SAVE_WAIT_IRQ);
assign ss_reset    = (ss_state == SST_RESTORE_HOLD_RST);
assign ss_pause    = (ss_state != SST_IDLE);
assign ss_cpu_exec = ss_save_active | ss_restore_active;

always @(posedge clk) begin
	ss_state_d   <= ss_state;
	ss_state_out <= ss_state;
end

assign ss_restore_done = (ss_state == SST_IDLE) && (ss_state_d == SST_RESTORE_WAIT_RST);

// Rilevazioni per-CPU (valide dentro gli stati che le usano)
wire m_iack_hit = ~m_iack_n & (m_word_addr[3:1] == 3'b111) & ~m_ds_n[0];
wire s_iack_hit = ~s_iack_n & (s_word_addr[3:1] == 3'b111) & ~s_ds_n[0];
wire m_normal_fetch = (m_ds_n == 2'b00) && m_rw && (m_fc == 3'b110) && ~m_saven;
wire s_normal_fetch = (s_ds_n == 2'b00) && s_rw && (s_fc == 3'b110) && ~s_saven;

// Falling di vblank = inizio frame attivo = la VBL-ISR del gioco (che
// ricompila la lista sprite in sprite RAM) e' gia' conclusa -> stato oggetti
// stabile. Il save inietta l'IRQ7 SOLO qui: se entrasse a inizio vblank (dove
// la ISR sta scrivendo la lista) il mini-handler la spezzerebbe a meta' -> la
// snoop-copy OBJ resta parziale -> nemici/esplosioni congelati (bug al SAVE).
// NB: gate INTERNO alla FSM del save, NON tocca paused_safe (che gate anche
// l'audio: spostarne la fase rompe gli spari ADPCM).
reg vbl_area_d;
always @(posedge clk) vbl_area_d <= vblank_area;
wire vbl_falling = vbl_area_d & ~vblank_area;

always @(posedge clk) begin
	ss_mem_write <= 1'b0;

	case (ss_state)
		SST_IDLE: begin
			ss_mem_read <= 1'b0;
			m_irq_taken <= 0; s_irq_taken <= 0;
			m_ssp_done  <= 0; s_ssp_done  <= 0;
			m_exited    <= 0; s_exited    <= 0;
			if (do_save)    ss_state <= SST_SAVE_WAIT_PAUSE;
			if (do_restore) ss_state <= SST_RESTORE_WAIT_PAUSE;
		end

		SST_SAVE_WAIT_PAUSE: begin
			// pausa frame-aligned attiva E siamo appena entrati nel frame
			// attivo (VBL-ISR conclusa) -> sicuro iniettare IRQ7.
			if (paused_real && vbl_falling) ss_state <= SST_SAVE_WAIT_IRQ;
		end

		SST_RESTORE_WAIT_PAUSE: begin
			if (paused_real) begin
				ss_mem_read <= 1'b1;
				ss_state <= SST_RESTORE_WAIT_READ;
			end
		end

		// SAVE: IRQ7 forzato a entrambe; attende che ENTRAMBE prendano l'IACK
		SST_SAVE_WAIT_IRQ: begin
			if (m_iack_hit) m_irq_taken <= 1'b1;
			if (s_iack_hit) s_irq_taken <= 1'b1;
			if ((m_irq_taken | m_iack_hit) && (s_irq_taken | s_iack_hit))
				ss_state <= SST_SAVE_WAIT_SSP;
		end

		// Cattura i DUE SSP scritti a SS_SCRATCH (ciascuno sul proprio bus)
		SST_SAVE_WAIT_SSP: begin
			if (m_ds_n == 2'b00 && !m_rw && m_word_addr == SS_SCRATCH)
				ssp_saved_m[31:16] <= m_data_out;
			if (m_ds_n == 2'b00 && !m_rw && m_word_addr == (SS_SCRATCH | 24'd2)) begin
				ssp_saved_m[15:0] <= m_data_out;
				m_ssp_done <= 1'b1;
			end
			if (s_ds_n == 2'b00 && !s_rw && s_word_addr == SS_SCRATCH)
				ssp_saved_s[31:16] <= s_data_out;
			if (s_ds_n == 2'b00 && !s_rw && s_word_addr == (SS_SCRATCH | 24'd2)) begin
				ssp_saved_s[15:0] <= s_data_out;
				s_ssp_done <= 1'b1;
			end
			// ext_save_ready: gli handler esterni (Z80) devono aver finito
			// di catturare il loro stato PRIMA che parta lo stream.
			if (m_ssp_done && s_ssp_done && ext_save_ready) begin
				ss_mem_write <= 1'b1;
				ss_state <= SST_SAVE_WAIT_WRITE;
			end
		end

		SST_SAVE_WAIT_WRITE: begin
			if (~ss_busy & ~ss_mem_write) ss_state <= SST_SAVE_WAIT_EXIT;
		end

		// Attende che ENTRAMBI gli handler facciano rte (fetch su codice normale)
		SST_SAVE_WAIT_EXIT: begin
			if (m_normal_fetch) m_exited <= 1'b1;
			if (s_normal_fetch) s_exited <= 1'b1;
			if ((m_exited | m_normal_fetch) && (s_exited | s_normal_fetch))
				ss_state <= SST_IDLE;
		end

		SST_RESTORE_WAIT_READ: begin
			if (ss_busy & ss_mem_read) begin
				ss_mem_read <= 1'b0;
			end else if (~ss_busy & ~ss_mem_read) begin
				// [FIX slot vuoto] sotto-slot mai scritto: nessuno scatter e' avvenuto
				// (stato di gioco INTATTO). NON caricare il reset vector e NON resettare
				// le CPU: torna a IDLE lasciando il gioco a girare = load trasparente.
				if (slot_empty) begin
					ss_state <= SST_IDLE;
				end else begin
					rst_vec_m[0] <= ssp_rest_m[31:16];
					rst_vec_m[1] <= ssp_rest_m[15:0];
					rst_vec_m[2] <= 16'h00FF;   // PC reset = 0x00FF0008 (restart handler)
					rst_vec_m[3] <= 16'h0008;
					rst_vec_s[0] <= ssp_rest_s[31:16];
					rst_vec_s[1] <= ssp_rest_s[15:0];
					rst_vec_s[2] <= 16'h00FF;
					rst_vec_s[3] <= 16'h0008;
					ss_reset_counter <= 5'd0;
					ss_state <= SST_RESTORE_HOLD_RST;
				end
			end
		end

		SST_RESTORE_HOLD_RST: begin
			if (ce_cpu) begin
				ss_reset_counter <= ss_reset_counter + 5'd1;
				if (&ss_reset_counter) ss_state <= SST_RESTORE_WAIT_RST;
			end
		end

		// Attende che ENTRAMBE ripartano su codice normale (fuori da save+reset range)
		SST_RESTORE_WAIT_RST: begin
			if (m_normal_fetch && ~m_resetn) m_exited <= 1'b1;
			if (s_normal_fetch && ~s_resetn) s_exited <= 1'b1;
			if ((m_exited | (m_normal_fetch && ~m_resetn)) &&
			    (s_exited | (s_normal_fetch && ~s_resetn)))
				ss_state <= SST_IDLE;
		end

		default: ss_state <= SST_IDLE;
	endcase
end

endmodule
