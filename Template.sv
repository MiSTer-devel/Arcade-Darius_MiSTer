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

    Derived from the MiSTer core template by Sorgelig (MiSTer-devel).

*/

// Template.sv — Darius (Taito, 1986) top-level wrapper (module `emu`).
// Connects the MiSTer framework (`sys/`) to the Darius core
// (`darius_dual68k_top`): HPS I/O, OSD, SDRAM arbitration, audio/video
// output, ROM download, input routing.

module emu
(
	input         CLK_50M,
	input         RESET,
	inout  [48:0] HPS_BUS,
	output        CLK_VIDEO,
	output        CE_PIXEL,
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER,
	output        VGA_DISABLE,
	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,
`ifdef MISTER_FB_PALETTE
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,
	output  [1:0] BUTTONS,

	input         CLK_AUDIO,
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,
	output  [1:0] AUDIO_MIX,

	inout   [3:0] ADC_BUS,

	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

///////// Unused ports /////////
assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// DDR3: ROM audio Z80 A/B (adapter dentro darius_audio_z80, clock unico 96MHz)
assign DDRAM_CLK = clk_sys;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_SCALER  = 0;
assign VGA_DISABLE = 0;
// Pause: toggle on rising edge of joy[12] (standard MiSTer pause bit)
reg pause_toggle;
reg joy_pause_prev;
always @(posedge clk_sys) begin
	if (reset) begin
		pause_toggle <= 1'b0;
		joy_pause_prev <= 1'b0;
	end else begin
		joy_pause_prev <= joy0[12] | joy1[12];
		if ((joy0[12] | joy1[12]) && !joy_pause_prev)
			pause_toggle <= ~pause_toggle;
	end
end
wire pause_req = pause_toggle;  // solo pad joy[12] (OSD Pause rimosso: bit 17 libero per mixer)

// === Pausa frame-safe (pattern BoogieWings/F2 obj_paused) ===
// La pausa REALE cambia SOLO al rising edge di VBlank: MAI intraframe, su
// nessun chip (68k halt, Z80/YM/MSM cen). Evita race a metà bus-cycle /
// scanline e i freeze storici da pausa. ss_pause (Fase 3 savestate) si
// aggancerà qui: salita frame-aligned, mantenimento continuo durante SS.
wire ss_pause;          // dal sistema savestate (ss_m68k2 nel game top)
reg  vbl_ps_d;
reg  paused_safe;
always @(posedge clk_sys) begin
	if (reset) begin
		vbl_ps_d    <= 1'b0;
		paused_safe <= 1'b0;
	end else begin
		vbl_ps_d <= VBlank;
		if (ss_pause & paused_safe)
			paused_safe <= 1'b1;                 // mantieni per tutta la durata del SS
		else if (VBlank & ~vbl_ps_d)
			paused_safe <= pause_req | ss_pause; // salita/discesa solo a confine frame
	end
end
wire pause = paused_safe;
assign HDMI_FREEZE = 1'b0;  // overlay pause renderizzato real-time, no freeze scaler
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;  // signed audio
wire signed [15:0] game_audio_l, game_audio_r;
assign AUDIO_L = game_audio_l;
assign AUDIO_R = game_audio_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

// OSD layer offsets: 6-bit signed 2's complement, default 0 on reset
wire signed [9:0] osd_l0_xoff  = {{4{status[43]}}, status[43:38]};
wire signed [9:0] osd_l0_yoff  = {{4{status[49]}}, status[49:44]};
wire signed [9:0] osd_l1_xoff  = {{4{status[55]}}, status[55:50]};
wire signed [9:0] osd_l1_yoff  = {{4{status[61]}}, status[61:56]};
wire signed [9:0] osd_spr_xoff = {{4{status[67]}}, status[67:62]};
wire signed [9:0] osd_spr_yoff = {{4{status[73]}}, status[73:68]};
wire signed [9:0] osd_fg_xoff  = {{4{status[79]}}, status[79:74]};
wire signed [9:0] osd_fg_yoff  = {{4{status[85]}}, status[85:80]};

`include "build_id.v"
localparam CONF_STR = {
	"Darius;SS3E000000:200000;",
	"-;",
	"O[108:105],Savestate Slot,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;",
	"R[109],Save state (Alt-F1);",
	"R[110],Restore state (F1);",
	"-;",
	"P1,Video;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[6:5],Scale,Narrower HV-Integer,V-Integer,HV-Integer;",
	"P1O[10:8],Screen View,Triple,Follow,Follow 4:3+,Follow Wide,Screen Jump;",
	"P1O[11],Triple CRT Wide,Off,On;",
	"P1O[86],CRT Adjust,Off,On;",
	"H1P1O[91:87],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[98:92],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"H1P1O[103:99],CRT V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	"O[18],Clean Pause,Off,On;",
	"-;",
	"P2,Audio Mix;",
	"P2-,Gain per canale (Default=base);",
	"P2O[21:19],FM-A (BGM 0),Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[24:22],FM-B (BGM 1),Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[27:25],SSG-A ch A,Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[113:111],SSG-A ch B,Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[116:114],SSG-A ch C,Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[119:117],SSG-B ch A,Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[14:12],SSG-B ch B,Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[17:15],SSG-B ch C,Default,Mute,25%,50%,75%,125%,150%,200%;",
	"P2O[125:123],ADPCM (MSM),Default,Mute,25%,50%,75%,125%,150%,200%;",
	"-;",
	"DIP;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"-;",
	"J1,Fire,Bomb,Start 1P,Start 2P,Coin;",
	"jn,A,B,Start,Select,R;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [15:0] joy0, joy1;
wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;   // 16-bit: WIDE=1
wire        ioctl_wait_sdram;
wire        ioctl_wait_audio;
wire        ioctl_wait = ioctl_wait_sdram | ioctl_wait_audio;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(),
	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask({14'd0, ~status[86], 1'b0}),  // H1 (CRT Adjust) visibile solo se On
	.ps2_key(ps2_key),
	.joystick_0(joy0),
	.joystick_1(joy1),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait)
);

// --- Joystick to Darius input mapping ---
// MiSTer joy bits: 0=R, 1=L, 2=D, 3=U, 4=Fire(A), 5=Bomb(B), 6=Start1P, 7=Start2P, 8=Coin
// Darius P1 ($C00008): active low, bits: 7=unused,6=unused,5=Fire2,4=Fire1,3=U,2=D,1=L,0=R
// Darius P2 ($C0000A): same layout
// Darius SYSTEM ($C0000E): bit0=coin1, bit1=coin2, bit2=service (active low)
// MAME: bit0=UP, bit1=DOWN, bit2=RIGHT, bit3=LEFT, bit4=FIRE, bit5=BOMB (active low)
// MiSTer: joy[0]=R, joy[1]=L, joy[2]=D, joy[3]=U, joy[4]=btn1, joy[5]=btn2
wire [7:0] p1_input = ~{2'b00, joy0[5], joy0[4], joy0[1], joy0[0], joy0[2], joy0[3]};
wire [7:0] p2_input = ~{2'b00, joy1[5], joy1[4], joy1[1], joy1[0], joy1[2], joy1[3]};


// MAME SYSTEM port ($C0000C) bit layout:
//   bit 7:6 = unused (idle=0, IP_ACTIVE_HIGH)
//   bit 5   = start2 (active LOW, idle=1)
//   bit 4   = start1 (active LOW, idle=1)
//   bit 3   = tilt   (active LOW, idle=1) — not mapped, hardcode idle
//   bit 2   = service(active LOW, idle=1) — not mapped, hardcode idle
//   bit 1   = coin2  (active HIGH, idle=0)
//   bit 0   = coin1  (active HIGH, idle=0)
// MiSTer joy bits: [3:0]=R,L,D,U  [4]=A  [5]=B
//                   [10]=Start  [11]=Coin  [12]=Pause
// MAME SYSTEM port ($C0000C) — verified from _mame_darius.cpp lines 753-758:
//   bit 0: COIN1   (active HIGH) ← joy0[11]
//   bit 1: COIN2   (active HIGH) ← joy1[11]
//   bit 2: START1  (active LOW)  ← ~joy0[10]
//   bit 3: START2  (active LOW)  ← ~joy1[10]
//   bit 4: SERVICE (active LOW)  ← idle=1
//   bit 5: TILT    (active LOW)  ← idle=1
//   bit 7:6: unused (=0)
wire [7:0] system_input = {2'b11, 2'b11, ~joy1[10], ~joy0[10], joy1[11], joy0[11]};

// DIP switches — loaded from MRA via ioctl (index 254)
// Active-LOW: default "FF,FF" = all OFF = all 1s
reg [15:0] dip_sw = 16'hFFFF;
always @(posedge clk_sys)
	if (ioctl_wr && (ioctl_index == 16'd254) && !ioctl_addr[26:1])
		dip_sw <= ioctl_dout;

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

// Game reset: includes download (game held in reset while ROM loads)
wire reset = RESET | status[0] | buttons[1] | ~pll_locked | ioctl_download;
// Bridge reset: ONLY pll_locked — bridge must run during download, before RESET drops
wire bridge_reset = ~pll_locked;
// Video reset: ONLY pll_locked — CRT needs sync always, even during RESET and download
wire video_reset = ~pll_locked;

///////////////////////   SDRAM   ///////////////////////////////

// Genesis 4-port SDRAM controller (Sorgelig + port 3 for audio)
// Port 0: Tile ROM + download
// Port 1: Main CPU ROM
// Port 2: Sub CPU ROM
// Port 3: Audio Z80 ROM

wire [24:1] sd_addr0, sd_addr1, sd_addr2, sd_addr3;
wire [15:0] sd_din0, sd_din1, sd_din2, sd_din3;
wire        sd_wrl0, sd_wrh0, sd_wrl1, sd_wrh1, sd_wrl2, sd_wrh2, sd_wrl3, sd_wrh3;
wire        sd_req0, sd_req1, sd_req2, sd_req3;
wire        sd_ack0, sd_ack1, sd_ack2, sd_ack3;
wire [15:0] sd_dout0, sd_dout1, sd_dout2, sd_dout3;
wire        sdram_ready;

sdram sdram_ctrl
(
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

	.init(~pll_locked),
	.clk(clk_sys),
	.prio_mode(2'd0),   // RR Equal fisso (non esposto in OSD; bit 34-35 liberi per mixer)
	.ready(sdram_ready),

	.addr0(sd_addr0), .wrl0(sd_wrl0), .wrh0(sd_wrh0),
	.din0(sd_din0), .dout0(sd_dout0), .req0(sd_req0), .ack0(sd_ack0),

	.addr1(sd_addr1), .wrl1(sd_wrl1), .wrh1(sd_wrh1),
	.din1(sd_din1), .dout1(sd_dout1), .req1(sd_req1), .ack1(sd_ack1),

	.addr2(sd_addr2), .wrl2(sd_wrl2), .wrh2(sd_wrh2),
	.din2(sd_din2), .dout2(sd_dout2), .req2(sd_req2), .ack2(sd_ack2),

	.addr3(sd_addr3), .wrl3(sd_wrl3), .wrh3(sd_wrh3),
	.din3(sd_din3), .dout3(sd_dout3), .req3(sd_req3), .ack3(sd_ack3)
);

///////////////////////   BRIDGE   ///////////////////////////////

// Bridge between darius game logic (level protocol) and Genesis SDRAM (toggle protocol)
wire [23:0] game_tile_addr, game_main_addr, game_sub_addr;
wire        game_tile_req, game_main_req, game_sub_req;
wire        game_tile_is_sprite;
wire        game_tile_is_text;
wire [31:0] game_tile_data;
wire        game_tile_valid;
wire [15:0] game_main_data, game_sub_data;
// Audio Z80 ROM removed from SDRAM — will use BRAM when audio implemented
wire        game_main_ready, game_sub_ready;

// ROM instruction cache — between game and SDRAM bridge
wire [23:0] bridge_main_addr, bridge_sub_addr;
wire        bridge_main_req, bridge_sub_req;
wire [15:0] bridge_main_data, bridge_sub_data;
wire        bridge_main_ready, bridge_sub_ready;

rom_cache #(.CACHE_BITS(9)) u_main_cache (
	.clk(clk_sys), .reset(reset),
	.cpu_addr(game_main_addr), .cpu_req(game_main_req),
	.cpu_data(game_main_data), .cpu_ready(game_main_ready),
	.sdram_addr(bridge_main_addr), .sdram_req(bridge_main_req),
	.sdram_data(bridge_main_data), .sdram_ready(bridge_main_ready)
);

rom_cache #(.CACHE_BITS(9)) u_sub_cache (
	.clk(clk_sys), .reset(reset),
	.cpu_addr(game_sub_addr), .cpu_req(game_sub_req),
	.cpu_data(game_sub_data), .cpu_ready(game_sub_ready),
	.sdram_addr(bridge_sub_addr), .sdram_req(bridge_sub_req),
	.sdram_data(bridge_sub_data), .sdram_ready(bridge_sub_ready)
);

sdram_bridge bridge
(
	.clk(clk_sys),
	.reset(bridge_reset),
	.sdram_ready(sdram_ready),

	// HPS download
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait_sdram),

	// Game: Tile ROM (32-bit)
	.tile_byte_addr(game_tile_addr),
	.tile_req(game_tile_req),
	.tile_is_sprite(game_tile_is_sprite),
	.tile_is_text(game_tile_is_text),
	.tile_data(game_tile_data),
	.tile_valid(game_tile_valid),

	// Game: Main CPU ROM (16-bit)
	.main_byte_addr(bridge_main_addr),
	.main_req(bridge_main_req),
	.main_data(bridge_main_data),
	.main_ready(bridge_main_ready),

	// Game: Sub CPU ROM (16-bit)
	.sub_byte_addr(bridge_sub_addr),
	.sub_req(bridge_sub_req),
	.sub_data(bridge_sub_data),
	.sub_ready(bridge_sub_ready),

	// SDRAM ports
	.sdram_addr0(sd_addr0), .sdram_din0(sd_din0),
	.sdram_wrl0(sd_wrl0), .sdram_wrh0(sd_wrh0),
	.sdram_req0(sd_req0), .sdram_ack0(sd_ack0), .sdram_dout0(sd_dout0),

	.sdram_addr1(sd_addr1), .sdram_din1(sd_din1),
	.sdram_wrl1(sd_wrl1), .sdram_wrh1(sd_wrh1),
	.sdram_req1(sd_req1), .sdram_ack1(sd_ack1), .sdram_dout1(sd_dout1),

	.sdram_addr2(sd_addr2), .sdram_din2(sd_din2),
	.sdram_wrl2(sd_wrl2), .sdram_wrh2(sd_wrh2),
	.sdram_req2(sd_req2), .sdram_ack2(sd_ack2), .sdram_dout2(sd_dout2),

	.sdram_addr3(sd_addr3), .sdram_din3(sd_din3),
	.sdram_wrl3(sd_wrl3), .sdram_wrh3(sd_wrh3),
	.sdram_req3(sd_req3), .sdram_ack3(sd_ack3), .sdram_dout3(sd_dout3)
);

///////////////////////   GAME   ///////////////////////////////

wire [9:0]  render_x;
wire [8:0]  render_y;
wire [23:0] tile_rgb;
wire [1:0]  tile_prio;
wire        tile_opaque;
wire [23:0] game_sprite_rgb;
wire [1:0]  game_sprite_prio;
wire        game_sprite_opaque;
wire [23:0] game_fg_rgb;
wire        game_fg_opaque;
wire [15:0] map_xscroll_l0, map_xscroll_l1;
wire [15:0] map_yscroll_l0, map_yscroll_l1;
wire [9:0]  ship_x;
wire        ship_commit;

darius_dual68k_top game
(
	.clk(clk_sys),
	.reset(reset),
	.pause(pause),
	.clk_sel(3'd0),      // Main CPU 8MHz (default originale; non esposto in OSD -> bit 20-22 liberi)
	.sub_clk_sel(3'd0),  // Sub CPU 8MHz  (default originale; bit 23-25 liberi)
	.z80_clk_sel(2'd0),  // Z80 4MHz      (default originale; bit 36-37 liberi)
	.p1_input(p1_input),
	.p2_input(p2_input),
	.system_input(system_input),
	.dsw_input(dip_sw),

	// SDRAM ROM (via bridge)
	.main_rom_rdata(game_main_data),
	.main_rom_ready(game_main_ready),
	.sub_rom_rdata(game_sub_data),
	.sub_rom_ready(game_sub_ready),
	.tilerom_data(game_tile_data),
	.tilerom_valid(game_tile_valid),

	.main_rom_addr(game_main_addr),
	.main_rom_req(game_main_req),
	.sub_rom_addr(game_sub_addr),
	.sub_rom_req(game_sub_req),
	.tilerom_addr(game_tile_addr),
	.tilerom_req(game_tile_req),
	.tilerom_is_sprite(game_tile_is_sprite),
	.tilerom_is_text(game_tile_is_text),

	// Audio ROM download (ioctl → BRAM)
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),

	// Video
	.render_x(render_x),
	.render_y(render_y),
	.tile_rgb(tile_rgb),
	.tile_prio(tile_prio),
	.tile_opaque(tile_opaque),
	.sprite_rgb(game_sprite_rgb),
	.sprite_prio(game_sprite_prio),
	.sprite_opaque(game_sprite_opaque),
	.ship_x(ship_x),
	.ship_commit(ship_commit),
	.fg_rgb(game_fg_rgb),
	.fg_opaque(game_fg_opaque),

	// Scroll/debug
	.xscroll_l0(map_xscroll_l0),
	.xscroll_l1(map_xscroll_l1),
	.yscroll_l0(map_yscroll_l0),
	.yscroll_l1(map_yscroll_l1),
	.ctrl_l0(),
	.ctrl_l1(),
	// OSD layer offsets
	.l0_xoff(osd_l0_xoff), .l0_yoff(osd_l0_yoff),
	.l1_xoff(osd_l1_xoff), .l1_yoff(osd_l1_yoff),
	.spr_xoff(osd_spr_xoff), .spr_yoff(osd_spr_yoff),
	.fg_xoff(osd_fg_xoff), .fg_yoff(osd_fg_yoff),
	// Text ROM download → FG BRAM
	.fg_dl_wr(ioctl_download && ioctl_wr && ioctl_index == 16'd0 &&
	           ioctl_addr >= 27'h1C0000 && ioctl_addr < 27'h1C8000),
	.fg_dl_addr(ioctl_addr[14:1]),
	.fg_dl_data(ioctl_dout),
	// Audio ROM DDR3
	.audio_ioctl_wait(ioctl_wait_audio),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT),
	.DDRAM_ADDR(DDRAM_ADDR), .DDRAM_DOUT(DDRAM_DOUT),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE),
	// Savestate
	.ss_save(ss_save_t), .ss_load(ss_load_t), .ss_slot(ss_slot),
	.paused_real(paused_safe), .ss_pause_out(ss_pause),
	// OSD mixer gain (9 selettori 4-bit). Default 0 = mixer invariato.
	.mix_sel_fm0(mix_sel_fm0), .mix_sel_fm1(mix_sel_fm1),
	.mix_sel_p0a(mix_sel_p0a), .mix_sel_p0b(mix_sel_p0b), .mix_sel_p0c(mix_sel_p0c),
	.mix_sel_p1a(mix_sel_p1a), .mix_sel_p1b(mix_sel_p1b), .mix_sel_p1c(mix_sel_p1c),
	.mix_sel_msm(mix_sel_msm),
	// Audio
	.audio_l(game_audio_l),
	.audio_r(game_audio_r)
);

// OSD mixer gain — 9 selettori 3-bit su bit liberi contigui (validati con
// tools/status_bit_audit.py: 0 collisioni). Estesi a 4-bit (zero-pad) per il
// modulo audio_mixer_gain: 3 bit = 8 voci (indici 0..7 di osd_mul; 0=Default).
// Bit liberati hardcodando i 3 controlli non-OSD (SDRAM prio, CPU/Z80 clk).
wire [3:0] mix_sel_fm0 = {1'b0, status[21:19]};
wire [3:0] mix_sel_fm1 = {1'b0, status[24:22]};
wire [3:0] mix_sel_p0a = {1'b0, status[27:25]};
wire [3:0] mix_sel_p0b = {1'b0, status[113:111]};
wire [3:0] mix_sel_p0c = {1'b0, status[116:114]};
wire [3:0] mix_sel_p1a = {1'b0, status[119:117]};
wire [3:0] mix_sel_p1b = {1'b0, status[14:12]};
wire [3:0] mix_sel_p1c = {1'b0, status[17:15]};
wire [3:0] mix_sel_msm = {1'b0, status[125:123]};

// === Savestate UI (pattern BoogieWings, 1:1 — slot estesi a 16) ===
wire [3:0]  ss_slot;
wire        ss_save_t, ss_load_t;
wire [15:0] joy_all = joy0 | joy1;
savestate_ui #(.INFO_TIMEOUT_BITS(25)) u_ss_ui (
	.clk         (clk_sys),
	.ps2_key     (ps2_key),
	.allow_ss    (1'b1),
	.joySS       (joy_all[13]),
	.joyRight    (joy_all[0]),
	.joyLeft     (joy_all[1]),
	.joyDown     (joy_all[2]),
	.joyUp       (joy_all[3]),
	.joyStart    (joy_all[12]),
	.joyRewind   (1'b0),
	.rewindEnable(1'b0),
	.status_slot (status[108:105]),
	.autoincslot (1'b0),
	.OSD_saveload(status[110:109]),  // R[109]=save, R[110]=restore
	.ss_save     (ss_save_t),
	.ss_load     (ss_load_t),
	.ss_info_req (),
	.ss_info     (),
	.statusUpdate(),
	.selected_slot(ss_slot)
);

///////////////////////   VIDEO   ///////////////////////////////

// Triple screen video timing via dedicated module (864x224 Darius layout)
wire ce_pix;
wire HBlank, VBlank, HSync, VSync;
wire [7:0] video_r, video_g, video_b;

triple_screen_test u_video (
	.clk(clk_sys),
	.reset(video_reset),
	.layer_en({~status[33], ~status[32], ~status[31], ~status[30]}),  // {FG, SPR, L1, L0}
	.tile_rgb(tile_rgb),
	.tile_prio(tile_prio),
	.tile_opaque(tile_opaque),
	.sprite_pix_rgb(game_sprite_rgb),
	.sprite_prio(game_sprite_prio),
	.sprite_opaque(game_sprite_opaque),
	.fg_rgb(game_fg_rgb),
	.fg_opaque(game_fg_opaque),
	.ce_pix(ce_pix),
	.HBlank(HBlank),
	.HSync(HSync),
	.VBlank(VBlank),
	.VSync(VSync),
	.render_x(render_x),
	.render_y(render_y),
	.R(video_r),
	.G(video_g),
	.B(video_b)
);

// Sample & hold a ce_pix: l'RGB composito viene campionato nel punto sicuro
// del periodo pixel (fine intervallo, pipeline palette/warm-up cambio pannello
// già stabili) e tenuto COSTANTE per l'intero periodo. Rimuove A MONTE la
// colonna spuria ai confini pannello (x=288/576) visibile sul path analogico
// (che emette in continuo, non campiona una volta sola come lo scaler HDMI).
// Sync ritardati insieme all'RGB: relazioni temporali identiche, HDMI invariato.
reg [7:0] vid_r_q, vid_g_q, vid_b_q;
reg       hb_q, vbl_q, hs_q, vs_q;
always @(posedge clk_sys) if (ce_pix) begin
	vid_r_q <= video_r;
	vid_g_q <= video_g;
	vid_b_q <= video_b;
	hb_q    <= HBlank;
	vbl_q   <= VBlank;
	hs_q    <= HSync;
	vs_q    <= VSync;
end

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = crt_on ? rd_ce : fc_ce;
assign VGA_HS    = crt_on ? str_hs : fc_hs;  // follow: HSync ricentrato; CRT Adjust: dal modulo
assign VGA_VS    = crt_on ? str_vs : vs_q;

// Follow-cam: vista single-screen che segue la navicella (status[10:8]);
// [11] = Triple CRT Wide (ri-emissione 16MHz, solo view=Triple)
wire [2:0] follow_view = status[10:8];
wire follow_en = (follow_view != 3'd0);
wire wide3_mode = (follow_view == 3'd0) && status[11];
wire [7:0] fc_r, fc_g, fc_b;
wire       fc_hb, fc_hs, fc_ce;
wire [9:0] fc_ovl_x;

darius_followcam u_followcam (
	.clk        (clk_sys),
	.reset      (video_reset),
	.view       (follow_view),
	.wide3      (status[11]),
	.ce_pix     (ce_pix),
	.HBlank     (hb_q),
	.HSync      (hs_q),
	.VBlank     (vbl_q),
	.render_y   (render_y),
	.render_x   (render_x),
	.ship_x     (ship_x),
	.ship_commit(ship_commit),
	.r_in       (vid_r_q),
	.g_in       (vid_g_q),
	.b_in       (vid_b_q),
	.r_out      (fc_r),
	.g_out      (fc_g),
	.b_out      (fc_b),
	.HBlank_out (fc_hb),
	.HSync_out  (fc_hs),
	.pix_ce     (fc_ce),
	.ovl_x      (fc_ovl_x)
);

// Pause overlay: dim video + logo 48x48 al centro durante pausa.
// OSD "Clean Pause" (status[18]): ON=video raw senza addon, OFF=overlay attivo.
wire [7:0] av_r, av_g, av_b;
pause_overlay u_pause_ovl (
	.clk       (clk_sys),
	.pause     (pause_req & paused_safe),  // SOLO pausa utente: il savestate
	                                       // pausa il gioco ma non mostra overlay
	.clean     (status[18]),
	.render_x  (fc_ovl_x),
	.render_y  (render_y),
	.rgb_r_in  (fc_r),
	.rgb_g_in  (fc_g),
	.rgb_b_in  (fc_b),
	.rgb_r_out (av_r),
	.rgb_g_out (av_g),
	.rgb_b_out (av_b)
);

// ── CRT Adjust (H-Size + H-Position + V-Shift) — portato 1:1 da Legionnaire ──
// Contenuto spostato/scalato nel line buffer, sync nativi -> nessun desync CRT.
// OFF (default) = bypass puro, HDMI intoccato. ON: anche l'HDMI segue (trade-off
// documentato nel modulo).
localparam int H_TOTAL_DAR = 1527;
localparam int V_TOTAL_DAR = 262;

reg crt_on;
always @(posedge clk_sys) if (ce_pix) crt_on <= status[86];

// H-Size (status[91:87], 5-bit two's complement): 0=nativo, +1..+15 enlarge,
// -1..-16 shrink. Base periodo Darius = 16 quarti (4 clk @96M = 24MHz pixel):
// clamp minimo 8 quarti (2 clk) — hsize molto negativo non puo' azzerare il periodo.
reg signed [4:0] hsize_s;
always @(posedge clk_sys) if (ce_pix) hsize_s <= $signed(status[91:87]);

// H-Position (status[98:92], 7 bit): 0..48 = +0..+48 (destra), 79..127 = -48..-1.
// Scala x4: la linea Darius e' 1527 slot (vs 384 Legionnaire) — senza scala
// ogni passo muove ~1/4 del dovuto. x4 -> ±192 slot = ±12.6% linea, passo 0.26%.
reg [6:0] hpos_d;
always @(posedge clk_sys) if (ce_pix) hpos_d <= status[98:92];
wire signed [8:0] hpos_off = ((hpos_d <= 7'd48)
	? $signed({2'b0, hpos_d})
	: $signed({2'b0, hpos_d}) - 9'sd128) <<< 2;

// V-Shift (status[103:99], signed 5-bit -16..+15 righe).
reg signed [5:0] vshift_off;
always @(posedge clk_sys) if (ce_pix) vshift_off <= $signed(status[103:99]);

// Read rate a QUARTI di ciclo, accumulatore. Periodo = 16+hsize quarti
// (nativo 16 = 4 clk), reset sul RISE di hs_ref (pattern Legionnaire).
wire hs_ref;
reg  hs_ref_d;
always @(posedge clk_sys) hs_ref_d <= hs_ref;
wire hs_ref_rise = hs_ref & ~hs_ref_d;
// Base periodo: 16 quarti (4 clk, 24MHz) — 24 quarti (6 clk, 16MHz) in Triple Wide
wire [7:0] rd_base = wide3_mode ? 8'd24 : 8'd16;
wire [7:0] rd_period_raw = rd_base + {{3{hsize_s[4]}}, hsize_s};
wire [7:0] rd_period = (rd_period_raw < 8'd8) ? 8'd8 : rd_period_raw;
reg  [7:0] rd_acc;
wire rd_tick = (rd_acc + 8'd4) >= {1'b0, rd_period};
always @(posedge clk_sys) begin
	if      (hs_ref_rise) rd_acc <= 8'd0;
	else if (rd_tick)     rd_acc <= rd_acc + 8'd4 - {1'b0, rd_period};
	else                  rd_acc <= rd_acc + 8'd4;
end
wire rd_ce = crt_on ? rd_tick : fc_ce;

wire [7:0] str_r, str_g, str_b;
wire       str_hs, str_vs, str_hb, str_vb;
// Darius = gioco LARGO e centrato (attivo 864-1296 su 1527): HPOS_CONTENTSHIFT.
crt_adjust #(
	.VTOTAL   (V_TOTAL_DAR),
	.HTOTAL   (H_TOTAL_DAR),
	.HPOS_MODE(1),         // 1 = HPOS_CONTENTSHIFT (gioco largo/centrato)
	.AW       (12)         // 2048 sample/banco (linea 1527 slot)
) u_crt_adjust (
	.clk      (clk_sys),
	.pxl_cen  (fc_ce),
	.pxl2_cen (rd_ce),
	.active   (crt_on),
	.hsize    (hsize_s),
	.hoffset  (hpos_off),
	.voffset  (vshift_off),
	.r_in     (av_r), .g_in (av_g), .b_in (av_b),
	.hs_in    (fc_hs),           // HSync nativo del modo corrente -> no desync
	.vs_in    (vs_q),
	.hb_in    (fc_hb | vbl_q),
	.vb_in    (vbl_q),
	.r_out    (str_r), .g_out (str_g), .b_out (str_b),
	.hs_out   (str_hs), .vs_out (str_vs),
	.hb_out   (str_hb), .vb_out (str_vb),
	.hs_ref_out (hs_ref)
);

// Finestra DE per l'OSD: apre all'attivo nativo (VBlank ritardato 1 riga),
// chiude a larghezza stretchata piena (pattern Legionnaire/Blood Bros).
reg vblank_1l;
reg fc_hb_d;
always @(posedge clk_sys) if (ce_pix) fc_hb_d <= fc_hb;
wire line_tick = ce_pix && (fc_hb & ~fc_hb_d);
always @(posedge clk_sys) if (line_tick) vblank_1l <= vbl_q;
wire native_active = ~(fc_hb | vblank_1l);
reg  native_active_d;
always @(posedge clk_sys) if (ce_pix) native_active_d <= native_active;
wire native_rise = native_active & ~native_active_d;
wire str_active = ~str_hb;
reg  str_active_d;
always @(posedge clk_sys) if (rd_ce) str_active_d <= str_active;
wire str_fall = str_active_d & ~str_active;
reg de_osd;
always @(posedge clk_sys) begin
	if      (native_rise) de_osd <= 1'b1;
	else if (str_fall)    de_osd <= 1'b0;
end

// Output: ON -> dal modulo; OFF -> nativo.
assign VGA_R = crt_on ? str_r : av_r;
assign VGA_G = crt_on ? str_g : av_g;
assign VGA_B = crt_on ? str_b : av_b;

// Aspect ratio: Original = 4:1 (3x 4:3 monitors); in follow AR esatto W:224.
// Full Screen = 0:0
wire [11:0] follow_arx = (follow_view == 3'd2) ? 12'd320 :
                         (follow_view == 3'd3) ? 12'd432 : 12'd288;  // 1 e 4 (jump) = 288
wire [11:0] arx = (!ar) ? (follow_en ? follow_arx : 12'd4) : (ar - 1'd1);
wire [11:0] ary = (!ar) ? (follow_en ? 12'd224     : 12'd1) : 12'd0;

// Integer scaling (Scale menu: Normal / V-Integer / Narrower HV-Integer)
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(crt_on ? rd_ce : fc_ce),
	.VGA_VS(vs_q),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(crt_on ? de_osd : ~(fc_hb | vbl_q)),
	.ARX(arx),
	.ARY(ary),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE((status[6:5] == 2'd0) ? 3'd2 :   // Narrower HV-Integer (default, = HV-Integer-)
	        (status[6:5] == 2'd1) ? 3'd1 :   // V-Integer
	                                3'd4)    // HV-Integer
);

// LED: blink during download
assign LED_USER = ioctl_download;

// ============================================================
// JTAG Debug Probes (readable via quartus_stp / System Console)
// ============================================================
// JTAG boot trace removed to save M10K for 64KB work RAM

endmodule
