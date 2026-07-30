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

    Cache logic based on Darius2NinjaWarriors, adapted to Darius Rework:
      - SDRAM backend via tile_rom_arbiter client r3 (pulse/valid protocol),
        NOT a DDR3 toggle (Darius Rework keeps the sprite ROM in SDRAM).
      - cache DOUBLED vs NinjaWarriors: 2048 sets x 2 way = 4096 entries
        (Darius sprite address space = 20 bit, addr[19:0]).
*/

// sprite_rom_cache — 2-way set associative cache of the sprite ROM row-words,
// inserted BETWEEN the renderer and the tile_rom_arbiter (client r3). The
// renderer does not know it exists: it sees the same spriterom_* protocol
// (req_pulse -> resp_valid).
//
// sprite address (from the renderer): {cur_code[12:0], row[3], half, row[2:0], 2'b00}
//   -> 20 real bits (addr[19:0]); addr[1:0]=00 always.
//   Index = addr[12:2]  (11 bit -> 2048 sets)
//   Tag   = addr[19:13] (7 bit)  + valid
//
// Backend: pulse/valid protocol identical to the arbiter's r3 client:
//   be_req rising -> arbiter serves -> be_valid pulse 1 ck with be_data.

module sprite_rom_cache (
	input  wire        clk,
	input  wire        reset,

	// Lato renderer (rising-edge protocol, come spriterom_*)
	input  wire [23:0] req_addr,
	input  wire        req_pulse,
	output reg  [31:0] resp_data,
	output reg         resp_valid,

	// Lato backend (verso arbiter client r3): pulse req -> valid pulse
	output reg  [23:0] be_addr,
	output reg         be_req,
	input  wire [31:0] be_data,
	input  wire        be_valid
);

// =====================================================================
// 2-way set associative, 2048 set x 2 way = 4096 entry (RADDOPPIATA)
// =====================================================================
(* ramstyle = "M10K" *) reg [31:0] cache_data_w0 [0:2047];
(* ramstyle = "M10K" *) reg [31:0] cache_data_w1 [0:2047];
(* ramstyle = "M10K" *) reg [7:0]  cache_tag_w0  [0:2047];   // [7]=valid, [6:0]=tag
(* ramstyle = "M10K" *) reg [7:0]  cache_tag_w1  [0:2047];
reg lru [0:2047];   // 1-bit LRU per set

wire [10:0] req_idx    = req_addr[12:2];
wire [6:0]  req_tag_in = req_addr[19:13];

reg [10:0] clr_cnt;
reg        cache_ready;

// lookup registered (1 ck)
reg [31:0] cache_data_q_w0, cache_data_q_w1;
reg [7:0]  cache_tag_q_w0,  cache_tag_q_w1;
reg [6:0]  req_tag_q;
reg [10:0] req_idx_q;
reg        lru_q;

always @(posedge clk) begin
	cache_data_q_w0 <= cache_data_w0[req_idx];
	cache_data_q_w1 <= cache_data_w1[req_idx];
	cache_tag_q_w0  <= cache_tag_w0[req_idx];
	cache_tag_q_w1  <= cache_tag_w1[req_idx];
	lru_q           <= lru[req_idx];
	req_tag_q       <= req_tag_in;
	req_idx_q       <= req_idx;
end

wire hit_w0 = cache_ready && cache_tag_q_w0[7] && (cache_tag_q_w0[6:0] == req_tag_q);
wire hit_w1 = cache_ready && cache_tag_q_w1[7] && (cache_tag_q_w1[6:0] == req_tag_q);
wire cache_hit = hit_w0 | hit_w1;

localparam ST_CLEAR    = 2'd0;
localparam ST_IDLE     = 2'd1;
localparam ST_LOOKUP   = 2'd2;
localparam ST_WAIT_RAM = 2'd3;

reg [1:0]  state;
reg [23:0] pending_addr;

reg        cache_we_w0, cache_we_w1;
reg [10:0] cache_we_idx;
reg [7:0]  cache_we_tag;
reg [31:0] cache_we_data;
reg        lru_we;
reg        lru_we_val;

always @(posedge clk) begin
	if (cache_we_w0) begin
		cache_data_w0[cache_we_idx] <= cache_we_data;
		cache_tag_w0 [cache_we_idx] <= cache_we_tag;
	end
	if (cache_we_w1) begin
		cache_data_w1[cache_we_idx] <= cache_we_data;
		cache_tag_w1 [cache_we_idx] <= cache_we_tag;
	end
	if (lru_we) lru[cache_we_idx] <= lru_we_val;
end

always @(posedge clk) begin
	if (reset) begin
		state         <= ST_CLEAR;
		clr_cnt       <= 11'd0;
		cache_ready   <= 1'b0;
		be_req        <= 1'b0;
		be_addr       <= 24'd0;
		resp_valid    <= 1'b0;
		resp_data     <= 32'd0;
		pending_addr  <= 24'd0;
		cache_we_w0   <= 1'b0;
		cache_we_w1   <= 1'b0;
		cache_we_idx  <= 11'd0;
		cache_we_tag  <= 8'd0;
		cache_we_data <= 32'd0;
		lru_we        <= 1'b0;
		lru_we_val    <= 1'b0;
	end else begin
		resp_valid  <= 1'b0;
		cache_we_w0 <= 1'b0;
		cache_we_w1 <= 1'b0;
		lru_we      <= 1'b0;
		be_req      <= 1'b0;   // pulse: alto 1 ck

		case (state)
			ST_CLEAR: begin
				cache_we_w0  <= 1'b1;
				cache_we_w1  <= 1'b1;
				cache_we_idx <= clr_cnt;
				cache_we_tag <= 8'd0;     // valid=0
				cache_we_data<= 32'd0;
				if (clr_cnt == 11'd2047) begin
					cache_ready <= 1'b1;
					state       <= ST_IDLE;
				end else
					clr_cnt <= clr_cnt + 11'd1;
			end

			ST_IDLE: begin
				if (req_pulse) begin
					pending_addr <= req_addr;
					state        <= ST_LOOKUP;
				end
			end

			ST_LOOKUP: begin
				if (cache_hit) begin
					resp_data    <= hit_w0 ? cache_data_q_w0 : cache_data_q_w1;
					resp_valid   <= 1'b1;
					cache_we_idx <= req_idx_q;
					lru_we       <= 1'b1;
					lru_we_val   <= hit_w0 ? 1'b1 : 1'b0;
					state        <= ST_IDLE;
				end else begin
					// miss: richiedi al backend (arbiter r3), pulse 1 ck
					be_addr <= pending_addr;
					be_req  <= 1'b1;
					state   <= ST_WAIT_RAM;
				end
			end

			ST_WAIT_RAM: begin
				if (be_valid) begin
					resp_data    <= be_data;
					resp_valid   <= 1'b1;
					cache_we_idx <= pending_addr[12:2];
					cache_we_tag <= {1'b1, pending_addr[19:13]};
					cache_we_data<= be_data;
					if (lru_q) cache_we_w1 <= 1'b1;
					else       cache_we_w0 <= 1'b1;
					lru_we     <= 1'b1;
					lru_we_val <= ~lru_q;
					state      <= ST_IDLE;
				end
			end

			default: state <= ST_IDLE;
		endcase
	end
end

endmodule
