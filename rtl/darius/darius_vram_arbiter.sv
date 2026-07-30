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

// darius_vram_arbiter — shared VRAM read arbiter.
// Edge-based round-robin for 3 panel renderers on the same VRAM.
// Panels pulse req for 1 cycle when the address is set, the arbiter latches
// it as pending, serves the next cycle, and delivers data 2 cycles later.

module darius_vram_arbiter (
	input  wire        clk,
	input  wire        reset,

	input  wire [14:0] p0_addr,
	input  wire        p0_req,
	output reg  [15:0] p0_data,
	output reg         p0_valid,

	input  wire [14:0] p1_addr,
	input  wire        p1_req,
	output reg  [15:0] p1_data,
	output reg         p1_valid,

	input  wire [14:0] p2_addr,
	input  wire        p2_req,
	output reg  [15:0] p2_data,
	output reg         p2_valid,

	output reg  [14:0] vram_addr,
	input  wire [15:0] vram_data
);

reg [2:0] pending;

// Pipeline: grant (set addr) → BRAM read → deliver
reg [1:0] pipe1_panel;  // granted panel (addr sent to BRAM)
reg       pipe1_valid;
reg [1:0] pipe2_panel;  // BRAM data arriving
reg       pipe2_valid;

reg [1:0] next_prio;

always @(posedge clk) begin
	if (reset) begin
		pending <= 0;
		pipe1_panel <= 0; pipe1_valid <= 0;
		pipe2_panel <= 0; pipe2_valid <= 0;
		p0_valid <= 0; p1_valid <= 0; p2_valid <= 0;
		next_prio <= 0;
	end else begin
		// Latch incoming requests (pulse-based, 1 cycle)
		if (p0_req) pending[0] <= 1'b1;
		if (p1_req) pending[1] <= 1'b1;
		if (p2_req) pending[2] <= 1'b1;

		// Pipeline advance
		pipe2_panel <= pipe1_panel;
		pipe2_valid <= pipe1_valid;

		// Clear valids
		p0_valid <= 1'b0;
		p1_valid <= 1'b0;
		p2_valid <= 1'b0;

		// Deliver data from pipe2 (BRAM data now valid)
		if (pipe2_valid) begin
			case (pipe2_panel)
				2'd0: begin p0_data <= vram_data; p0_valid <= 1'b1; end
				2'd1: begin p1_data <= vram_data; p1_valid <= 1'b1; end
				2'd2: begin p2_data <= vram_data; p2_valid <= 1'b1; end
				default: ;
			endcase
		end

		// Grant next pending (round-robin)
		pipe1_valid <= 1'b0;
		begin : grant_block
			integer i;
			reg [1:0] check;
			reg granted;
			granted = 0;
			for (i = 0; i < 3 && !granted; i = i + 1) begin
				check = (next_prio + i[1:0] >= 2'd3) ? next_prio + i[1:0] - 2'd3 : next_prio + i[1:0];
				if (pending[check]) begin
					case (check)
						2'd0: vram_addr <= p0_addr;
						2'd1: vram_addr <= p1_addr;
						2'd2: vram_addr <= p2_addr;
						default: ;
					endcase
					pipe1_panel <= check;
					pipe1_valid <= 1'b1;
					pending[check] <= 1'b0;  // clear (wins over set if same cycle)
					next_prio <= (check == 2'd2) ? 2'd0 : check + 2'd1;
					granted = 1;
				end
			end
		end
	end
end

endmodule
