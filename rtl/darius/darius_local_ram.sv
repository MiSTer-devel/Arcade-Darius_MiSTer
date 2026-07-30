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

// darius_local_ram — single-port BRAM with byte enable.
// HI/LO split for clean M10K inference with byte enable.
// Used for main RAM, local sprite RAM and other single-client RAMs.

module darius_local_ram
#(
	parameter integer ADDR_WIDTH = 11
)
(
	input  wire                  clk,
	input  wire                  rd,
	input  wire                  wr,
	input  wire [1:0]            be,    // byte enable: be[1]=write HI, be[0]=write LO
	input  wire [ADDR_WIDTH-1:0] addr,
	input  wire [15:0]           wdata,
	output wire [15:0]           rdata
);

// Split into two 8-bit RAMs for proper M10K byte-enable inference
(* ramstyle = "M10K" *) reg [7:0] ram_hi [0:(1 << ADDR_WIDTH)-1];
(* ramstyle = "M10K" *) reg [7:0] ram_lo [0:(1 << ADDR_WIDTH)-1];
reg [7:0] rdata_hi, rdata_lo;

// synthesis translate_off
initial begin : init_ram
	integer i;
	for (i = 0; i < (1 << ADDR_WIDTH); i = i + 1) begin
		ram_hi[i] = 8'h00;
		ram_lo[i] = 8'h00;
	end
end
// synthesis translate_on

// Lettura non gated (come le RAM di BoogieWings): rdata segue addr con 1 clk
// di latenza — necessario per ss_ram16_adaptor (legge via addr_out senza rd);
// per la CPU equivalente (campiona rdata solo nei propri cicli di read).
always @(posedge clk) begin
	if (wr && be[1]) ram_hi[addr] <= wdata[15:8];
	if (wr && be[0]) ram_lo[addr] <= wdata[7:0];
	rdata_hi <= ram_hi[addr];
	rdata_lo <= ram_lo[addr];
end

assign rdata = {rdata_hi, rdata_lo};

endmodule
