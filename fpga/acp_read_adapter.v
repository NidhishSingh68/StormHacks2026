// acp_read_adapter.v - Avalon-MM burst reads -> cache-coherent AXI3 reads
//
// The frame DMA (fb_dma, an mSGDMA) reads the CPU's back buffer from DDR.
// The CPU draws into normal cached memory, so the newest pixels may still be
// in its L1/L2 caches. Reads therefore go through the Cortex-A9's ACP
// (Accelerator Coherency Port), which the FPGA-to-HPS bridge exposes at
// 0x80000000-0xBFFFFFFF = DDR 0x00000000-0x3FFFFFFF, and are marked
// cacheable + shared so the ACP snoops the caches:
//   ARCACHE = 4'b1111 (write-back, read/write allocate)
//   ARUSER  = 5'b11111 (shared, inner write-back)
// Platform Designer always sends ARCACHE = 0 for Avalon masters, which the
// ACP treats as non-coherent, hence this adapter outside the system.
//
// The DMA is given plain DDR physical addresses; the window is added here.
// Read-only: the write channels are tied off.

module acp_read_adapter (
    input  wire        clk,

    // Avalon-MM read master side (from fb_dma.mm_read)
    input  wire [31:0] avm_address,     // DDR physical byte address
    input  wire        avm_read,
    input  wire [2:0]  avm_burstcount,  // 1..4 beats of 8 bytes
    output wire [63:0] avm_readdata,
    output wire        avm_waitrequest,
    output wire        avm_readdatavalid,

    // AXI3 master side (to the HPS f2h_axi_slave)
    output wire [7:0]  arid,
    output wire [31:0] araddr,
    output wire [3:0]  arlen,
    output wire [2:0]  arsize,
    output wire [1:0]  arburst,
    output wire [1:0]  arlock,
    output wire [3:0]  arcache,
    output wire [2:0]  arprot,
    output wire [4:0]  aruser,
    output wire        arvalid,
    input  wire        arready,
    input  wire [63:0] rdata,
    input  wire        rvalid,
    output wire        rready,

    output wire [7:0]  awid,
    output wire [31:0] awaddr,
    output wire [3:0]  awlen,
    output wire [2:0]  awsize,
    output wire [1:0]  awburst,
    output wire [1:0]  awlock,
    output wire [3:0]  awcache,
    output wire [2:0]  awprot,
    output wire [4:0]  awuser,
    output wire        awvalid,
    output wire [7:0]  wid,
    output wire [63:0] wdata,
    output wire [7:0]  wstrb,
    output wire        wlast,
    output wire        wvalid,
    output wire        bready
);

    // one address phase per Avalon burst; the same ID keeps responses in order
    assign arid    = 8'd0;
    assign araddr  = {2'b10, avm_address[29:0]};        // ACP window
    assign arlen   = {1'b0, avm_burstcount} - 4'd1;
    assign arsize  = 3'b011;                            // 8 bytes per beat
    assign arburst = 2'b01;                             // INCR
    assign arlock  = 2'b00;
    assign arcache = 4'b1111;
    assign arprot  = 3'b000;
    assign aruser  = 5'b11111;
    assign arvalid = avm_read;
    assign avm_waitrequest = ~arready;

    // the DMA only asks for data it has room for, so never stall the bridge
    assign rready            = 1'b1;
    assign avm_readdata      = rdata;
    assign avm_readdatavalid = rvalid;

    // no writes
    assign awid    = 8'd0;
    assign awaddr  = 32'd0;
    assign awlen   = 4'd0;
    assign awsize  = 3'b011;
    assign awburst = 2'b01;
    assign awlock  = 2'b00;
    assign awcache = 4'b0000;
    assign awprot  = 3'b000;
    assign awuser  = 5'b00000;
    assign awvalid = 1'b0;
    assign wid     = 8'd0;
    assign wdata   = 64'd0;
    assign wstrb   = 8'd0;
    assign wlast   = 1'b0;
    assign wvalid  = 1'b0;
    assign bready  = 1'b1;

    wire unused = clk;

endmodule
