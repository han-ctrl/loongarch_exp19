`timescale 1ns / 1ps

module sram_to_axi_bridge(
    input wire          aclk,
    input wire          aresetn,
    //inst sram interface
    input wire          inst_sram_req,
    input wire          inst_sram_wr,
    input wire[2:0]     inst_sram_size,
    input wire[3:0]     inst_sram_wstrb,
    input wire[31:0]    inst_sram_addr,
    input wire[31:0]    inst_sram_wdata,
    output wire         inst_sram_addr_ok,
    output wire         inst_sram_data_ok,
    output wire[31:0]   inst_sram_rdata,
    //data sram interface
    input wire          data_sram_req,
    input wire          data_sram_wr,
    input wire[2:0]     data_sram_size,
    input wire[3:0]     data_sram_wstrb,
    input wire[31:0]    data_sram_addr,
    input wire[31:0]    data_sram_wdata,
    output wire         data_sram_addr_ok,
    output wire         data_sram_data_ok,
    output wire[31:0]   data_sram_rdata,
    //read request interface
    output wire[3:0]    arid,
    output wire[31:0]   araddr,
    output wire[7:0]    arlen,
    output wire[2:0]    arsize,
    output wire[1:0]    arburst,
    output wire[1:0]    arlock,
    output wire[3:0]    arcache,
    output wire[2:0]    arprot,
    output wire         arvalid,
    input wire          arready,
    //read response interface
    input wire[3:0]     rid,
    input wire[31:0]    rdata,
    input wire[1:0]     rresp,
    input wire          rlast,
    input wire          rvalid,
    output wire         rready,
    //write request interface
    output wire[3:0]    awid,
    output wire[31:0]   awaddr,
    output wire[7:0]    awlen,
    output wire[2:0]    awsize,
    output wire[1:0]    awburst,
    output wire[1:0]    awlock,
    output wire[3:0]    awcache,
    output wire[2:0]    awprot,
    output wire         awvalid,
    input wire          awready,
    //write data interface
    output wire[3:0]    wid,
    output wire[31:0]   wdata,
    output wire[3:0]    wstrb,
    output wire         wlast,
    output wire         wvalid,
    input wire          wready,
    //write response interface
    input wire[3:0]     bid,
    input wire[1:0]     bresp,
    input wire          bvalid,
    output wire         bready
    );
//read request
reg [3:0]   arid_r;
reg [31:0]  araddr_r;
reg [2:0]   arsize_r;
reg         arvalid_r;
//write request
reg [3:0]   awid_r;
reg [31:0]  awaddr_r;
reg [2:0]   awsize_r;
reg         awvalid_r;
//write data
reg [31:0]  wdata_r;
reg [3:0]   wstrb_r;
reg         wvalid_r;

wire        read_req, read_stall;
wire [3:0]  read_id;
wire [31:0] read_addr;
wire [2:0]  read_size;
reg [2:0]   write_cnt;
assign read_req   = (inst_sram_req && !inst_sram_wr) || (data_sram_req && !data_sram_wr);
assign read_stall = write_cnt!=3'b0;
assign read_id    = 4'b0000 & {4{inst_sram_req && !inst_sram_wr}} | 4'b0001 & {4{data_sram_req && !data_sram_wr}};//优先读数据
assign read_addr  = {inst_sram_addr & {32{inst_sram_req && !inst_sram_wr && read_id==4'b0}}} | {data_sram_addr & {32{data_sram_req && !data_sram_wr}}};//优先传送读数据的地址
assign read_size  = {inst_sram_size & {3{inst_sram_req && !inst_sram_wr && read_id==4'b0}}} | {data_sram_size & {3{data_sram_req && !data_sram_wr}}};

always @(posedge aclk ) begin
    if(aresetn)begin
        arid_r   <= 4'b10;
        araddr_r <= 32'b0;
        arsize_r <= 3'b0;
        arvalid_r<= 1'b0;
    end
    else if(!arvalid && read_req && !read_stall)begin
        arid_r   <= read_id;
        araddr_r <= read_addr;
        arsize_r <= read_size;
        arvalid_r<= 1'b1;        
    end
    else if(arvalid && arready)begin//握手成功，数据传送了
        arid_r   <= 4'b10;
        araddr_r <= 32'b0;
        arsize_r <= 3'b0;
        arvalid_r<= 1'b0;        
    end
end

wire    write_req;
assign write_req = data_sram_req && data_sram_wr;
always @(posedge aclk ) begin//写地址通道
    if(aresetn)begin
        awaddr_r <= 32'b0;
        awsize_r <= 3'b0;
        awvalid_r<= 1'b0;
    end
    else if(!awvalid && !wvalid && write_req)begin
        awaddr_r <= data_sram_addr;
        awsize_r <= data_sram_size;
        awvalid_r<= 1'b1;
    end
    else if(awvalid && awready)begin
        awaddr_r <= 32'b0;
        awsize_r <= 3'b0;
        awvalid_r<= 1'b0;
    end
end
always @(posedge aclk ) begin//写数据通道
    if(aresetn)begin
        wdata_r <= 32'b0;
        wstrb_r <= 4'b0;
        wvalid_r<= 1'b0;
    end
    else if(!awvalid && !wvalid && write_req)begin
        wdata_r <= data_sram_wdata;
        wstrb_r <= data_sram_wstrb;
        wvalid_r<= 1'b1;
    end
    else if(wvalid && wready)begin
        wdata_r <= 32'b0;
        wstrb_r <= 4'b0;
        wvalid_r<= 1'b0;
    end    
end
always @(posedge aclk ) begin
    if(aresetn)begin
        write_cnt <= 3'b0;
    end
    else if((awvalid && awready) && !(bvalid && bready))begin
        write_cnt <= write_cnt + 1;
    end
    else if(!(awvalid && awready) && (bvalid && bready))begin
        write_cnt <= write_cnt - 1;
    end
end

assign inst_sram_addr_ok = (arid==4'b0) && arvalid && arready;
assign inst_sram_data_ok = (rid==4'b0) && rvalid && rready;
assign inst_sram_rdata   = {32{rid==4'b0}} & rdata;

assign data_sram_addr_ok = ((arid==4'b1) && arvalid && arready) || ((awid==4'b1) && awvalid && awready);
assign data_sram_data_ok = (rid==4'b1) && rvalid && rready || bvalid && bready;
assign data_sram_rdata   = {32{rid==4'b1}} & rdata;

assign arid    = arid_r;
assign araddr  = araddr_r;
assign arsize  = arsize_r;
assign arvalid = arvalid_r;
assign rready  = 1'b1;

assign awaddr  = awaddr_r;
assign awsize  = awsize_r;
assign awvalid = awvalid_r;
assign wdata   = wdata_r;
assign wstrb   = wstrb_r;
assign wlast   = 1'b1;
assign wvalid  = wvalid_r;
assign bready  = 1'b1;

assign arlen    = 8'h0; 
assign arburst  = 2'b1;
assign arlock   = 2'b0;
assign arcache  = 4'b0;
assign arprot   = 3'b0;

assign awid     = 4'b1;
assign awlen    = 8'b0;
assign awburst  = 2'b1;
assign awlock   = 2'b0;
assign awcache  = 4'b0;
assign awprot   = 3'b0;

assign wid      = 4'b1;
assign wlast    = 1'b1;

endmodule