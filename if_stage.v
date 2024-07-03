


`include "mycpu.vh"

module if_stage(
    input                          clk            ,
    input                          reset          ,
    //allwoin
    input                          ds_allowin     ,
    //brbus
    input  [`BR_BUS_WD       -1:0] br_bus         ,
    //from csr
    input                          flush          ,
    input  [31:0]                  exception_pc   ,
    input  [1:0 ]                  crmd_plv,
    input                          crmd_da,
    input                          crmd_pg,
    input                          dmw0_plv0,
    input                          dmw0_plv3,
    input  [2:0]                   dmw0_pseg,
    input  [2:0]                   dmw0_vseg,
    input                          dmw1_plv0,
    input                          dmw1_plv3,
    input  [2:0]                   dmw1_pseg,
    input  [2:0]                   dmw1_vseg,
    input  [9:0]                   asid_asid,
    //to ds
    output                         fs_to_ds_valid ,
    output [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus   ,
    // inst sram interface
    output                         inst_sram_req,//请求
    output                         inst_sram_wr,//1写请求，0读请求
    output [ 2:0]                  inst_sram_size,//传输的字节数
    output [ 3:0]                  inst_sram_wstrb,//写使能
    output [31:0]                  inst_sram_addr,
    output [31:0]                  inst_sram_wdata,
    inout                          inst_sram_addr_ok,//地址接收  或地址和数据接收完成
    input                          inst_sram_data_ok,//数据返回 或数据写入完成
    input  [31:0]                  inst_sram_rdata,
    //from tlb
    input  wire                    s0_found,
    input  wire [ 3:0]             s0_index,
    input  wire [19:0]             s0_ppn,
    input  wire [ 5:0]             s0_ps,
    input  wire [ 1:0]             s0_plv,
    input  wire [ 1:0]             s0_mat,
    input  wire                    s0_d,
    input  wire                    s0_v,
    //to tlb
    output wire [18:0]             s0_vppn,
    output wire                    s0_va_bit12,
    output wire [ 9:0]             s0_asid
);

reg         fs_valid;
wire        fs_ready_go;
wire        fs_allowin;
wire        to_fs_valid;

wire [31:0] seq_pc;
wire [31:0] nextpc;

wire         br_taken;
wire [ 31:0] br_target;
wire         br_stall;

wire [31:0] fs_inst;
reg  [31:0] fs_inst_register;
reg         fs_inst_r_valid;
reg  [31:0] fs_pc;
wire        pre_if_ready_go;
wire [`EXCEPTION_CODE-1:0] exception_code;
wire        adef;
wire        cancel; 
reg         exception_cancel;
//reg         inst_sram_req_reg;
reg         inst_sram_addr_ok_reg;
reg  [2:0]  state;
localparam WAIT_ADDR_OK=3'b001;
localparam WAIT_DATA_OK=3'b010;
localparam WAIT_STUCK_OK=3'b100; 

wire ppi, pif, tlbr;
wire [31:0] direct_pa, dmw0_pa, dmw1_pa, tlb_pa, inst_pa;
wire direct_pa_valid, dmw0_pa_valid, dmw1_pa_valid, tlb_pa_valid;

//数据传输
assign fs_to_ds_bus = {exception_code,//14
                       fs_inst&{32{~adef}} ,
                       fs_pc   };
assign {br_stall, br_taken, br_target} = br_bus;

// pre-IF stage
assign pre_if_ready_go = inst_sram_addr_ok | inst_sram_addr_ok_reg;//请求发出且被接收
assign to_fs_valid     = pre_if_ready_go;
assign seq_pc          = fs_pc + 3'h4;
assign nextpc          = {32{br_taken}} & br_target |
                         {32{flush}} & exception_pc | 
                         {32{!br_taken && !flush}} & seq_pc; 

always @(posedge clk) begin
    if(reset) begin
        inst_sram_addr_ok_reg <= 1'b0;
    end
    else if(ds_allowin) begin
        inst_sram_addr_ok_reg <= 1'b0;
    end
    else if(inst_sram_addr_ok) begin
        inst_sram_addr_ok_reg <= 1'b1;
    end
end
always @(posedge clk ) begin
    if(reset) begin
        fs_inst_r_valid <= 1'b0;
        fs_inst_register<= 32'b0;
    end
    else if(!ds_allowin && fs_valid && inst_sram_data_ok)begin
        fs_inst_r_valid <= 1'b1;
        fs_inst_register<= inst_sram_rdata;        
    end
    else if(ds_allowin)begin
        fs_inst_r_valid <= 1'b0;
        fs_inst_register<= 32'b0;        
    end
end

// IF stage
assign fs_ready_go    = ~br_taken && ((state==WAIT_DATA_OK && inst_sram_data_ok) || (state==WAIT_STUCK_OK && fs_inst_r_valid)) && !(exception_cancel || cancel);   // 准备发送
assign fs_allowin     = !fs_valid || (fs_ready_go && ds_allowin) || flush;     // 可接收数据（不阻塞
assign fs_to_ds_valid =  fs_valid && fs_ready_go && ~flush;   
assign fs_inst  = fs_inst_r_valid ? fs_inst_register :
                                    inst_sram_rdata; 
always @(posedge clk ) begin
    if(reset)state <= WAIT_ADDR_OK;
    else if(state==WAIT_ADDR_OK && inst_sram_addr_ok)state <= WAIT_DATA_OK;
    else if(state==WAIT_DATA_OK && inst_sram_data_ok)begin
        if(exception_cancel || cancel)state <= WAIT_ADDR_OK;
        else state <= WAIT_STUCK_OK;
    end
    else if(state==WAIT_STUCK_OK && ds_allowin)state <= WAIT_ADDR_OK;
end                                                                     
always @(posedge clk) begin
    if (reset) begin
        fs_valid <= 1'b0;
    end 
    else if(flush)begin
        fs_valid <= 1'b0;
    end
    else if (fs_allowin) begin
        fs_valid <= to_fs_valid;    // 数据有效
    end
end
always @(posedge clk) begin
    if (reset) begin
        fs_pc <= 32'h1c000000;     //trick: to make nextpc be 0x1c000000 during reset  1bfffffc
    end
    else if (fs_ready_go || br_taken || flush) begin
        fs_pc <= nextpc;
    end
end
always @(posedge clk ) begin
    if(reset) 
        exception_cancel<=1'b0;
    else if((flush || br_taken) && (state == WAIT_ADDR_OK  || (state == WAIT_DATA_OK && ~inst_sram_data_ok)))
        exception_cancel<=1'b1;
    else if(inst_sram_data_ok) 
        exception_cancel<=1'b0;
end
assign cancel = (flush || br_taken) && (state == WAIT_ADDR_OK || (state == WAIT_DATA_OK && ~inst_sram_data_ok));
//取指令
assign inst_sram_req    = state==WAIT_ADDR_OK && !(|exception_code);
assign inst_sram_size   = 3'b010;
assign inst_sram_wr     = 1'b0;
assign inst_sram_wstrb  = 4'b0;
assign inst_sram_addr   = inst_pa;
assign inst_sram_wdata  = 32'b0;

//异常检测
assign pif  = tlb_pa_valid && !s0_v && fs_valid;//取指操作页无效例外
assign ppi  = tlb_pa_valid && s0_v && crmd_plv==2'b11 && s0_plv==2'b00 && fs_valid;//页特权等级不合规例外
assign tlbr = !direct_pa_valid && !dmw0_pa_valid && !dmw1_pa_valid && !tlb_pa_valid && fs_valid;//TLB 重填例外
assign adef = (fs_pc[1:0] != 2'b00) && fs_valid;//取指地址错例外
assign exception_code = {tlbr, 5'b0, adef, ppi, 2'b0, pif, 3'b0};
//{TLBR(IF), TLBR(EX), INE, BRK, SYS, ALE, ADEF, PPI(IF), PPI(EX), PME, PIF, PIS, PIL, INT}
//虚实地址转换
assign inst_pa = {32{direct_pa_valid}} & direct_pa |
                 {32{dmw0_pa_valid}} & dmw0_pa     |
                 {32{dmw1_pa_valid}} & dmw1_pa     |
                 {32{tlb_pa_valid}} & tlb_pa       ;
assign {direct_pa, direct_pa_valid} = {fs_pc, !crmd_pg && crmd_da};
assign dmw0_pa       = {dmw0_pseg, fs_pc[28:0]};
assign dmw0_pa_valid = (crmd_plv==2'b0&&dmw0_plv0 || crmd_plv==2'b11&&dmw0_plv3) && dmw0_vseg==fs_pc[31:29] && crmd_pg && !crmd_da;
assign dmw1_pa       = {dmw1_pseg, fs_pc[28:0]};
assign dmw1_pa_valid = (crmd_plv==2'b0&&dmw1_plv0 || crmd_plv==2'b11&&dmw1_plv3) && dmw1_vseg==fs_pc[31:29] && crmd_pg && !crmd_da;
assign tlb_pa        = {32{s0_ps==6'd12}}&{s0_ppn, fs_pc[11:0]} | {32{s0_ps==6'd21}}&{s0_ppn[19:9], fs_pc[20:0]};
assign tlb_pa_valid  = s0_found && crmd_pg && !crmd_da && !dmw0_pa_valid && !dmw1_pa_valid;
assign {s0_vppn, s0_va_bit12 ,s0_asid} = {fs_pc[31:12], asid_asid};
endmodule


