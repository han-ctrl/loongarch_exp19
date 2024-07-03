`timescale 1ns / 1ps
`include "mycpu.vh"

module CSR(
input                    clk,
input                    reset, 
input  [`TO_CSR_BUS-1:0] to_csr_bus,
output [31:0]            csr_rdata,
output [31:0]            exception_pc,//异常开始地址，异常结束返回地址
output                   csr_interrupt,//异常开始，异常结束返回
output                   flush,
//to fs, es
output [1:0 ]            crmd_plv_out,
output                   crmd_da_out,
output                   crmd_pg_out,
output                   dmw0_plv0_out,
output                   dmw0_plv3_out,
output [2:0]             dmw0_pseg_out,
output [2:0]             dmw0_vseg_out,
output                   dmw1_plv0_out,
output                   dmw1_plv3_out,
output [2:0]             dmw1_pseg_out,
output [2:0]             dmw1_vseg_out,
output [9:0]             asid_asid_out,
//to exe 
output [31:13]           es_tlbehi_vppn,      
//to tlb
output wire [$clog2(`TLBNUM)-1:0]r_index,
input  wire                      r_e,
input  wire [              18:0] r_vppn,
input  wire [               5:0] r_ps,
input  wire [               9:0] r_asid,
input  wire                      r_g,
input  wire [              19:0] r_ppn0,
input  wire [               1:0] r_plv0,
input  wire [               1:0] r_mat0,
input  wire                      r_d0,
input  wire                      r_v0,
input  wire [              19:0] r_ppn1,
input  wire [               1:0] r_plv1,
input  wire [               1:0] r_mat1,
input  wire                      r_d1,
input  wire                      r_v1,
output wire                      we,     //w(rite) e(nable)
output wire [$clog2(`TLBNUM)-1:0]w_index,
output wire                      w_e,//存在位，为1表示可以参与查找
output wire [              18:0] w_vppn,
output wire [               5:0] w_ps,
output wire [               9:0] w_asid,
output wire                      w_g,
output wire [              19:0] w_ppn0,
output wire [               1:0] w_plv0,
output wire [               1:0] w_mat0,
output wire                      w_d0,
output wire                      w_v0,
output wire [              19:0] w_ppn1,
output wire [               1:0] w_plv1,
output wire [               1:0] w_mat1,
output wire                      w_d1,
output wire                      w_v1
    );  

wire csr_we;
wire [13:0] csr_num;
wire [31:0] csr_mask;
wire [31:0] csr_wdata;
wire        ertn_flush;
wire        wb_exception;
wire [31:0] wb_pc;
reg [`TO_CSR_BUS-1:0]  to_csr_bus_r;
wire [5:0]  ecode;
wire [8:0]  esubcode;
wire [`EXCEPTION_CODE-1:0] exception_code;
wire exception_begin, exception_end;
wire        inst_tlbwr;
wire        inst_tlbrd;
wire        inst_tlbfill;

// CRMD
wire [31:0] crmd;
reg  [1:0]  crmd_plv;
reg         crmd_ie;
reg         crmd_da;
reg         crmd_pg;
reg  [6:5]  crmd_datf;
reg  [8:7]  crmd_datm;

//PRMD
wire [31:0] prmd;
reg  [1:0]  prmd_pplv;
reg         prmd_pie;

//ESTAT
wire [31:0]  estat;
reg  [12:0]  estat_is; 
reg  [21:16] estat_ecode;
reg  [30:22] estat_esubcode;

//ERA
wire [31:0] era;
reg  [31:0] era_pc;

//EENTRY
wire [31:0] eentry;
reg  [31:6] eentry_va;

//SAVE
reg [31:0] save0_data;
reg [31:0] save1_data;
reg [31:0] save2_data;
reg [31:0] save3_data;

//ECFG
wire [31:0] ecfg;
reg [12:0] ecfg_lie;//lie[10]=0;

//BADV 
wire [31:0] badv;
reg [31:0]  badv_VAddr;
wire        addr_is_pc;
wire        addr_is_data_sram;
wire [31:0] data_vaddr;

//TID
wire [31:0] tid;
reg [31:0]  tid_tid;

//TCFG
wire [31:0] tcfg;
reg [0:0]   tcfg_En;
reg [1:1]   tcfg_Periodic;
reg [31:2]  tcfg_InitVal;

//TVAL
wire [31:0] tval;
reg [31:0]  tval_TimeVal;//当前定时器的计数值,为4的整数倍

//TICLR
wire [31:0] ticlr;
wire        ticlr_clr;

//TLBIDX
wire [31:0] tlbidx;
reg [3:0]   tlbidx_index;
reg [29:24] tlbidx_ps;
reg [31:31] tlbidx_ne;
//TLBEHI
wire [31:0] tlbehi;
reg [31:13] tlbehi_vppn;
//TLBELO0
wire [31:0] tlblo0;
reg [0:0]   tlblo0_v;
reg [1:1]   tlblo0_d;
reg [3:2]   tlblo0_plv;
reg [5:4]   tlblo0_mat;
reg [6:6]   tlblo0_g;
reg [27:8]  tlblo0_ppn;
//TLBELO1
wire [31:0] tlblo1;
reg [0:0]   tlblo1_v;
reg [1:1]   tlblo1_d;
reg [3:2]   tlblo1_plv;
reg [5:4]   tlblo1_mat;
reg [6:6]   tlblo1_g;
reg [27:8]  tlblo1_ppn;
//ASID
wire [31:0] asid;
reg [9:0]   asid_asid;
wire [23:16] asid_asidbits;
//TLBRENTRY
wire [31:0] tlbrentry;
reg [31:6]  tlbrentry_pa;//重填例外的物理地址
//DMW0-1
wire [31:0] dmw0,      dmw1;
reg [0:0]   dmw0_plv0, dmw1_plv0;//为 1 表示在特权等级 PLV0 下可以使用该窗口的配置进行直接映射地址翻译。
reg [3:3]   dmw0_plv3, dmw1_plv3;//为 1 表示在特权等级 PLV3 下可以使用该窗口的配置进行直接映射地址翻译。
reg [5:4]   dmw0_mat,  dmw1_mat;//虚地址落在该映射窗口下访存操作的存储访问类型。
reg [27:25] dmw0_pseg, dmw1_pseg;//直接映射窗口的物理地址的[31:29]位。
reg [31:29] dmw0_vseg, dmw1_vseg;//直接映射窗口的虚地址的[31:29]位。

//数据接收
assign exception_begin = wb_exception;
assign exception_end = ertn_flush;
assign csr_interrupt = |(estat_is[12:0] & ecfg_lie[12:0]) && crmd_ie;
assign flush = exception_begin | exception_end;//刷新信号
assign exception_pc  = ({32{wb_exception}} & (ecode==`ECODE_TLBR ? tlbrentry : eentry)) |  ({32{ertn_flush}} & era);//异常地址跳转的地址
assign {inst_tlbrd,
        inst_tlbwr,
        inst_tlbfill,
        data_vaddr,
        csr_we,
        csr_num,
        csr_mask,
        csr_wdata,
        ertn_flush,
        wb_exception,
        exception_code,
        wb_pc}       =      to_csr_bus;
assign csr_rdata =  {32{csr_num==`CRMD}}&crmd        |
                    {32{csr_num==`PRMD}}&prmd        |
                    {32{csr_num==`ESTAT}}&estat      |
                    {32{csr_num==`ERA}}&era          |
                    {32{csr_num==`EENTRY}}&eentry    |
                    {32{csr_num==`SAVE0}}&save0_data |
                    {32{csr_num==`SAVE1}}&save1_data |
                    {32{csr_num==`SAVE2}}&save2_data |
                    {32{csr_num==`SAVE3}}&save3_data |
                    {32{csr_num==`ECFG}}&ecfg        |
                    {32{csr_num==`BADV}}&badv        |
                    {32{csr_num==`TID}}&tid          |
                    {32{csr_num==`TCFG}}&tcfg        |
                    {32{csr_num==`TVAL}}&tval        |
                    {32{csr_num==`TICLR}}&ticlr      |
                    {32{csr_num==`TLBIDX}}&tlbidx    |
                    {32{csr_num==`TLBEHI}}&tlbehi    |
                    {32{csr_num==`TLBELO0}}&tlblo0   |
                    {32{csr_num==`TLBELO1}}&tlblo1   |
                    {32{csr_num==`ASID}}&asid        |
                    {32{csr_num==`TLBRENTRY}}&tlbrentry|
                    {32{csr_num==`DMW0}}&dmw0        |
                    {32{csr_num==`DMW1}}&dmw1        ;

assign ecode =  {6{exception_code[13] | exception_code[12]}} & (`ECODE_TLBR) |
                     {6{exception_code[11]}} & (`ECODE_INE)  |
                     {6{exception_code[10]}} & (`ECODE_BRK)  |
                     {6{exception_code[9]}} & (`ECODE_SYS)   |
                     {6{exception_code[8]}} & (`ECODE_ALE)   |
                     {6{exception_code[7]}} & (`ECODE_ADE)   |
                     {6{exception_code[6] | exception_code[5]}} & (`ECODE_PPI)   |
                     {6{exception_code[4]}} & (`ECODE_PME)   |
                     {6{exception_code[3]}} & (`ECODE_PIF)   |
                     {6{exception_code[2]}} & (`ECODE_PIS)   |
                     {6{exception_code[1]}} & (`ECODE_PIL)   |
                     {6{exception_code[0]}} & (`ECODE_INT);
assign esubcode = {9{exception_code[7]}}  & (`ESUBCODE_ADEF);

//tlb
assign r_index = tlbidx_index;
assign we      = inst_tlbwr | inst_tlbfill;
assign w_index = tlbidx_index;
assign w_e     = !tlbidx_ne;
assign w_vppn  = tlbehi_vppn;
assign w_ps    = tlbidx_ps;
assign w_asid  = asid_asid;
assign w_g     = tlblo0_g && tlblo1_g;
assign w_ppn0 = tlblo0_ppn;
assign w_plv0 = tlblo0_plv;
assign w_mat0 = tlblo0_mat;
assign w_d0   = tlblo0_d;
assign w_v0   = tlblo0_v;
assign w_ppn1 = tlblo1_ppn;
assign w_plv1 = tlblo1_plv;
assign w_mat1 = tlblo1_mat;
assign w_d1   = tlblo1_d;
assign w_v1   = tlblo1_v;
assign es_tlbehi_vppn = tlbehi_vppn;
//to fs
assign asid_asid_out = asid_asid;
assign {dmw1_plv0_out, dmw1_plv3_out, dmw1_pseg_out, dmw1_vseg_out} = {dmw1_plv0, dmw1_plv3, dmw1_pseg, dmw1_vseg};
assign {dmw0_plv0_out, dmw0_plv3_out, dmw0_pseg_out, dmw0_vseg_out} = {dmw0_plv0, dmw0_plv3, dmw0_pseg, dmw0_vseg};
assign {crmd_pg_out, crmd_da_out, crmd_plv_out} = {crmd_pg, crmd_da, crmd_plv};
// CRMD
assign crmd = {23'b0, crmd_datm, crmd_datf, crmd_pg, crmd_da, crmd_ie, crmd_plv};

always @(posedge clk ) begin//ie, plv
    if(reset) begin
        {crmd_ie, crmd_plv} <= 3'b0;
    end
    else if(exception_begin)begin
        {crmd_ie, crmd_plv} <= 3'b0;
    end
    else if(exception_end)begin
        {crmd_ie, crmd_plv} <= {prmd_pie, prmd_pplv};
    end
    else if(csr_we && csr_num==`CRMD)begin
        {crmd_ie, crmd_plv} <= (csr_mask[2:0] & csr_wdata[2:0]) | (~csr_mask[2:0] & {crmd_ie, crmd_plv});
    end
end
always @(posedge clk ) begin//pg, da
    if(reset)begin
        {crmd_pg, crmd_da} <= 2'b01;
    end
    if((exception_begin) && ecode==`ECODE_TLBR)begin
        {crmd_pg, crmd_da} <= 2'b01;//TLB重填 pg=0, da=1
    end
    else if(exception_end && estat_ecode==`ECODE_TLBR)begin
        {crmd_pg, crmd_da} <= 2'b10;//TLB重填结束 pg=1, da=0
    end
    else if(csr_we && csr_num==`CRMD)begin
        {crmd_pg, crmd_da} <= (csr_mask[4:3] & csr_wdata[4:3]) | (~csr_mask[4:3] & {crmd_pg, crmd_da});
    end
end
always @(posedge clk ) begin//datm, datf
    if(reset)begin
        {crmd_datm, crmd_datf} <= 4'b0;
    end
    else if(csr_we && csr_num==`CRMD)begin
        {crmd_datm, crmd_datf} <= (csr_mask[8:5] & csr_wdata[8:5]) | (~csr_mask[8:5] & {crmd_datm, crmd_datf});
    end
end

//PRMD
assign prmd = {29'b0, prmd_pie, prmd_pplv};

always @(posedge clk ) begin
    if(exception_begin)begin
        {prmd_pie, prmd_pplv} <= {crmd_ie, crmd_plv};
    end
    else if(csr_we && csr_num==`PRMD)begin
        {prmd_pie, prmd_pplv} <= (csr_mask[2:0] & csr_wdata[2:0]) | (~csr_mask[2:0] & {prmd_pie, prmd_pplv});
    end
end

//ESTAT
assign estat = {1'b0, estat_esubcode, estat_ecode, 3'b0, estat_is};

always @(posedge clk ) begin
    if(reset) begin
        estat_is[1:0] <= 2'b0; //软中断
    end
    else if(csr_we && csr_num==`ESTAT)begin
        estat_is[1:0] <= {csr_mask[1:0] & csr_wdata[1:0]} | (~csr_mask[1:0] & estat_is[1:0]);
    end
    estat_is[10] <= 1'b0;
    estat_is[9:2] <= 8'b0;//硬中断
    if(tval_TimeVal==32'b0) estat_is[11] <= 1'b1;//定时器中断
    else if(csr_we && csr_num==`TICLR && csr_mask[0] && csr_wdata[0])estat_is[11] <= 1'b0;
    estat_is[12] <= 1'b0;//核间中断
end
always @(posedge clk ) begin
   if(exception_begin)begin
        estat_ecode <= ecode;
        estat_esubcode <= esubcode;
   end
end

//ERA
assign era = era_pc;

always @(posedge clk ) begin
    if(reset) begin
        era_pc <= 32'b0;
    end
    else if(exception_begin)begin
        era_pc <= wb_pc;
    end
    else if(csr_we && csr_num==`ERA)begin
        era_pc <= (csr_mask & csr_wdata) | (~csr_mask & era_pc);
    end
end

//EENTRY
assign eentry = {eentry_va, 6'b0};

always @(posedge clk ) begin
    if(reset) begin 
        eentry_va <= 26'b0; 
    end
    else if(csr_we && csr_num==`EENTRY)begin
        eentry_va <= (csr_mask[31:6] & csr_wdata[31:6]) | (~csr_mask[31:6] & eentry_va);
    end
end

//SAVE
always @(posedge clk ) begin
    if(reset)begin
        save0_data <= 32'b0;
        save1_data <= 32'b0;
        save2_data <= 32'b0;
        save3_data <= 32'b0;
    end
    else begin
        if(csr_we && csr_num==`SAVE0)begin
            save0_data <= (csr_mask & csr_wdata) | (~csr_mask & save0_data);
        end
        if(csr_we && csr_num==`SAVE1)begin
            save1_data <= (csr_mask & csr_wdata) | (~csr_mask & save1_data);
        end
        if(csr_we && csr_num==`SAVE2)begin
            save2_data <= (csr_mask & csr_wdata) | (~csr_mask & save2_data);
        end
        if(csr_we && csr_num==`SAVE3)begin
            save3_data <= (csr_mask & csr_wdata) | (~csr_mask & save3_data);
        end    
    end
end

//ECFG
assign ecfg = {19'b0, ecfg_lie};
always @(posedge clk ) begin
    if(reset)ecfg_lie <= 13'b0;
    else if(csr_we && csr_num==`ECFG)begin
        ecfg_lie <= (csr_mask[12:0] & csr_wdata[12:0] & 13'h1bff) | (~csr_mask[12:0] & ecfg_lie & 13'h1bff);
    end
end

//BADV 
assign addr_is_pc = exception_code[13] | exception_code[7] | exception_code[6] | exception_code[3]; //TLBR(IF) ,ADEF ,PPI(IF) ,PIF
assign addr_is_data_sram = exception_code[12] | exception_code[8] | exception_code[5] | exception_code[4] | exception_code[2] | exception_code[1]; //TLBR(EX),  ALE,  PPI(EX), PME PIS, PIL
assign badv = badv_VAddr;
always @(posedge clk ) begin
    if(reset) badv_VAddr <= 32'b0;
    else if(addr_is_pc) badv_VAddr <= wb_pc;
    else if(addr_is_data_sram) badv_VAddr <= data_vaddr;
    else if(csr_we && csr_num==`BADV) badv_VAddr <= (csr_mask & csr_wdata) | (~csr_mask & badv_VAddr);
end

//TID
assign tid = tid_tid;
always @(posedge clk ) begin
    if(reset) tid_tid <= 32'b0;
    else if(csr_we && csr_num==`TID)tid_tid <= (csr_mask & csr_wdata) | (~csr_mask & tid_tid);
end

//TCFG
assign tcfg = {tcfg_InitVal, tcfg_Periodic, tcfg_En};
always @(posedge clk ) begin
    if(reset) tcfg_En <= 1'b0;
    else if(csr_we && csr_num==`TCFG)begin
        {tcfg_InitVal, tcfg_Periodic, tcfg_En} <= (csr_mask & csr_wdata) | (~csr_mask & {tcfg_InitVal, tcfg_Periodic, tcfg_En});
    end
end

//TVAL
assign tval = tval_TimeVal;
always @(posedge clk ) begin
    if(reset)//非4的整数倍，赋值后tcfg_En有效也不进行计时 
            tval_TimeVal <= 32'hffffffff;
    else if(csr_we && csr_num==`TCFG && csr_mask[0] && csr_wdata[0])//reset后的第一次赋初值 tcfg_En <= csr_mask[0] & csr_wdata[0]
            tval_TimeVal <= {((csr_mask[31:2] & csr_wdata[31:2])|(~csr_mask[31:2] & tcfg_InitVal)), 2'b0};
    else if(tcfg_En && tcfg_Periodic && tval_TimeVal==32'b0)//减到0后，赋初值 
            tval_TimeVal <= {tcfg_InitVal,2'b0};
    else if(tcfg_En && tval_TimeVal != 32'hffffffff) 
            tval_TimeVal <= tval_TimeVal - 1'b1;
end

//TICLR
assign ticlr = {31'b0, ticlr_clr};
assign ticlr_clr = 1'b0; 

//TLBIDX
assign tlbidx = {tlbidx_ne, 1'b0, tlbidx_ps, 20'b0, tlbidx_index};
always @(posedge clk ) begin
    if (reset) begin
        tlbidx_index <= 4'b0;
        tlbidx_ps <= 6'b0;
        tlbidx_ne <= 1'b0;
    end
    else if (csr_we && csr_num==`TLBIDX) begin
        tlbidx_index <= (csr_mask[3:0] & csr_wdata[3:0]) | (~csr_mask[3:0] & tlbidx_index);
        tlbidx_ps <= (csr_mask[29:24] & csr_wdata[29:24]) | (~csr_mask[29:24] & tlbidx_ps);
        tlbidx_ne <= (csr_mask[31:31] & csr_wdata[31:31]) | (~csr_mask[31:31] & tlbidx_ne);
    end
    else if(inst_tlbrd)begin
        tlbidx_ps <= {6{r_e}} & r_ps;
        tlbidx_ne <= !r_e;
    end
end

//TLBEHI
assign tlbehi = {tlbehi_vppn ,13'b0};
always @(posedge clk ) begin
    if(reset)tlbehi_vppn <= 19'b0;
    //当触发 TLB 重填例外、load 操作页无效例外、store 操作页无效例外、取指操作页无效例外、页写允许例外和页特权等级不合规例外时，触发例外的虚地址的[31:13]位被记录到这里。
    else if(exception_code[13] || exception_code[6] || exception_code[3])begin
        tlbehi_vppn <= wb_pc[31:13];
    end
    else if(exception_code[12] || exception_code[5] || exception_code[4] || exception_code[2] || exception_code[1])begin
        tlbehi_vppn <= data_vaddr[31:13];
    end
    else if(csr_we && csr_num==`TLBEHI)begin
        tlbehi_vppn <= (csr_mask[31:13] & csr_wdata[31:13]) | (~csr_mask[31:13] & tlbehi_vppn);
    end
    else if(inst_tlbrd)begin
        tlbehi_vppn <=  {19{r_e}} & r_vppn;
    end
end

//TLBELO0
assign tlblo0 = {4'b0, tlblo0_ppn, 1'b0, tlblo0_g, tlblo0_mat, tlblo0_plv, tlblo0_d, tlblo0_v};
always @(posedge clk ) begin
    if(reset){tlblo0_ppn, tlblo0_g, tlblo0_mat, tlblo0_plv, tlblo0_d, tlblo0_v} <= 27'b0;
    else if(csr_we && csr_num==`TLBELO0)begin
        {tlblo0_g, tlblo0_mat, tlblo0_plv, tlblo0_d, tlblo0_v} <= (csr_mask[6:0] & csr_wdata[6:0]) | (~csr_mask[6:0] & {tlblo0_g, tlblo0_mat, tlblo0_plv, tlblo0_d, tlblo0_v});
        tlblo0_ppn <= (csr_mask[27:8] & csr_wdata[27:8]) | (~csr_mask[27:8] & tlblo0_ppn);
    end
    else if(inst_tlbrd)begin
        {tlblo0_ppn, tlblo0_g, tlblo0_mat, tlblo0_plv, tlblo0_d, tlblo0_v} <= {27{r_e}} & {r_ppn0, r_g, r_mat0, r_plv0, r_d0, r_v0};
    end
end

//TLBELO1
assign tlblo1 = {4'b0, tlblo1_ppn, 1'b0, tlblo1_g, tlblo1_mat, tlblo1_plv, tlblo1_d, tlblo1_v};
always @(posedge clk ) begin
    if(reset){tlblo1_ppn, tlblo1_g, tlblo1_mat, tlblo1_plv, tlblo1_d, tlblo1_v} <= 27'b0;
    else if(csr_we && csr_num==`TLBELO1)begin
        {tlblo1_g, tlblo1_mat, tlblo1_plv, tlblo1_d, tlblo1_v} <= (csr_mask[6:0] & csr_wdata[6:0]) | (~csr_mask[6:0] & {tlblo1_g, tlblo1_mat, tlblo1_plv, tlblo1_d, tlblo1_v});
        tlblo1_ppn <= (csr_mask[27:8] & csr_wdata[27:8]) | (~csr_mask[27:8] & tlblo1_ppn);
    end
    else if(inst_tlbrd)begin
        {tlblo1_ppn, tlblo1_g, tlblo1_mat, tlblo1_plv, tlblo1_d, tlblo1_v} <= {27{r_e}} & {r_ppn1, r_g, r_mat1, r_plv1, r_d1, r_v1};
    end
end

//ASID
assign asid = {8'b0, asid_asidbits, 6'b0, asid_asid};
assign asid_asidbits = 8'ha;
always @(posedge clk ) begin
    if(reset)asid_asid <= 10'b0;
    else if(csr_we && csr_num==`ASID)begin
        asid_asid <= (csr_mask[9:0] & csr_wdata[9:0]) | (~csr_mask[9:0] & asid_asid);
    end
    else if(inst_tlbrd)begin
        asid_asid <= {10{r_e}} & r_asid;
    end
end

//TLBRENTRY
assign tlbrentry = {tlbrentry_pa, 6'b0};
always @(posedge clk ) begin
    if(reset)tlbrentry_pa <= 26'b0;
    else if(csr_we && csr_num==`TLBRENTRY)tlbrentry_pa <= (csr_mask[31:6] & csr_wdata[31:6]) | (~csr_mask[31:6] & tlbrentry_pa);
end

//DMW0-1
assign dmw0 = {dmw0_vseg, 1'b0, dmw0_pseg, 19'b0, dmw0_mat, dmw0_plv3, 2'b0, dmw0_plv0};
assign dmw1 = {dmw1_vseg, 1'b0, dmw1_pseg, 19'b0, dmw1_mat, dmw1_plv3, 2'b0, dmw1_plv0};
always @(posedge clk ) begin
    if(reset)begin
        {dmw0_vseg, dmw0_pseg, dmw0_mat, dmw0_plv3, dmw0_plv0} <= 22'b0;
    end
    else if(csr_we && csr_num==`DMW0)begin
        dmw0_plv0 <= csr_mask[0] & csr_wdata[0] | ~csr_mask[0] & dmw0_plv0;
        {dmw0_mat, dmw0_plv3} <= csr_mask[5:3] & csr_wdata[5:3] | ~csr_mask[5:3] & {dmw0_mat, dmw0_plv3};
        dmw0_pseg <= csr_mask[27:25] & csr_wdata[27:25] | ~csr_mask[27:25] & dmw0_pseg;
        dmw0_vseg <= csr_mask[31:29] & csr_wdata[31:29] | ~csr_mask[31:29] & dmw0_vseg;
    end
end
always @(posedge clk ) begin
    if(reset)begin
        {dmw1_vseg, dmw1_pseg, dmw1_mat, dmw1_plv3, dmw1_plv0} <= 22'b0;
    end
    else if(csr_we && csr_num==`DMW1)begin
        dmw1_plv0 <= csr_mask[0] & csr_wdata[0] | ~csr_mask[0] & dmw1_plv0;
        {dmw1_mat, dmw1_plv3} <= csr_mask[5:3] & csr_wdata[5:3] | ~csr_mask[5:3] & {dmw1_mat, dmw1_plv3};
        dmw1_pseg <= csr_mask[27:25] & csr_wdata[27:25] | ~csr_mask[27:25] & dmw1_pseg;
        dmw1_vseg <= csr_mask[31:29] & csr_wdata[31:29] | ~csr_mask[31:29] & dmw1_vseg;
    end
end
endmodule
