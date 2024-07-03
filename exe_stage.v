`include "mycpu.vh"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //from wb
    input                          ws_exception  ,
    //from csr
    input                          flush         ,
    input [31:13]                  tlbehi_vppn   ,
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
    input [9:0]                    asid_asid     ,
    //from ms
    input                          ms_exception  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ds
    output [`ES_TO_DS_FWD-1    :0] es_to_ds_fwd  ,
    output                         tlb_flush_exe ,
    // data sram interface(write)
    output                         data_sram_req ,
    output                         data_sram_wr  ,
    input                          data_sram_addr_ok,
    output  [2:0]                  data_sram_size,
    output  [3:0]                  data_sram_wstrb,
    output  [31:0]                 data_sram_addr,
    output  [31:0]                 data_sram_wdata,
    //from tlb
    input  wire                    s1_found,
    input  wire [ 3:0]             s1_index,
    input  wire [19:0]             s1_ppn,
    input  wire [ 5:0]             s1_ps,
    input  wire [ 1:0]             s1_plv,
    input  wire [ 1:0]             s1_mat,
    input  wire                    s1_d,
    input  wire                    s1_v,
    //to tlb
    output wire [18:0]             s1_vppn,
    output wire                    s1_va_bit12,
    output wire [ 9:0]             s1_asid,
    output wire [4:0]              invtlb_op,
    output wire                    invtlb_valid
);

reg         es_valid      ;
wire        es_ready_go   ;
reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
//ALU
wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;
wire [31:0] es_result  ;
wire        is_alu;
wire [11:0] alu_op     ;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_is_4;
wire        res_from_mem;
wire        dst_is_r1;
wire        es_gr_we;
wire        es_mem_we;
wire [4: 0] es_dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire [31:0] es_pc;
wire        es_load_op;

wire        inst_mul_w   ;
wire        inst_mulh_w  ;
wire        inst_mulh_wu ;
wire        inst_div_w   ;
wire        inst_div_wu  ;
wire        inst_mod_w   ;
wire        inst_mod_wu  ;
wire        inst_st_w    ;
wire        inst_st_b    ;
wire        inst_st_h    ;
wire        inst_ld_w    ;
wire        inst_ld_b    ;
wire        inst_ld_h    ;
wire        inst_ld_bu   ;
wire        inst_ld_hu   ;
wire        ld,st;
// Div & Divu
wire [31:0] divider_dividend;
wire [31:0] divider_divisor;
wire [63:0] unsigned_divider_res;
wire [63:0] signed_divider_res;

assign divider_dividend = rj_value;
assign divider_divisor  = rkd_value;

wire unsigned_dividend_tready;
wire unsigned_dividend_tvalid;
wire unsigned_divisor_tready;
wire unsigned_divisor_tvalid;
wire unsigned_dout_tvalid;

wire signed_dividend_tready;
wire signed_dividend_tvalid;
wire signed_divisor_tready;
wire signed_divisor_tvalid;
wire signed_dout_tvalid;
// Mult & Multu
wire [31:0] mult_src1;
wire [31:0] mult_src2;
wire [63:0] unsigned_mult_res;
wire [63:0] signed_mult_res;

wire        inst_csr;
wire        es_csr_we_in, es_csr_we_out;
wire        es_ertn_flush;
wire [13:0] es_csr_num_in, es_csr_num_out;
wire [31:0] es_csr_mask_in, es_csr_wdata_in, es_csr_mask_out, es_csr_wdata_out;
wire [`EXCEPTION_CODE-1:0] exception_code_in;
wire [`EXCEPTION_CODE-1:0] exception_code_out;
wire        ale;
wire [31:0] st_ld_addr;
wire        inst_rdcntid_w;
reg  [63:0] counter64;
wire        inst_rdcntvh_w;
wire        inst_rdcntvl_w;
wire        es_inst_rdtime;
wire [3:0]  st_w_strb, st_h_strb, st_b_strb;
wire        data_sram_req_reg;

wire        inst_tlbwr;
wire        inst_tlbrd;
wire        inst_tlbfill;
wire        inst_invtlb;

wire [31:0] direct_pa, dmw0_pa, dmw1_pa, tlb_pa, data_pa;
wire direct_pa_valid, dmw0_pa_valid, dmw1_pa_validl, tlb_pa_valid;
wire pil;//load 操作页无效例外
wire pis;//store 操作页无效例外
wire ppi;//页特权等级不合规例外
wire pme;//页修改例外
wire tlbr;//TLB 重填例外

//tlb 
assign ld = inst_ld_w | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu;
assign st = inst_st_w | inst_st_h | inst_st_b;
assign s1_vppn     =  {19{inst_invtlb}}  & rkd_value[31:13] 
                    | {19{inst_tlbsrch}} & tlbehi_vppn
                    | {19{ld | st}}      & alu_result[31:13];
assign s1_va_bit12 = inst_invtlb & rkd_value[12] | (ld | st) & alu_result[12];
assign s1_asid     =  {10{inst_invtlb}}  & rj_value[9:0] 
                    | {10{inst_tlbsrch | ld | st}} & asid_asid;
assign invtlb_valid     = inst_invtlb && es_valid && !flush;
assign es_csr_we_out    = inst_tlbsrch ? 1'b1 : es_csr_we_in;
assign es_csr_num_out   = inst_tlbsrch ? `TLBIDX : es_csr_num_in;
assign es_csr_mask_out  = inst_tlbsrch ? {1'b1, 27'b0, 4'hf} : es_csr_mask_in;
assign es_csr_wdata_out = inst_tlbsrch ? {~s1_found, 27'b0, s1_index} : es_csr_wdata_in;
assign tlb_flush_exe    = inst_tlbsrch | inst_invtlb;
//虚实地址转换
assign data_pa = {32{direct_pa_valid}} & direct_pa |
                 {32{dmw0_pa_valid}} & dmw0_pa     |
                 {32{dmw1_pa_valid}} & dmw1_pa     |
                 {32{tlb_pa_valid}} & tlb_pa       ;
assign {direct_pa, direct_pa_valid} = {alu_result, !crmd_pg && crmd_da};
assign dmw0_pa       = {dmw0_pseg, alu_result[28:0]};
assign dmw0_pa_valid = (crmd_plv==2'b0&&dmw0_plv0 || crmd_plv==2'b11&&dmw0_plv3) && dmw0_vseg==alu_result[31:29] && crmd_pg && !crmd_da;
assign dmw1_pa       = {dmw1_pseg, alu_result[28:0]};
assign dmw1_pa_valid = (crmd_plv==2'b0&&dmw1_plv0 || crmd_plv==2'b11&&dmw1_plv3) && dmw1_vseg==alu_result[31:29] && crmd_pg && !crmd_da;
assign tlb_pa        = {32{s1_ps==6'd12}}&{s1_ppn, alu_result[11:0]} | {32{s1_ps==6'd21}}&{s1_ppn[19:9], alu_result[20:0]};
assign tlb_pa_valid  = s1_found && crmd_pg && !crmd_da && !dmw0_pa_valid && !dmw1_pa_valid;

//Forward 
wire [4:0]       es_to_ds_dest;
wire [31:0]      es_to_ds_result;
wire             es_to_ds_load_op;
assign es_to_ds_load_op = es_load_op;
assign es_to_ds_dest    = es_dest & {5{es_valid && es_gr_we}};
assign es_to_ds_fwd = { es_inst_rdtime,
                        inst_csr,
                        es_to_ds_dest,
                        es_to_ds_result,
                        es_to_ds_load_op
                    };
assign es_inst_rdtime =  inst_rdcntid_w;
//ALU
assign alu_src1 = src1_is_pc  ? es_pc  : rj_value;
assign alu_src2 = src2_is_imm ? imm : rkd_value;
assign es_to_ds_result = es_result;
alu u_alu(
    .alu_op     (alu_op    ),
    .alu_src1   (alu_src1  ),
    .alu_src2   (alu_src2  ),
    .alu_result (alu_result)
    );

//stable counter
always @(posedge clk ) begin
    if(reset) counter64 <= 64'b0;
    else counter64 <= counter64 + 1;
end

//Sram 
assign data_sram_req_reg = !flush && es_valid && ms_allowin && (ld | st);
assign data_sram_req   = data_sram_req_reg;
assign data_sram_wr    = st;
assign data_sram_size  = {3'b000 & {2{inst_ld_b | inst_ld_bu | inst_ld_b}}} |
                         {3'b001 & {2{inst_ld_h | inst_ld_hu | inst_st_h}}} |
                         {3'b010 & {2{inst_ld_w | inst_st_w}}};
assign st_b_strb       = {{4{inst_st_b && data_sram_addr[1:0]==2'b00}} & 4'b0001 } |
                         {{4{inst_st_b && data_sram_addr[1:0]==2'b01}} & 4'b0010 } |
                         {{4{inst_st_b && data_sram_addr[1:0]==2'b10}} & 4'b0100 } |
                         {{4{inst_st_b && data_sram_addr[1:0]==2'b11}} & 4'b1000 } ;
assign st_h_strb       = {{4{inst_st_h && data_sram_addr[1:0]==2'b00}} & 4'b0011 } |
                         {{4{inst_st_h && data_sram_addr[1:0]==2'b10}} & 4'b1100 } ;
assign st_w_strb       = {{4{inst_st_w && data_sram_addr[1:0]==2'b00}} & 4'b1111 } ;
assign data_sram_wstrb = {4{es_valid && ~ms_exception && ~ws_exception && ~|exception_code_out}} & (st_b_strb | st_h_strb | st_w_strb);
                                                                                                                                 
assign data_sram_addr  = data_pa;
assign data_sram_wdata = {32{inst_st_b}} & {4{rkd_value[7:0]}}  |
                         {32{inst_st_h}} & {2{rkd_value[15:0]}} | 
                         {32{!inst_st_b & !inst_st_h}} & rkd_value;           

// Mult & Multu
assign mult_src1 = rj_value;
assign mult_src2 = rkd_value;

assign unsigned_mult_res = mult_src1 * mult_src2;
assign signed_mult_res   = $signed(mult_src1) * $signed(mult_src2);

// Div & Divu
unsigned_divider u_unsigned_divider (
    .aclk                   (clk),
    .s_axis_dividend_tdata  (divider_dividend),//被除数
    .s_axis_dividend_tready (unsigned_dividend_tready),//应答信号，可以进行计算
    .s_axis_dividend_tvalid (unsigned_dividend_tvalid),//请求信号，有数据要计算，上升沿采集
    .s_axis_divisor_tdata   (divider_divisor),//除数
    .s_axis_divisor_tready  (unsigned_divisor_tready),
    .s_axis_divisor_tvalid  (unsigned_divisor_tvalid),
    .m_axis_dout_tdata      (unsigned_divider_res),//商、余数
    .m_axis_dout_tvalid     (unsigned_dout_tvalid)
);

signed_divider u_signed_divider (
    .aclk                   (clk),
    .s_axis_dividend_tdata  (divider_dividend),
    .s_axis_dividend_tready (signed_dividend_tready),
    .s_axis_dividend_tvalid (signed_dividend_tvalid),
    .s_axis_divisor_tdata   (divider_divisor),
    .s_axis_divisor_tready  (signed_divisor_tready),
    .s_axis_divisor_tvalid  (signed_divisor_tvalid),
    .m_axis_dout_tdata      (signed_divider_res),
    .m_axis_dout_tvalid     (signed_dout_tvalid)
);

// Divider status control
reg  unsigned_dividend_sent;//已送出数据
reg  unsigned_divisor_sent;
reg  unsigned_divider_done;//除法完成

assign unsigned_dividend_tvalid = es_valid && (inst_div_wu| inst_mod_wu) && !unsigned_dividend_sent;
assign unsigned_divisor_tvalid  = es_valid && (inst_div_wu| inst_mod_wu) && !unsigned_divisor_sent;

always @ (posedge clk) begin
    if (reset) begin
        unsigned_dividend_sent <= 1'b0;
    end else if (unsigned_dividend_tready && unsigned_dividend_tvalid) begin
        unsigned_dividend_sent <= 1'b1;
    end else if (es_ready_go && ms_allowin) begin
        unsigned_dividend_sent <= 1'b0;
    end
    
    if (reset) begin
        unsigned_divisor_sent <= 1'b0;
    end else if (unsigned_divisor_tready && unsigned_divisor_tvalid) begin
        unsigned_divisor_sent <= 1'b1;
    end else if (es_ready_go && ms_allowin) begin
        unsigned_divisor_sent <= 1'b0;
    end

    if (reset) begin
        unsigned_divider_done <= 1'b0;
    end else if (es_ready_go && !ms_allowin) begin
        unsigned_divider_done <= 1'b1;
    end else if (ms_allowin) begin
        unsigned_divider_done <= 1'b0;
    end
end

reg  signed_dividend_sent;
reg  signed_divisor_sent;
reg  signed_divider_done;

assign signed_dividend_tvalid = es_valid && (inst_div_w| inst_mod_w) && !signed_dividend_sent;
assign signed_divisor_tvalid  = es_valid && (inst_div_w| inst_mod_w) && !signed_divisor_sent;

always @ (posedge clk) begin
    if (reset) begin
        signed_dividend_sent <= 1'b0;
    end else if (signed_dividend_tready && signed_dividend_tvalid) begin
        signed_dividend_sent <= 1'b1;
    end else if (es_ready_go && ms_allowin) begin
        signed_dividend_sent <= 1'b0;
    end
    
    if (reset) begin
        signed_divisor_sent <= 1'b0;
    end else if (signed_divisor_tready && signed_divisor_tvalid) begin
        signed_divisor_sent <= 1'b1;
    end else if (es_ready_go && ms_allowin) begin
        signed_divisor_sent <= 1'b0;
    end

    if (reset) begin
        signed_divider_done <= 1'b0;
    end else if (es_ready_go && !ms_allowin) begin
        signed_divider_done <= 1'b1;
    end else if (ms_allowin) begin
        signed_divider_done <= 1'b0;
    end
end

assign is_alu = |alu_op;
assign es_result     =  {{32{inst_div_w}}  & signed_divider_res[63:32]}   |
                        {{32{inst_mod_w}}  & signed_divider_res[31:0]}    |
                        {{32{inst_div_wu}} & unsigned_divider_res[63:32]} |
                        {{32{inst_mod_wu}} & unsigned_divider_res[31:0]}  |
                        {{32{inst_mul_w}}  & signed_mult_res[31:0]}       |
                        {{32{inst_mulh_w}} & signed_mult_res[63:32]}      |
                        {{32{inst_mulh_wu}}& unsigned_mult_res[63:32]}    |
                        {{32{is_alu}}      & alu_result[31:0]}            |
                        {{32{inst_rdcntvh_w}}&counter64[63:32]}           |
                        {{32{inst_rdcntvl_w}}&counter64[31:0]}            ;       

//异常处理
assign pil    = ld && tlb_pa_valid  && !s1_v && es_valid;
assign pis    = st && tlb_pa_valid  && !s1_v && es_valid;
assign ppi    = (ld | st) && tlb_pa_valid && s1_v && crmd_plv==2'b11 && s1_plv==2'b00 && es_valid;
assign pme    = st && tlb_pa_valid && s1_v && !ppi && !s1_d && es_valid;
assign tlbr   = (ld | st) && !direct_pa_valid && !dmw0_pa_valid && !dmw1_pa_valid && !tlb_pa_valid;
assign st_ale = (inst_st_w && data_sram_addr[1:0]!=2'b00) | (inst_st_h && data_sram_addr[0]!=1'b0); 
assign ld_ale = {inst_ld_w && data_sram_addr[1:0]!=2'b00} | {(inst_ld_h||inst_ld_hu)&&data_sram_addr[0]!=1'b0};
assign ale    = (st_ale | ld_ale) && es_valid;
assign exception_code_out = exception_code_in | {1'b0, tlbr, 3'b0, ale, 2'b0,  ppi, pme, 1'b0, pis, pil, 1'b0};

//数据接收
assign es_ready_go    = es_valid && (~data_sram_req_reg || data_sram_addr_ok) && //有发出请求而没被接收时，ready_go=0
                ((inst_div_w | inst_mod_w)     ? signed_dout_tvalid || signed_divider_done :
                 (inst_div_wu| inst_mod_wu)    ? unsigned_dout_tvalid || unsigned_divider_done :
                 1'b1) ;
assign es_allowin     = !es_valid || (es_ready_go && ms_allowin) || flush;
assign es_to_ms_valid =  es_valid && es_ready_go && ~flush;
always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;
    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end
end
always @(posedge clk ) begin
    if(reset)ds_to_es_bus_r <= `DS_TO_ES_BUS_WD'b0;
    else if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end
assign es_to_ms_bus = { inst_tlbrd, //207:207
                        inst_tlbwr,
                        inst_tlbfill,
                        data_sram_req_reg,
                        inst_rdcntvh_w,
                        inst_rdcntvl_w,
                        inst_rdcntid_w, 
                        alu_result, // 200:169
                        inst_csr,       //1 168:168
                        es_csr_we_out   ,//1
                        es_csr_num_out ,//14
                        es_csr_mask_out,//32
                        es_csr_wdata_out,//32
                        es_ertn_flush  ,//1
                        exception_code_out ,//13
                        inst_ld_b    ,  //74:74 
                        inst_ld_h    ,
                        inst_ld_bu   ,
                        inst_ld_hu   ,
                        res_from_mem&&~ld_ale ,  //70:70 1
                        es_gr_we     ,  //69:69 1
                        es_dest      ,  //68:64 5
                        es_result    ,  //63:32 32
                        es_pc           //31:0  32
                      };      
assign {inst_tlbsrch,//273:273
        inst_invtlb,
        inst_tlbrd,
        inst_tlbwr,
        inst_tlbfill,
        invtlb_op,
        inst_rdcntvh_w,
        inst_rdcntvl_w,
        inst_rdcntid_w ,
        inst_csr,       //1
        es_csr_we_in   ,//1
        es_csr_num_in  ,//14
        es_csr_mask_in ,//32
        es_csr_wdata_in,//32
        es_ertn_flush  ,//1
        exception_code_in ,//13
        alu_op,
        es_load_op,
        src1_is_pc,
        src2_is_imm,
        src2_is_4,
        es_gr_we,
        es_mem_we,
        es_dest,
        imm,
        rj_value,
        rkd_value,
        es_pc,
        res_from_mem,
        inst_mul_w   ,
        inst_mulh_w  ,
        inst_mulh_wu ,
        inst_div_w   ,
        inst_div_wu  ,
        inst_mod_w   ,
        inst_mod_wu  ,
        inst_st_w    ,
        inst_st_b    ,
        inst_st_h    ,
        inst_ld_w    ,
        inst_ld_b    ,
        inst_ld_h    ,
        inst_ld_bu   ,
        inst_ld_hu   
       } = ds_to_es_bus_r;

endmodule
