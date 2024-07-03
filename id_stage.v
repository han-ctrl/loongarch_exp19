
`include "mycpu.vh"

module id_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from es, ms, ws
    input [`ES_TO_DS_FWD-1    :0]  es_to_ds_fwd  ,
    input [`MS_TO_DS_FWD-1    :0]  ms_to_ds_fwd  ,
    input [`WS_TO_DS_FWD-1    :0]  ws_to_ds_fwd  ,
    input                          tlb_flush_wb  ,
    input                          tlb_flush_exe ,
    //form csr
    input                          flush         ,
    input                          csr_interrupt  ,
    //from fs
    input                          fs_to_ds_valid,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to fs
    output [`BR_BUS_WD       -1:0] br_bus        ,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus
);

wire        br_stall;
wire        br_taken;
wire [31:0] br_target;

wire [31:0] ds_pc;
wire [31:0] ds_inst;

reg         ds_valid   ;
wire        ds_ready_go;

wire [11:0] alu_op;

wire        load_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        res_from_mem;
wire        dst_is_r1;
wire        gr_we;
wire        mem_we;
wire        src_reg_is_rd;
wire [4: 0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;

wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_ld_w;
wire        inst_st_w;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_lu12i_w;
wire        inst_pcaddu12i;
wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_div_wu;
wire        inst_mod_w;
wire        inst_mod_wu;
wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;
wire        inst_ld_b;
wire        inst_ld_h;
wire        inst_ld_bu;
wire        inst_ld_hu;
wire        inst_st_b;
wire        inst_st_h;
wire        inst_tlbsrch;
wire        inst_tlbwr;
wire        inst_tlbrd;
wire        inst_tlbfill;
wire        inst_invtlb;

//tlb
wire [4:0]  invtlb_op;
//exception inst
wire        inst_csrrd;
wire        inst_csrwr;
wire        inst_csrxchg;
wire        inst_ertn;
wire        inst_syscall;
wire        inst_break;
wire        inst_rdcntid_w;
wire        inst_rdcntvl_w;
wire        inst_rdcntvh_w;

wire        need_ui5;
wire        need_si12;
wire        need_ui12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;

wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire [31:0] mem_result;
wire [31:0] final_result;
wire rj_eq_rd, rj_less_rd, u_rj_less_rd;
//stall信号
reg tlb_stall_r;
wire stall, load_stall, csr_stall, tlb_stall;//load_stall and csr_stall
wire is_read_rj, is_read_rk, is_read_rd;
wire rj_conflict, rk_conflict, rd_conflict;
wire es_inst_csr, ms_inst_csr, ws_inst_csr;
wire es_inst_rdtime, ms_inst_rdtime, ws_inst_rdtime; 
wire [4:0]                    es_to_ds_dest ;
wire [4:0]                    ms_to_ds_dest ;
wire [4:0]                    ws_to_ds_dest ;
wire                          es_to_ds_load_op;
wire [31:0]                   es_to_ds_result;
wire [31:0]                   ms_to_ds_result;
wire [31:0]                   ws_to_ds_result;
//exception
wire        inst_csr;
wire        ds_csr_we;
wire        ds_ertn_flush;
wire [13:0] ds_csr_num;
wire [31:0] ds_csr_mask, ds_csr_wdata;
wire [`EXCEPTION_CODE-1:0] exception_code_in;
wire [`EXCEPTION_CODE-1:0] exception_code_out;
wire        ine;

assign op_31_26  = ds_inst[31:26];
assign op_25_22  = ds_inst[25:22];
assign op_21_20  = ds_inst[21:20];
assign op_19_15  = ds_inst[19:15];

assign rd   = ds_inst[ 4: 0];
assign rj   = ds_inst[ 9: 5];
assign rk   = ds_inst[14:10];

assign i12  = ds_inst[21:10];
assign i20  = ds_inst[24: 5];
assign i16  = ds_inst[25:10];
assign i26  = {ds_inst[ 9: 0], ds_inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~ds_inst[25];
assign inst_pcaddu12i = op_31_26_d[6'h07] & ~ds_inst[25];
assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_mul_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_div_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_mod_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];
assign inst_blt    = op_31_26_d[6'h18];
assign inst_bge    = op_31_26_d[6'h19];
assign inst_bltu   = op_31_26_d[6'h1a];
assign inst_bgeu   = op_31_26_d[6'h1b];
assign inst_ld_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_ld_bu  = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_ld_hu  = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
assign inst_st_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_st_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h5];
assign inst_csrrd  = op_31_26_d[6'h01] & ~(ds_inst[25] | ds_inst[24]) & (rj==5'h00);
assign inst_csrwr  = op_31_26_d[6'h01] & ~(ds_inst[25] | ds_inst[24]) & (rj==5'h01);
assign inst_csrxchg= op_31_26_d[6'h01] & ~(ds_inst[25] | ds_inst[24]) & ~((rj==5'h00) | (rj==5'h01));
assign inst_ertn   = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & (rk==5'b01110) & (rj==5'b0) & (rd==5'b0);
assign inst_syscall= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_break  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];
assign inst_rdcntid_w= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk==5'b11000) & (rd==5'b0);
assign inst_rdcntvl_w= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk==5'b11000) & (rj==5'b0);
assign inst_rdcntvh_w= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk==5'b11001) & (rj==5'b0);
assign inst_tlbsrch  = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & (rk == 5'h0a);
assign inst_tlbrd    = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & (rk == 5'h0b);
assign inst_tlbwr    = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & (rk == 5'h0c);
assign inst_tlbfill  = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h10] & (rk == 5'h0d);
assign inst_invtlb   = op_31_26_d[6'h01] & op_25_22_d[4'h9] & op_21_20_d[2'h0] & op_19_15_d[5'h13];

assign alu_op[ 0] = inst_add_w | inst_addi_w | inst_ld_w | inst_st_w
                    | inst_jirl | inst_bl | inst_pcaddu12i | inst_ld_b |
                    inst_ld_h | inst_ld_hu | inst_ld_bu | inst_st_h | inst_st_b;
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltui;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_slli_w | inst_sll_w;
assign alu_op[ 9] = inst_srli_w | inst_srl_w;
assign alu_op[10] = inst_srai_w | inst_sra_w;
assign alu_op[11] = inst_lu12i_w;

assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
//assign need_si12  =  inst_addi_w | inst_ld_w | inst_st_w | inst_slti | inst_sltui |
//                     inst_ld_h | inst_ld_bu | inst_ld_hu | inst_st_b | inst_st_h;
assign need_ui12  =  inst_andi | inst_ori | inst_xori;
assign need_si16  =  inst_jirl | inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu;
assign need_si20  =  inst_lu12i_w | inst_pcaddu12i;
assign need_si26  =  inst_b | inst_bl;
assign src2_is_4  =  inst_jirl | inst_bl;

assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
             need_ui5  ? rk                         :
             need_ui12 ? {20'b0, i12[11:0]}         :
            /*need_si12*/{{20{i12[11]}}, i12[11:0]} ;

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                /*need_si16*/{{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w | inst_blt |
                        inst_bge | inst_bltu | inst_bgeu | inst_st_b | inst_st_h |
                        inst_csr;

assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;

assign src2_is_imm   = inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w |
                       inst_ld_w   | inst_st_w   | inst_lu12i_w| inst_jirl   |
                       inst_bl     | inst_pcaddu12i | inst_slti | inst_sltui  |
                       inst_andi   | inst_ori    | inst_xori   | inst_ld_b | inst_ld_h |
                       inst_ld_bu  | inst_ld_hu  | inst_st_b   | inst_st_h;

assign res_from_mem  = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
assign dst_is_r1     = inst_bl;
assign gr_we         = ~(inst_st_w || inst_beq || inst_bne || inst_b 
                        || inst_blt || inst_bge || inst_bltu || inst_bgeu
                        || inst_st_h || inst_st_b || inst_ertn || inst_syscall || inst_break 
                        || inst_tlbfill || inst_invtlb || inst_tlbwr || inst_tlbrd || inst_tlbsrch);
assign mem_we        = inst_st_w | inst_st_b | inst_st_h;
assign dest          =  dst_is_r1 ?     5'd1 : 
                        inst_rdcntid_w ? rj  : rd;

assign rf_raddr1 = {5{!(inst_tlbrd | inst_tlbsrch | inst_tlbwr | inst_tlbfill)}} & rj;
assign rf_raddr2 = {5{!(inst_tlbrd | inst_tlbsrch | inst_tlbwr | inst_tlbfill)}} & (src_reg_is_rd ? rd :rk);
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2 (rf_raddr2),
    .rdata2 (rf_rdata2),
    .we     (rf_we    ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata )
    );

assign rj_value  = rj_conflict ? ((rj == es_to_ds_dest) ? es_to_ds_result :
                              (rj == ms_to_ds_dest) ? ms_to_ds_result : ws_to_ds_result)
                            : rf_rdata1;

assign rkd_value = rk_conflict ? ((rk == es_to_ds_dest) ? es_to_ds_result :
                            (rk == ms_to_ds_dest) ? ms_to_ds_result : ws_to_ds_result) : 
                   rd_conflict ? ((rd == es_to_ds_dest) ? es_to_ds_result :
                            (rd == ms_to_ds_dest) ? ms_to_ds_result : ws_to_ds_result) : 
                            rf_rdata2;

assign rj_eq_rd = (rj_value == rkd_value);
assign rj_less_rd = ($signed(rj_value) < $signed(rkd_value)); 
assign u_rj_less_rd = (rj_value < rkd_value);

//Forward to if stage
assign br_taken = (   inst_beq  &&  rj_eq_rd
                   || inst_bne  && !rj_eq_rd
                   || inst_jirl
                   || inst_bl
                   || inst_b
                   || inst_blt  &&  rj_less_rd
                   || inst_bge  && !rj_less_rd
                   || inst_bltu &&  u_rj_less_rd
                   || inst_bgeu && !u_rj_less_rd
)  && ds_valid && ~stall && ~flush;//br_load时阻塞
assign br_target = (inst_beq || inst_bne || inst_bl || inst_b || inst_blt || 
                    inst_bge || inst_bltu || inst_bgeu) ? (ds_pc + br_offs) :
                                        /*inst_jirl*/ (rj_value + jirl_offs);
assign br_stall = ds_valid && br_taken;
assign br_bus = {br_stall, br_taken, br_target};


//exception
assign invtlb_op = rd;
assign ex_invtlb = inst_invtlb && (invtlb_op[2:0]==3'b111 || invtlb_op[4:3]!=2'b00);
assign inst_csr = inst_csrrd | inst_csrwr | inst_csrxchg;
assign ds_csr_num = ds_inst[23:10];
assign ds_csr_mask = inst_csrxchg ? rj_value : 32'hffffffff;
assign ds_csr_wdata = rkd_value;
assign ds_csr_we = inst_csrwr | inst_csrxchg;
assign ds_ertn_flush = inst_ertn && (exception_code_out==`EXCEPTION_CODE'b0);
assign ine = (~inst_add_w & ~inst_sub_w & ~inst_slt & ~inst_sltu & ~inst_nor &
                ~inst_and & ~inst_or & ~inst_xor & ~inst_mul_w & ~inst_mulh_w & ~inst_mulh_wu &
                ~inst_div_w & ~inst_mod_w & ~inst_div_wu & ~inst_mod_wu & ~inst_sll_w &
                ~inst_srl_w & ~inst_sra_w & ~inst_slli_w & ~inst_srli_w & ~inst_srai_w &
                ~inst_slti & ~inst_sltui & ~inst_addi_w & ~inst_andi & ~inst_ori &
                ~inst_xori & ~inst_ld_b & ~inst_ld_h & ~inst_ld_bu & ~inst_ld_hu &
                ~inst_ld_w & ~inst_st_b & ~inst_st_h & ~inst_st_w  & ~inst_jirl & ~inst_b & ~inst_bl & 
                ~inst_beq & ~inst_bne & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu & ~inst_lu12i_w 
                & ~inst_pcaddu12i & ~inst_syscall & ~inst_break & ~inst_csrrd & ~inst_csrwr & ~inst_csrxchg & ~inst_ertn
                & ~inst_rdcntid_w & ~inst_rdcntvl_w & ~inst_rdcntvh_w & ~inst_tlbsrch & ~inst_tlbwr & ~inst_tlbrd & ~inst_invtlb & ~inst_tlbfill 
                | ex_invtlb
                ) 
                && (exception_code_in == `EXCEPTION_CODE'b0); 
assign exception_code_out = exception_code_in | {2'b0, ine, inst_break, inst_syscall, 8'b0, csr_interrupt};

// 数据接收、传出
reg  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;

assign {exception_code_in,
        ds_inst,
        ds_pc  } = fs_to_ds_bus_r;
assign {rf_we   ,  //37:37
        rf_waddr,  //36:32
        rf_wdata   //31:0
       } = ws_to_rf_bus;
assign ds_to_es_bus = { inst_tlbsrch,
                        inst_invtlb,
                        inst_tlbrd,
                        inst_tlbwr,
                        inst_tlbfill,
                        invtlb_op,
                        inst_rdcntvh_w,
                        inst_rdcntvl_w,
                        inst_rdcntid_w ,// 261:261
                        inst_csr       ,//1 260:260
                        ds_csr_we      ,//1
                        ds_csr_num     ,//14
                        ds_csr_mask    ,//32
                        ds_csr_wdata   ,//32
                        ds_ertn_flush  ,//1
                        exception_code_out ,//13
                        alu_op       ,   // 12 154:166
                        load_op      ,   // 1
                        src1_is_pc   ,   // 1
                        src2_is_imm  ,   // 1
                        src2_is_4    ,   // 1
                        gr_we        ,   // 1
                        mem_we       ,   // 1
                        dest         ,   // 5
                        imm          ,   // 32
                        rj_value     ,   // 32
                        rkd_value    ,   // 32
                        ds_pc        ,   // 32
                        res_from_mem ,   // 1  152
                        inst_mul_w   ,  //14:14
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
                        inst_ld_hu      //0:0
                    };
assign { es_inst_rdtime,
         es_inst_csr,
         es_to_ds_dest,
         es_to_ds_result,
         es_to_ds_load_op
        }  =  es_to_ds_fwd;

assign {ms_inst_rdtime,
        ms_inst_csr, 
        ms_to_ds_dest,
        ms_to_ds_result
        }  =  ms_to_ds_fwd;

assign {ws_inst_rdtime,
        ws_inst_csr,
        ws_to_ds_dest,
        ws_to_ds_result
        }  =  ws_to_ds_fwd;


assign load_op        = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
assign ds_ready_go    = ds_valid && ~stall;
assign ds_allowin     = !ds_valid || (ds_ready_go && es_allowin && !tlb_stall) || flush;
assign ds_to_es_valid = ds_valid && ds_ready_go && ~flush;
always @(posedge clk) begin
    if (reset) begin
        ds_valid <= 1'b0;
    end
    else if (ds_allowin) begin
        ds_valid <= fs_to_ds_valid;
    end
end
always @(posedge clk ) begin
    if(reset)fs_to_ds_bus_r <= `FS_TO_DS_BUS_WD'b0;
    else if (fs_to_ds_valid && ds_allowin) begin
        fs_to_ds_bus_r <= fs_to_ds_bus;
    end
end

//stall
assign is_read_rj = inst_add_w | inst_sub_w | inst_slt |inst_sltu | inst_nor | inst_and | inst_or | inst_xor | 
                    inst_slli_w | inst_srai_w | inst_srli_w | inst_addi_w | inst_st_w | inst_ld_w | inst_beq | 
                    inst_bne | inst_jirl | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori |
                    inst_sll_w | inst_srl_w | inst_sra_w | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w
                    | inst_div_wu | inst_mod_w | inst_mod_wu | inst_blt | inst_bge | inst_bltu | inst_bgeu |
                    inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu | inst_st_b | inst_st_h | inst_csrxchg | inst_rdcntid_w;

assign is_read_rk = inst_add_w | inst_sub_w | inst_slt | inst_sltu | inst_nor | inst_and | inst_or | inst_xor
                    | inst_sll_w | inst_srl_w | inst_sra_w | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w
                    | inst_div_wu | inst_mod_w | inst_mod_wu;
                    
assign is_read_rd = inst_beq | inst_bne | inst_st_w| inst_blt | inst_bge | inst_bltu | inst_bgeu
                    | inst_st_b | inst_st_h | inst_csrwr | inst_csrxchg | inst_rdcntvh_w | inst_rdcntvl_w;

assign rj_conflict = is_read_rj && rj!=5'b0 && ( (es_to_ds_dest==rj) || (ms_to_ds_dest==rj) || (ws_to_ds_dest==rj) );
assign rk_conflict = is_read_rk && rk!=5'b0 && ( (es_to_ds_dest==rk) || (ms_to_ds_dest==rk) || (ws_to_ds_dest==rk) );
assign rd_conflict = is_read_rd && rd!=5'b0 && ( (es_to_ds_dest==rd) || (ms_to_ds_dest==rd) || (ws_to_ds_dest==rd) );
assign load_stall = (es_to_ds_load_op ) &&  ((rj==es_to_ds_dest)&&rj_conflict)||
                                            ((rk==es_to_ds_dest)&&rk_conflict)||
                                            ((rd==es_to_ds_dest)&&rd_conflict);
assign csr_stall = (es_inst_csr || ms_inst_csr || ws_inst_csr || es_inst_rdtime || ms_inst_rdtime || ws_inst_rdtime) && (rj_conflict || rk_conflict || rd_conflict);
assign stall = csr_stall || load_stall;
assign tlb_stall = (tlb_stall_r || (inst_tlbfill | inst_tlbwr | inst_invtlb)) && !(tlb_flush_exe || tlb_flush_wb);
always @(posedge clk ) begin
    if(reset)begin
        tlb_stall_r <= 1'b0;
    end
    else if(tlb_flush_exe || tlb_flush_wb)begin
        tlb_stall_r <= 1'b0;
    end
    else if(inst_tlbfill | inst_tlbwr | inst_invtlb | inst_tlbrd | inst_tlbsrch)begin
        tlb_stall_r <= 1'b1;
    end
end
endmodule
