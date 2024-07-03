`include "mycpu.vh"

module mycpu_top(
    input         aclk,
    input         aresetn,
    //read request interface
    output wire [ 3:0]  arid,
    output wire [31:0]  araddr,
    output wire [ 7:0]  arlen,
    output wire [ 2:0]  arsize,
    output wire [ 1:0]  arburst,
    output wire [ 1:0]  arlock,
    output wire [ 3:0]  arcache,
    output wire [ 2:0]  arprot,
    output wire         arvalid,
    input  wire         arready,
    //read data interface
    input  wire [ 3:0]  rid,
    input  wire [31:0]  rdata,
    input  wire [ 1:0]  rresp,
    input  wire         rlast,
    input  wire         rvalid,
    output wire         rready,
    //write request interface
    output wire [ 3:0]  awid,
    output wire [31:0]  awaddr,
    output wire [ 7:0]  awlen,
    output wire [ 2:0]  awsize,
    output wire [ 1:0]  awburst,
    output wire [ 1:0]  awlock,
    output wire [ 3:0]  awcache,
    output wire [ 2:0]  awprot,
    output wire         awvalid,
    input  wire         awready,
    //write data interface
    output wire [ 3:0]  wid,
    output wire [31:0]  wdata,
    output wire [ 3:0]  wstrb,
    output wire         wlast,
    output wire         wvalid,
    input  wire         wready,
    //write response interface
    input  wire [ 3:0]  bid,
    input  wire [ 1:0]  bresp,
    input  wire         bvalid,
    output wire         bready,
    // trace debug interface
    output [31:0] debug_wb_pc,
    output [ 3:0] debug_wb_rf_we,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge aclk) reset <= ~aresetn;

wire [31:0]  csr_rdata;
wire [31:0]  exception_pc;
wire         csr_exception;
wire         flush;
wire         ms_exception;
wire         ws_exception;
wire         ds_allowin;
wire         es_allowin;
wire         ms_allowin;
wire         ws_allowin;
wire         fs_to_ds_valid;
wire         ds_to_es_valid;
wire         es_to_ms_valid;
wire         ms_to_ws_valid;
wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus;
wire [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus;
wire [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus;
wire [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus;
wire [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus;
wire [`BR_BUS_WD       -1:0] br_bus;
wire [`ES_TO_DS_FWD    -1:0] es_to_ds_fwd;
wire [`MS_TO_DS_FWD    -1:0] ms_to_ds_fwd;
wire [`WS_TO_DS_FWD    -1:0] ws_to_ds_fwd;
wire [`TO_CSR_BUS      -1:0] to_csr_bus;
//tlb
wire [18:0] s0_vppn;
wire        s0_va_bit12;
wire [ 9:0] s0_asid;
wire        s0_found;
wire [ 3:0] s0_index;
wire [19:0] s0_ppn;
wire [ 5:0] s0_ps;
wire [ 1:0] s0_plv;
wire [ 1:0] s0_mat;
wire        s0_d;
wire        s0_v;

wire [18:0] s1_vppn;
wire        s1_va_bit12;
wire [ 9:0] s1_asid;
wire        s1_found;
wire [ 3:0] s1_index;
wire [19:0] s1_ppn;
wire [ 5:0] s1_ps;
wire [ 1:0] s1_plv;
wire [ 1:0] s1_mat;
wire        s1_d;
wire        s1_v;

wire [4:0]  invtlb_op;
wire        invtlb_valid;

wire        we, w_e, w_g, w_d0, w_v0, w_d1, w_v1;
wire        r_e, r_g, r_d0, r_v0, r_d1, r_v1;
wire [$clog2(`TLBNUM)-1:0] w_index, r_index;
wire [18:0] w_vppn, r_vppn;
wire [5:0]  w_ps, r_ps;
wire [9:0]  w_asid, r_asid;
wire [19:0] w_ppn0, w_ppn1, r_ppn0, r_ppn1;
wire [1:0]  w_plv0, w_plv1, w_mat0, w_mat1, r_plv0, r_plv1, r_mat0, r_mat1;
wire [31:13] tlbehi_vppn;
wire [9:0]  asid_asid;
// inst sram interface
wire        inst_sram_req;//请求
wire        inst_sram_wr;//1写请求，0读请求
wire [ 2:0] inst_sram_size;//传输的字节数 00-1byte 01-2byte 10-4byte 100-16byte
wire [ 3:0] inst_sram_wstrb;//写使能
wire [31:0] inst_sram_addr;
wire [31:0] inst_sram_wdata;
wire        inst_sram_addr_ok;//地址接收  或地址和数据接收完成
wire        inst_sram_data_ok;//数据返回 或数据写入完成
wire  [31:0]inst_sram_rdata;
// data sram interface
wire        data_sram_req;
wire        data_sram_wr;
wire [ 2:0] data_sram_size;   
wire [ 3:0] data_sram_wstrb;
wire [31:0] data_sram_addr;
wire [31:0] data_sram_wdata;
wire        data_sram_addr_ok;
wire        data_sram_data_ok;
wire  [31:0]data_sram_rdata;
wire        tlb_flush_wb;
wire        tlb_flush_exe;

wire [1:0]  crmd_plv;
wire        crmd_da, crmd_pg;
wire        dmw0_plv0, dmw0_plv3;
wire [2:0]  dmw0_pseg, dmw0_vseg;
wire        dmw1_plv0, dmw1_plv3;
wire [2:0]  dmw1_pseg, dmw1_vseg;
// IF stage
if_stage u_if_stage(
    .clk            (aclk            ),
    .reset          (reset          ),
    //allowin
    .ds_allowin     (ds_allowin     ),
    //brbus
    .br_bus         (br_bus         ),
    //from csr
    .flush          (flush          ),
    .exception_pc   (exception_pc   ),
    .crmd_plv       (crmd_plv       ),
    .crmd_da        (crmd_da        ),
    .crmd_pg        (crmd_pg        ),
    .dmw0_plv0      (dmw0_plv0      ),
    .dmw0_plv3      (dmw0_plv3      ),
    .dmw0_pseg      (dmw0_pseg      ),
    .dmw0_vseg      (dmw0_vseg      ),
    .dmw1_plv0      (dmw1_plv1      ),
    .dmw1_plv3      (dmw1_plv3      ),
    .dmw1_pseg      (dmw1_pseg      ),
    .dmw1_vseg      (dmw1_vseg      ),
    .asid_asid      (asid_asid      ),
    //outputs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    // inst sram interface
    .inst_sram_req    (inst_sram_req    ),
    .inst_sram_wr     (inst_sram_wr     ),
    .inst_sram_size   (inst_sram_size   ),
    .inst_sram_wstrb  (inst_sram_wstrb  ),
    .inst_sram_addr   (inst_sram_addr   ),
    .inst_sram_wdata  (inst_sram_wdata  ),
    .inst_sram_addr_ok(inst_sram_addr_ok),
    .inst_sram_data_ok(inst_sram_data_ok),
    .inst_sram_rdata  (inst_sram_rdata  ),
    //from tlb
    .s0_found         (s0_found         ),
    .s0_index         (s0_index         ),
    .s0_ppn           (s0_ppn           ),
    .s0_ps            (s0_ps            ),
    .s0_plv           (s0_plv           ),
    .s0_mat           (s0_mat           ),
    .s0_d             (s0_d             ),
    .s0_v             (s0_v             ),
    //to tlb
    .s0_vppn          (s0_vppn          ),
    .s0_va_bit12      (s0_va_bit12      ),
    .s0_asid          (s0_asid          )
);
// ID stage
id_stage u_id_stage(
    .clk            (aclk            ),
    .reset          (reset          ),
    //allowin
    .es_allowin     (es_allowin     ),
    .ds_allowin     (ds_allowin     ),
    //from es, ms, ws
    .es_to_ds_fwd   (es_to_ds_fwd   ),
    .ms_to_ds_fwd   (ms_to_ds_fwd   ),
    .ws_to_ds_fwd   (ws_to_ds_fwd   ),
    .tlb_flush_wb   (tlb_flush_wb   ),
    .tlb_flush_exe  (tlb_flush_exe  ),
    //from csr
    .flush          (flush          ),
    .csr_interrupt  (csr_interrupt  ),
    //from fs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    //to es
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to fs
    .br_bus         (br_bus         ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   )
);
// EXE stage
exe_stage u_exe_stage(
    .clk            (aclk            ),
    .reset          (reset          ),
    //allowin
    .ms_allowin     (ms_allowin     ),
    .es_allowin     (es_allowin     ),
    //from ds
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //from wb
    .ws_exception   (ws_exception   ),
    //from csr
    .flush          (flush          ),
    .tlbehi_vppn    (tlbehi_vppn    ),
    .crmd_plv       (crmd_plv       ),
    .crmd_da        (crmd_da        ),
    .crmd_pg        (crmd_pg        ),
    .dmw0_plv0      (dmw0_plv0      ),
    .dmw0_plv3      (dmw0_plv3      ),
    .dmw0_pseg      (dmw0_pseg      ),
    .dmw0_vseg      (dmw0_vseg      ),
    .dmw1_plv0      (dmw1_plv1      ),
    .dmw1_plv3      (dmw1_plv3      ),
    .dmw1_pseg      (dmw1_pseg      ),
    .dmw1_vseg      (dmw1_vseg      ),
    .asid_asid      (asid_asid      ),
    //from ms 
    .ms_exception   (ms_exception   ),
    //to ms
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ds
    .es_to_ds_fwd   (es_to_ds_fwd   ),
    .tlb_flush_exe  (tlb_flush_exe  ),
    // data sram interface
    .data_sram_req   (data_sram_req ),
    .data_sram_wr    (data_sram_wr  ),
    .data_sram_size  (data_sram_size ),
    .data_sram_wstrb (data_sram_wstrb),
    .data_sram_addr  (data_sram_addr ),
    .data_sram_wdata (data_sram_wdata),
    .data_sram_addr_ok(data_sram_addr_ok),
    //from tlb
    .s1_found         (s1_found         ),
    .s1_index         (s1_index         ),
    .s1_ppn           (s1_ppn           ),
    .s1_ps            (s1_ps            ),
    .s1_plv           (s1_plv           ),
    .s1_mat           (s1_mat           ),
    .s1_d             (s1_d             ),
    .s1_v             (s1_v             ),
    //to tlb
    .s1_vppn          (s1_vppn          ),
    .s1_va_bit12      (s1_va_bit12      ),
    .s1_asid          (s1_asid          ),
    .invtlb_op        (invtlb_op        ),
    .invtlb_valid     (invtlb_valid     )
);
// MEM stage
mem_stage u_mem_stage(
    .clk            (aclk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    .ms_allowin     (ms_allowin     ),
    //from es
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //from csr
    .flush          (flush          ),
    //to es
    .ms_exception   (ms_exception   ),
    //to ds
    .ms_to_ds_fwd   (ms_to_ds_fwd   ),
    //to ws
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //from data-sram
    .data_sram_data_ok(data_sram_data_ok),
    .data_sram_rdata(data_sram_rdata)
);
// WB stage
wb_stage u_wb_stage(
    .clk            (aclk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    //from ms
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //from csr
    .csr_rdata      (csr_rdata      ),
    //to es
    .ws_exception   (ws_exception   ),
    //to ds
    .ws_to_ds_fwd   (ws_to_ds_fwd   ),
    .tlb_flush_wb   (tlb_flush_wb   ),
    //to csr
    .to_csr_bus     (to_csr_bus     ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //trace debug interface
    .debug_wb_pc      (debug_wb_pc      ),
    .debug_wb_rf_we   (debug_wb_rf_we   ),
    .debug_wb_rf_wnum (debug_wb_rf_wnum ),
    .debug_wb_rf_wdata(debug_wb_rf_wdata)
);

CSR u_CSR( 
    .clk             (aclk           ),
    .reset           (reset         ),
    .to_csr_bus      (to_csr_bus    ),
    .csr_rdata       (csr_rdata     ),
    .exception_pc    (exception_pc  ),
    .csr_interrupt   (csr_interrupt ),
    .flush           (flush         ),
    //to fs, es
    .crmd_plv_out    (crmd_plv       ),
    .crmd_da_out     (crmd_da        ),
    .crmd_pg_out     (crmd_pg        ),
    .dmw0_plv0_out   (dmw0_plv0      ),
    .dmw0_plv3_out   (dmw0_plv3      ),
    .dmw0_pseg_out   (dmw0_pseg      ),
    .dmw0_vseg_out   (dmw0_vseg      ),
    .dmw1_plv0_out   (dmw1_plv1      ),
    .dmw1_plv3_out   (dmw1_plv3      ),
    .dmw1_pseg_out   (dmw1_pseg      ),
    .dmw1_vseg_out   (dmw1_vseg      ),
    .asid_asid_out   (asid_asid      ),
    //to es
    .es_tlbehi_vppn  (tlbehi_vppn   ),
    //to tlb
    .we              (we            ),
    .w_index         (w_index       ),
    .w_e             (w_e           ),
    .w_vppn          (w_vppn        ),
    .w_ps            (w_ps          ),
    .w_asid          (w_asid        ),
    .w_g             (w_g           ),
    .w_ppn0          (w_ppn0        ),
    .w_plv0          (w_plv0        ),
    .w_mat0          (w_mat0        ),
    .w_d0            (w_d0          ),
    .w_v0            (w_v0          ),
    .w_ppn1          (w_ppn1        ),
    .w_plv1          (w_plv1        ),
    .w_mat1          (w_mat1        ),
    .w_d1            (w_d1          ),
    .w_v1            (w_v1          ),

    .r_index         (r_index       ),
    .r_e             (r_e           ),
    .r_vppn          (r_vppn        ),
    .r_ps            (r_ps          ),
    .r_asid          (r_asid        ),
    .r_g             (r_g           ),
    .r_ppn0          (r_ppn0        ),
    .r_plv0          (r_plv0        ),
    .r_mat0          (r_mat0        ),
    .r_d0            (r_d0          ),
    .r_v0            (r_v0          ),
    .r_ppn1          (r_ppn1        ),
    .r_plv1          (r_plv1        ),
    .r_mat1          (r_mat1        ),
    .r_d1            (r_d1          ),
    .r_v1            (r_v1          )
);

sram_to_axi_bridge u_bridge(
    .aclk            (aclk           ),
    .aresetn         (reset          ),
    
    .inst_sram_req   (inst_sram_req  ),
    .inst_sram_wr    (inst_sram_wr   ),
    .inst_sram_size  (inst_sram_size ),
    .inst_sram_wstrb (inst_sram_wstrb),
    .inst_sram_addr  (inst_sram_addr ),
    .inst_sram_wdata (inst_sram_wdata),
    .inst_sram_addr_ok(inst_sram_addr_ok),
    .inst_sram_data_ok(inst_sram_data_ok),
    .inst_sram_rdata (inst_sram_rdata),

    .data_sram_req   (data_sram_req  ),
    .data_sram_wr    (data_sram_wr   ),
    .data_sram_size  (data_sram_size ),
    .data_sram_wstrb (data_sram_wstrb),
    .data_sram_addr  (data_sram_addr ),
    .data_sram_wdata (data_sram_wdata),
    .data_sram_addr_ok(data_sram_addr_ok),
    .data_sram_data_ok(data_sram_data_ok),
    .data_sram_rdata (data_sram_rdata),
    //read request interface
    .arid            (arid          ),
    .araddr          (araddr        ),
    .arlen           (arlen         ),
    .arsize          (arsize        ),
    .arburst         (arburst       ),
    .arlock          (arlock        ),
    .arcache         (arcache       ),
    .arprot          (arprot        ),
    .arvalid         (arvalid       ),
    .arready         (arready       ),
    //read response interface
    .rid             (rid           ),
    .rdata           (rdata         ),
    .rresp           (rresp         ),
    .rlast           (rlast         ),
    .rvalid          (rvalid        ),
    .rready          (rready        ),
    //write request interface
    .awid            (awid          ),
    .awaddr          (awaddr        ),
    .awlen           (awlen         ),
    .awsize          (awsize        ),
    .awburst         (awburst       ),
    .awlock          (awlock        ),
    .awcache         (awcache       ),
    .awprot          (awprot        ),
    .awvalid         (awvalid       ),
    .awready         (awready       ),
    //write data interface
    .wid             (wid           ),
    .wdata           (wdata         ),
    .wstrb           (wstrb         ),
    .wlast           (wlast         ),
    .wvalid          (wvalid        ),
    .wready          (wready        ),
    //write response interface
    .bid             (bid           ),
    .bresp           (bresp         ),
    .bvalid          (bvalid        ),
    .bready          (bready        )
);

tlb u_tlb(
    .clk             (aclk          ),

    .s0_vppn         (s0_vppn       ),
    .s0_va_bit12     (s0_va_bit12   ),
    .s0_asid         (s0_asid       ),
    .s0_found        (s0_found      ),
    .s0_index        (s0_index      ),
    .s0_ppn          (s0_ppn        ),
    .s0_ps           (s0_ps         ),
    .s0_plv          (s0_plv        ),
    .s0_mat          (s0_mat        ),
    .s0_d            (s0_d          ),
    .s0_v            (s0_v          ),

    .s1_vppn         (s1_vppn       ),
    .s1_va_bit12     (s1_va_bit12   ),
    .s1_asid         (s1_asid       ),
    .s1_found        (s1_found      ),
    .s1_index        (s1_index      ),
    .s1_ppn          (s1_ppn        ),
    .s1_ps           (s1_ps         ),
    .s1_plv          (s1_plv        ),
    .s1_mat          (s1_mat        ),
    .s1_d            (s1_d          ),
    .s1_v            (s1_v          ),

    .invtlb_valid    (invtlb_valid  ),
    .invtlb_op       (invtlb_op     ),

    .we              (we            ),
    .w_index         (w_index       ),
    .w_e             (w_e           ),
    .w_vppn          (w_vppn        ),
    .w_ps            (w_ps          ),
    .w_asid          (w_asid        ),
    .w_g             (w_g           ),
    .w_ppn0          (w_ppn0        ),
    .w_plv0          (w_plv0        ),
    .w_mat0          (w_mat0        ),
    .w_d0            (w_d0          ),
    .w_v0            (w_v0          ),
    .w_ppn1          (w_ppn1        ),
    .w_plv1          (w_plv1        ),
    .w_mat1          (w_mat1        ),
    .w_d1            (w_d1          ),
    .w_v1            (w_v1          ),

    .r_index         (r_index       ),
    .r_e             (r_e           ),
    .r_vppn          (r_vppn        ),
    .r_ps            (r_ps          ),
    .r_asid          (r_asid        ),
    .r_g             (r_g           ),
    .r_ppn0          (r_ppn0        ),
    .r_plv0          (r_plv0        ),
    .r_mat0          (r_mat0        ),
    .r_d0            (r_d0          ),
    .r_v0            (r_v0          ),
    .r_ppn1          (r_ppn1        ),
    .r_plv1          (r_plv1        ),
    .r_mat1          (r_mat1        ),
    .r_d1            (r_d1          ),
    .r_v1            (r_v1          )
);
endmodule
