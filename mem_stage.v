`include "mycpu.vh"

module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //from csr
    input                          flush         ,
    //to es
    output                         ms_exception  ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //to ds
    output [`MS_TO_DS_FWD-1    :0] ms_to_ds_fwd  ,
    //from data-sram
    input                          data_sram_data_ok,
    input  [31                 :0] data_sram_rdata
);
wire    inst_ld_b  ;
wire    inst_ld_h  ;
wire    inst_ld_bu ;
wire    inst_ld_hu ;
wire    inst_ld_w  ;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
reg         ms_valid;
wire        ms_ready_go;
wire [4:0]  ms_to_ds_dest;
wire [31:0] ms_to_ds_result;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
wire [31:0] mem_result;
wire [31:0] ms_final_result;

wire        inst_csr;
wire        ms_csr_we;
wire        ms_ertn_flush;
wire [13:0] ms_csr_num;
wire [31:0] ms_csr_mask, ms_csr_wdata;
wire [`EXCEPTION_CODE-1:0] exception_code_in;
wire [`EXCEPTION_CODE-1:0] exception_code_out;
wire [31:0] data_vaddr;
wire        inst_rdcntid_w;
wire        inst_rdcntvh_w;
wire        inst_rdcntvl_w;
wire        ms_inst_rdtime;
wire        data_sram_req_reg;
wire        inst_tlbwr;
wire        inst_tlbrd;
wire        inst_tlbfill;

//Forward
assign ms_to_ds_fwd = { ms_inst_rdtime,
                        inst_csr,
                        ms_to_ds_dest,
                        ms_to_ds_result
                        };
assign ms_inst_rdtime =  inst_rdcntid_w;
//数据接收
assign ms_ready_go    = ms_valid && (!data_sram_req_reg || data_sram_data_ok);
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin || flush;
assign ms_to_ws_valid = ms_valid && ms_ready_go && ~flush;
always @(posedge clk) begin
    if (reset) begin
        ms_valid <= 1'b0;
    end
    else if (ms_allowin) begin
        ms_valid <= es_to_ms_valid;
    end
end
always @(posedge clk ) begin
    if(reset)es_to_ms_bus_r <= `ES_TO_MS_BUS_WD'b0;
    else if (es_to_ms_valid && ms_allowin) begin
        es_to_ms_bus_r  <= es_to_ms_bus;
    end
end
assign {inst_tlbrd,
        inst_tlbwr,
        inst_tlbfill,
        data_sram_req_reg,
        inst_rdcntvh_w,
        inst_rdcntvl_w,
        inst_rdcntid_w,
        data_vaddr ,//201:170
        inst_csr,       //1 169:169
        ms_csr_we      ,//1
        ms_csr_num     ,//14
        ms_csr_mask    ,//32
        ms_csr_wdata   ,//32
        ms_ertn_flush  ,//1
        exception_code_in ,//14 75:88
        inst_ld_b    ,      //74:74
        inst_ld_h    ,
        inst_ld_bu   ,
        inst_ld_hu   ,
        ms_res_from_mem,  //70:70
        ms_gr_we       ,  //69:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

assign ms_to_ws_bus = { inst_tlbrd,//202:202
                        inst_tlbwr,
                        inst_tlbfill,
                        inst_rdcntvh_w,
                        inst_rdcntvl_w,
                        inst_rdcntid_w,
                        data_vaddr, // 196:165
                        inst_csr,       //1
                        ms_csr_we      ,//1
                        ms_csr_num     ,//14
                        ms_csr_mask    ,//32
                        ms_csr_wdata   ,//32
                        ms_ertn_flush  ,//1
                        exception_code_out ,//14
                        ms_gr_we       ,  //69:69
                        ms_dest        ,  //68:64
                        ms_final_result,  //63:32
                        ms_pc             //31:0
                      };

//内存数据读
wire [7:0] data_rdata_byte ;
wire [15:0] data_rdata_halfword;
assign data_rdata_byte = (ms_alu_result[1:0]==2'b00) ? data_sram_rdata[7:0] :
                         (ms_alu_result[1:0]==2'b01) ? data_sram_rdata[15:8] :
                         (ms_alu_result[1:0]==2'b10) ? data_sram_rdata[23:16] :
                         data_sram_rdata[31:24] ;
                
assign data_rdata_halfword = (ms_alu_result[1]==1'b0) ? data_sram_rdata[15:0] :
                             data_sram_rdata[31:16] ;

assign mem_result      = inst_ld_b ? {{24{data_rdata_byte[7]}},data_rdata_byte}           :
                         inst_ld_h ? {{16{data_rdata_halfword[15]}},data_rdata_halfword}  :
                         inst_ld_bu? {{24{1'b0}},data_rdata_byte}                         :
                         inst_ld_hu? {{16{1'b0}},data_rdata_halfword}                     :
                         data_sram_rdata;

assign ms_to_ds_dest  = ms_dest & {5{ms_valid && ms_gr_we}};
assign ms_final_result = ms_res_from_mem ? mem_result : ms_alu_result;
assign ms_to_ds_result = ms_final_result;

//异常处理
assign exception_code_out = exception_code_in;
assign ms_exception = ((exception_code_out!=`EXCEPTION_CODE'h0) | ms_ertn_flush) && ms_valid;

endmodule
