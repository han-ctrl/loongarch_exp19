

`include "mycpu.vh"

module wb_stage(
    input  wire                         clk           ,
    input  wire                         reset         ,
    //allowin
    output wire                         ws_allowin    ,
    //from ms
    input  wire                         ms_to_ws_valid,
    input  wire [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //from csr
    input wire  [31:0]                  csr_rdata     ,
    //to es
    output wire                         ws_exception  ,
    //to rf: for write back
    output wire [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus  ,
    //to ds
    output wire [`WS_TO_DS_FWD-1    :0] ws_to_ds_fwd  ,
    output wire                         tlb_flush_wb  ,
    //to csr
    output wire [`TO_CSR_BUS  -1    :0] to_csr_bus    ,
    //trace debug interface
    output wire [31:0] debug_wb_pc     ,
    output wire [ 3:0] debug_wb_rf_we  ,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
wire ws_csr_we, ws_ertn_flush;
wire [13:0] ws_csr_num_in, ws_csr_num_out;
wire [31:0] ws_csr_mask, ws_csr_wdata;
wire [`EXCEPTION_CODE-1:0] exception_code;
wire [31:0] data_vaddr;

wire [4:0]  ws_to_ds_dest;
wire [31:0] ws_to_ds_result;
reg         ws_valid;
wire        ws_ready_go;
reg [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus_r;
wire        ws_gr_we;
wire [ 4:0] ws_dest;
wire [31:0] ws_final_result;
wire [31:0] ws_pc;
wire        inst_csr;
wire        ws_flush;
wire        inst_rdcntid_w;
wire        inst_rdcntvh_w;
wire        inst_rdcntvl_w;
wire        ws_inst_rdtime;
wire        inst_tlbwr;
wire        inst_tlbrd;
wire        inst_tlbfill;

//tlb
assign tlb_flush_wb = inst_tlbfill | inst_tlbwr | inst_tlbrd;

//Forward
assign ws_to_ds_dest = ws_dest & {5{ws_valid && ws_gr_we}};
assign ws_to_ds_fwd = { ws_inst_rdtime,
                        inst_csr,
                        ws_to_ds_dest ,
                        ws_to_ds_result
                        } ;
assign ws_inst_rdtime = inst_rdcntid_w;
//写回通用寄存器
wire        rf_we;
wire [4 :0] rf_waddr;
wire [31:0] rf_wdata;
assign ws_to_ds_result = rf_wdata;
assign ws_to_rf_bus = {rf_we   ,  //37:37
                       rf_waddr,  //36:32
                       rf_wdata   //31:0
                      };
assign rf_we    = ws_gr_we && ws_valid && ~ws_flush;
assign rf_waddr = ws_dest;
assign rf_wdata = (inst_csr|inst_rdcntid_w) ? csr_rdata : ws_final_result;

//异常处理
assign ws_csr_num_out = ws_csr_num_in & {14{!inst_rdcntid_w}} | `TID & {14{inst_rdcntid_w}};
assign ws_exception = (|exception_code | ws_ertn_flush )& ws_valid;
assign ws_flush = ws_exception;
assign to_csr_bus ={inst_tlbrd,//160:160
                    inst_tlbwr,
                    inst_tlbfill,
                    data_vaddr, //157:126
                    ws_csr_we, 
                    ws_csr_num_out,
                    ws_csr_mask,
                    ws_csr_wdata,
                    ws_ertn_flush&ws_valid,
                    |exception_code &&ws_valid,
                    exception_code,//44:32
                    ws_pc};  //31:0

//接收数据
assign ws_ready_go = 1'b1;
assign ws_allowin  = !ws_valid || ws_ready_go || ws_flush;
always @(posedge clk) begin
    if (reset) begin
        ws_valid <= 1'b0;
    end
    else if (ws_allowin) begin
        ws_valid <= ms_to_ws_valid;
    end
end
always @(posedge clk ) begin
    if(reset) ms_to_ws_bus_r <= `MS_TO_WS_BUS_WD'b0;
    else if (ms_to_ws_valid & ws_allowin) begin
        ms_to_ws_bus_r <= ms_to_ws_bus;
    end    
end
assign {inst_tlbrd,
        inst_tlbwr,
        inst_tlbfill,
        inst_rdcntvh_w,
        inst_rdcntvl_w,
        inst_rdcntid_w,
        data_vaddr, //  
        inst_csr,       //1 164:164
        ws_csr_we      ,//1
        ws_csr_num_in  ,//14
        ws_csr_mask    ,//32
        ws_csr_wdata   ,//32
        ws_ertn_flush     ,//1
        exception_code ,//14 
        ws_gr_we       ,  //69:69
        ws_dest        ,  //68:64
        ws_final_result,  //63:32
        ws_pc             //31:0
       } = ms_to_ws_bus_r;

// debug info generate
assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_we    = {4{rf_we}};
assign debug_wb_rf_wnum  = rf_waddr;
assign debug_wb_rf_wdata = rf_wdata;

endmodule

