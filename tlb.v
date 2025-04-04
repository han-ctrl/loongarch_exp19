`timescale 1ns / 1ps

module tlb
#(
    parameter TLBNUM = 16
)
(
    input  wire                      clk,

    // search port 0 (for fetch)
    input  wire [              18:0] s0_vppn,//虚拟地址31...13位
    input  wire                      s0_va_bit12,//虚拟地址12位
    input  wire [               9:0] s0_asid,//地址空间标识符
    output wire                      s0_found,
    output wire [$clog2(TLBNUM)-1:0] s0_index,//支持tlbsrch指令，记录命中在第几项于TLBIDX.NE
    output wire [              19:0] s0_ppn,
    output wire [               5:0] s0_ps,
    output wire [               1:0] s0_plv,
    output wire [               1:0] s0_mat,
    output wire                      s0_d,
    output wire                      s0_v,

    // search port 1 (for load/store)
    input  wire [              18:0] s1_vppn,
    input  wire                      s1_va_bit12,
    input  wire [               9:0] s1_asid,
    output wire                      s1_found,
    output wire [$clog2(TLBNUM)-1:0] s1_index,
    output wire [              19:0] s1_ppn,
    output wire [               5:0] s1_ps,
    output wire [               1:0] s1_plv,
    output wire [               1:0] s1_mat,
    output wire                      s1_d,
    output wire                      s1_v,

    // invtlb opcode
    input  wire                      invtlb_valid,
    input  wire [               4:0] invtlb_op,

    // write port 
    input  wire                      we,     //w(rite) e(nable)
    input  wire [$clog2(TLBNUM)-1:0] w_index,
    input  wire                      w_e,//存在位，为1表示可以参与查找
    input  wire [              18:0] w_vppn,
    input  wire [               5:0] w_ps,
    input  wire [               9:0] w_asid,
    input  wire                      w_g,
    input  wire [              19:0] w_ppn0,
    input  wire [               1:0] w_plv0,
    input  wire [               1:0] w_mat0,
    input  wire                      w_d0,
    input  wire                      w_v0,
    input  wire [              19:0] w_ppn1,
    input  wire [               1:0] w_plv1,
    input  wire [               1:0] w_mat1,
    input  wire                      w_d1,
    input  wire                      w_v1,

    // read port
    input  wire [$clog2(TLBNUM)-1:0] r_index,
    output wire                      r_e,
    output wire [              18:0] r_vppn,
    output wire [               5:0] r_ps,
    output wire [               9:0] r_asid,
    output wire                      r_g,
    output wire [              19:0] r_ppn0,
    output wire [               1:0] r_plv0,
    output wire [               1:0] r_mat0,
    output wire                      r_d0,
    output wire                      r_v0,
    output wire [              19:0] r_ppn1,
    output wire [               1:0] r_plv1,
    output wire [               1:0] r_mat1,
    output wire                      r_d1,
    output wire                      r_v1
);
//TLB regfile
reg  [TLBNUM-1:0] tlb_e;
reg  [TLBNUM-1:0] tlb_ps4MB; //pagesize 1:4MB, 0:4KB
reg  [      18:0] tlb_vppn     [TLBNUM-1:0];
reg  [       9:0] tlb_asid     [TLBNUM-1:0];
reg               tlb_g        [TLBNUM-1:0];

reg  [      19:0] tlb_ppn0     [TLBNUM-1:0];
reg  [       1:0] tlb_plv0     [TLBNUM-1:0];
reg  [       1:0] tlb_mat0     [TLBNUM-1:0];
reg               tlb_d0       [TLBNUM-1:0];
reg               tlb_v0       [TLBNUM-1:0];
reg  [      19:0] tlb_ppn1     [TLBNUM-1:0];
reg  [       1:0] tlb_plv1     [TLBNUM-1:0];
reg  [       1:0] tlb_mat1     [TLBNUM-1:0];
reg               tlb_d1       [TLBNUM-1:0];
reg               tlb_v1       [TLBNUM-1:0];

//search
wire [TLBNUM-1:0] match0;
wire [TLBNUM-1:0] match1;
wire [TLBNUM-1:0] invtlb_match;
wire [$clog2(TLBNUM)-1:0] s0_index_arr[TLBNUM-1:0];//$clog2(TLBNUM)为TLBNUM取以2为底的对数，再向上取整
wire [$clog2(TLBNUM)-1:0] s1_index_arr[TLBNUM-1:0];
wire              s0_sel;
wire              s1_sel;

assign s0_found = |match0;
assign s1_found = |match1;
assign s0_index = s0_index_arr[TLBNUM-1];
assign s1_index = s1_index_arr[TLBNUM-1];
assign s0_ps    = tlb_ps4MB[s0_index] ? 6'd21 : 6'd12;
assign s1_ps    = tlb_ps4MB[s1_index] ? 6'd21 : 6'd12;
assign s0_sel   = tlb_ps4MB[s0_index] ? s0_vppn[8] : s0_va_bit12;
assign s1_sel   = tlb_ps4MB[s1_index] ? s1_vppn[8] : s1_va_bit12;

assign s0_ppn   = s0_sel ? tlb_ppn1[s0_index] : tlb_ppn0[s0_index];
assign s1_ppn   = s1_sel ? tlb_ppn1[s1_index] : tlb_ppn0[s1_index];
assign s0_mat   = s0_sel ? tlb_mat1[s0_index] : tlb_mat0[s0_index];
assign s1_mat   = s1_sel ? tlb_mat1[s1_index] : tlb_mat0[s1_index];
assign s0_plv   = s0_sel ? tlb_plv1[s0_index] : tlb_plv0[s0_index];
assign s1_plv   = s1_sel ? tlb_plv1[s1_index] : tlb_plv0[s1_index];
assign s0_d     = s0_sel ? tlb_d1[s0_index] : tlb_d0[s0_index];
assign s1_d     = s1_sel ? tlb_d1[s1_index] : tlb_d0[s1_index];
assign s0_v     = s0_sel ? tlb_v1[s0_index] : tlb_v0[s0_index];
assign s1_v     = s1_sel ? tlb_v1[s1_index] : tlb_v0[s1_index];

assign s0_index_arr[0] = {$clog2(TLBNUM){1'b0}};
assign s1_index_arr[0] = {$clog2(TLBNUM){1'b0}}; 
genvar i;
generate 
    for(i=0;i<TLBNUM;i=i+1)begin: search_tlb
        assign match0[i] = (s0_vppn[18:9]==tlb_vppn[i][18:9]) &&
                        (tlb_ps4MB[i] || s0_vppn[8:0]==tlb_vppn[i][8:0]) &&
                        (tlb_g[i] || s0_asid==tlb_asid[i]) &&
                        tlb_e[i];
        assign match1[i] = (s1_vppn[18:9]==tlb_vppn[i][18:9]) &&
                        (tlb_ps4MB[i] || s1_vppn[8:0]==tlb_vppn[i][8:0]) &&
                        (tlb_g[i] || s1_asid==tlb_asid[i]) &&
                        tlb_e[i];
        if(i!=0)begin
            assign s0_index_arr[i] = s0_index_arr[i-1] | ({$clog2(TLBNUM){match0[i]}} & i);
            assign s1_index_arr[i] = s1_index_arr[i-1] | ({$clog2(TLBNUM){match1[i]}} & i);
        end
        assign invtlb_match[i] = (invtlb_op==5'b0 || invtlb_op==5'b1) 
                                || invtlb_op==5'b10 && tlb_g[i] 
                                || invtlb_op==5'b11 && !tlb_g[i]
                                || invtlb_op==5'b100 && !tlb_g[i] && s1_asid==tlb_asid[i] 
                                || invtlb_op==5'b101 && !tlb_g[i] && s1_asid==tlb_asid[i] && s1_vppn[18:9]==tlb_vppn[i][18:9] && (tlb_ps4MB[i] || s1_vppn[8:0]==tlb_vppn[i][8:0])
                                || invtlb_op==5'b110 && (tlb_g[i] || s1_asid==tlb_asid[i]) &&  s1_vppn[18:9]==tlb_vppn[i][18:9] && (tlb_ps4MB[i] || s1_vppn[8:0]==tlb_vppn[i][8:0]);
        always @(posedge clk ) begin
            if(invtlb_valid && invtlb_match[i])tlb_e[i] <= 1'b0;
        end                     
    end
endgenerate

//write
always @(posedge clk ) begin
    if(we)begin
        tlb_e[w_index]     <= w_e;
        tlb_vppn[w_index]  <= w_vppn;
        tlb_ps4MB[w_index] <= (w_ps == 6'd21);
        tlb_asid[w_index]  <= w_asid;    
        tlb_g[w_index]     <= w_g;        
            
        tlb_ppn0[w_index] <= w_ppn0;
        tlb_plv0[w_index] <= w_plv0;
        tlb_mat0[w_index] <= w_mat0;
        tlb_d0[w_index]   <= w_d0;
        tlb_v0[w_index]   <= w_v0;
        tlb_ppn1[w_index] <= w_ppn1;
        tlb_plv1[w_index] <= w_plv1;
        tlb_mat1[w_index] <= w_mat1;
        tlb_d1[w_index]   <= w_d1;
        tlb_v1[w_index]   <= w_v1;
    end
end

//read
assign r_e    = tlb_e[r_index];
assign r_vppn = tlb_vppn[r_index];
assign r_ps   = tlb_ps4MB[r_index] ? 6'd21 : 6'd12;
assign r_asid = tlb_asid[r_index];
assign r_g    = tlb_g[r_index];

assign r_ppn0 = tlb_ppn0[r_index];
assign r_plv0 = tlb_plv0[r_index];
assign r_mat0 = tlb_mat0[r_index];
assign r_d0   = tlb_d0[r_index];
assign r_v0   = tlb_v0[r_index];
assign r_ppn1 = tlb_ppn1[r_index];
assign r_plv1 = tlb_plv1[r_index];
assign r_mat1 = tlb_mat1[r_index];
assign r_d1   = tlb_d1[r_index];
assign r_v1   = tlb_v1[r_index];
endmodule
