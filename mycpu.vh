`ifndef MYCPU_H
    `define MYCPU_H
    `define TLBNUM          16
    `define BR_BUS_WD       34
    `define FS_TO_DS_BUS_WD 78
    `define DS_TO_ES_BUS_WD 275
    `define ES_TO_MS_BUS_WD 209
    `define MS_TO_WS_BUS_WD 203
    `define WS_TO_RF_BUS_WD 39
    `define ES_TO_DS_FWD    40
    `define MS_TO_DS_FWD    39
    `define WS_TO_DS_FWD    39
    `define TO_CSR_BUS      162
    `define EXCEPTION_CODE  14
    /*---------CSR寄存器编号(地址)--------- */
    `define CRMD   14'h0
    `define PRMD   14'h1
    `define EUEN   14'h2
    `define ECFG   14'h4 
    `define ESTAT  14'h5
    `define ERA    14'h6
    `define BADV   14'h7
    `define EENTRY 14'hc
    `define TLBIDX 14'h10
    `define TLBEHI 14'h11
    `define TLBELO0 14'h12
    `define TLBELO1 14'h13
    `define ASID   14'h18
    `define PGDL   14'h19
    `define PGDH   14'h1a
    `define PGD    14'h1b
    `define CPUID  14'h20
    `define SAVE0  14'h30
    `define SAVE1  14'h31
    `define SAVE2  14'h32
    `define SAVE3  14'h33
    `define TID    14'h40
    `define TCFG   14'h41
    `define TVAL   14'h42
    `define TICLR  14'h44
    `define TLBRENTRY 14'h88
    `define CTAG   14'h98
    `define DMW0   14'h180
    `define DMW1   14'h181

    /* ---------各种异常一级编码---------- */
    `define ECODE_INT     6'h00 // 中断例外              0
    `define ECODE_PIL     6'h01 // load 操作页无效例外   1
    `define ECODE_PIS     6'h02 // store 操作页无效例外  2
    `define ECODE_PIF     6'h03 // 取指操作页无效例外    3
    `define ECODE_PME     6'h04 // 页修改例外            4
    `define ECODE_PPI     6'h07 // 页特权等级不合规例外  5(exe),6(if)
    `define ECODE_ADE     6'h08 // 取指地址错例外        7    
    `define ECODE_ALE     6'h09 // 地址非对齐例外        8
    `define ECODE_SYS     6'h0b // 系统调用例外          9
    `define ECODE_BRK     6'h0c // 断点例外              10
    `define ECODE_INE     6'h0d // 指令不存在例外        11
    `define ECODE_TLBR    6'h3f // TLB重填例外           12(exe),13(if)
    /*--------各种异常二级编码--------- */
    `define ESUBCODE_ADEF 9'h0  // 取指地址错例外   7
    `define ESUBCODE_ADEM 9'h1  // 访存地址错例外   7
`endif
