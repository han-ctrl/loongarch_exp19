# loongarch_exp19  
## 问题一  
assign inst_pa = {32{direct_pa_valid}} & direct_pa |  
                 {32{dmw0_pa_valid}} & dmw0_pa     |  
                 {32{dmw1_pa_valid}} & dmw1_pa     |  
                 {32{tlb_pa_valid}} & tlb_pa       ;  
              crmd_pg, crmd_da = 2'b01时，直接使用虚地址；  
              =2‘b10时，若高三位虚地址等于dmw_vseg，且plv符合则采用dmw_pseg, 虚地址低29位拼接成物理地址；  
              若都不是则采用tlb地址
