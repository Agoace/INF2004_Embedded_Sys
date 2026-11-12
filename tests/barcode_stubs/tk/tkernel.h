#ifndef BARCODE_KERNEL_STUB_H
#define BARCODE_KERNEL_STUB_H
#include "../../mtk_stubs/tk/tkernel.h"
#undef TK_SUPPORT_SMP
#define TK_SUPPORT_SMP 0
ER tk_stp_cyc(ID id);
ER tk_del_cyc(ID id);
#endif
