#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stdint.h>
#define TK_SUPPORT_SMP 1
#define EXPORT
#define IMPORT extern
#define CONST const
#define APP_TEST 0
#define TP_PRC1 1u
#define TA_HLNG 1u
#define TA_INHERIT 2u
#define TA_ASSPRC 0x10000u
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define E_OK 0
#define E_PAR (-17)
#define E_CTX (-25)
#define E_LIMIT (-34)
#define E_OBJ (-41)
typedef int ID, ER, INT, PRI, TMO;
typedef unsigned int UINT, UW;
typedef unsigned char UB;
typedef void (*FP)();
typedef struct { unsigned int mtxatr; } T_CMTX;
typedef struct { unsigned int cycatr; FP cychdr; unsigned int cyctim, cycphs, assprc; } T_CCYC;
typedef struct { unsigned int intatr; FP inthdr; } T_DINT;
ID tk_cre_mtx(const T_CMTX *m);
ER tk_loc_mtx(ID id, TMO timeout);
ER tk_unl_mtx(ID id);
ER tk_del_mtx(ID id);
ID tk_cre_cyc(const T_CCYC *c);
ER tk_sta_cyc(ID id);
ER tk_slp_tsk(TMO timeout);
INT tk_can_wup(ID id);
ER tk_wup_tsk(ID id);
INT tk_get_prc(void);
ER tk_def_int(UINT irq, const T_DINT *d);
void mock_di(UINT *irq);
void mock_ei(UINT irq);
#define DI(irq) mock_di(&(irq))
#define EI(irq) mock_ei(irq)
#endif
