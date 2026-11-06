#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stdint.h>
#define TK_SUPPORT_SMP 1
#define EXPORT
#define IMPORT extern
#define CONST const
#ifndef APP_TEST
#define APP_TEST 0
#endif
#define TP_PRC1 1u
#define TA_HLNG 1u
#define TA_INHERIT 2u
#define TA_ASSPRC 0x10000u
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define XOSC_MHz 12
#define E_SYS (-5)
#define E_OK 0
#define E_PAR (-17)
#define E_CTX (-25)
#define E_LIMIT (-34)
#define E_OBJ (-41)
typedef int ID, ER, INT, PRI, TMO;
typedef unsigned int UINT, UW, RELTIM;
typedef unsigned char UB;
typedef void (*FP)();
typedef struct { int32_t hi; uint32_t lo; } SYSTIM;
ER tk_get_otm(SYSTIM *time);
ER tk_dly_tsk(RELTIM timeout);
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
#define TA_USERBUF 0x20u
#define TA_RNG3 0x300u
#define TA_TFIFO 0u
#define TA_WMUL 8u
#define TMO_POL 0
#define TWF_ORW 1u
#define TWF_BITCLR 0x20u
#define E_TMOUT (-50)
typedef int SZ;
typedef struct { FP task; PRI itskpri; SZ stksz; UW tskatr; void *bufptr; UW assprc; } T_CTSK;
typedef struct { UW mbfatr; SZ bufsz; INT maxmsz; void *bufptr; } T_CMBF;
typedef struct { UW flgatr; UINT iflgptn; } T_CFLG;
ID tk_cre_tsk(const T_CTSK *c);
ER tk_sta_tsk(ID id, INT code);
ER tk_del_tsk(ID id);
ID tk_get_tid(void);
void tk_ext_tsk(void);
ID tk_cre_mbf(const T_CMBF *c);
ER tk_snd_mbf(ID id, const void *msg, INT size, TMO timeout);
INT tk_rcv_mbf(ID id, void *msg, TMO timeout);
ID tk_cre_flg(const T_CFLG *c);
ER tk_set_flg(ID id, UINT bits);
ER tk_clr_flg(ID id, UINT mask);
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *result, TMO timeout);
#endif
