#ifndef PIL_FAKE_TKERNEL_H
#define PIL_FAKE_TKERNEL_H
#include <stdint.h>
typedef int32_t INT, ID, ER, SZ;
typedef uint32_t UINT, UW;
typedef unsigned char UB;
typedef void (*FP)(void);
typedef struct {
    FP task; INT itskpri; UW tskatr; SZ stksz; void *bufptr; UW assprc;
} T_CTSK;
typedef struct { UW intatr; FP inthdr; } T_DINT;
#define TA_HLNG 1u
#define TA_RNG0 0u
#define TA_USERBUF 2u
#define TA_ASSPRC 4u
#define TP_PRC1 1u
#define E_OK 0
#define CLK_PERI_FREQ 125000000u
ID tk_cre_tsk(const T_CTSK *c);
ER tk_sta_tsk(ID id, INT code);
ER tk_del_tsk(ID id);
ER tk_wup_tsk(ID id);
ER tk_slp_tsk(INT delay);
ER tk_def_int(UINT n, const T_DINT *d);
void DisableInt(UINT n);
void EnableInt(UINT n, UINT priority);
void ClearInt(UINT n);
#endif
