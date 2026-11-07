#ifndef TEMPLATE_TEST_TKERNEL_H
#define TEMPLATE_TEST_TKERNEL_H
#include <stdint.h>
#ifndef TK_SUPPORT_SMP
#define TK_SUPPORT_SMP 0
#endif
#define USE_OBJECT_NAME 1
#define EXPORT
#define IMPORT extern
#define E_OK 0
#define E_SYS (-5)
#define E_PAR (-17)
#define E_OBJ (-41)
#define TA_HLNG 1u
#define TA_RNG1 0x100u
#define TA_ASSPRC 0x10000u
#define TP_PRC1 1u
#define TRUE 1
#define FALSE 0
typedef int ID,ER,INT,PRI,BOOL;
typedef uint32_t UW,UINT;
typedef unsigned char UB;
typedef void (*FP)();
typedef struct {
    UW tskatr;FP task;PRI itskpri;int stksz;UW assprc;UB *dsname;
} T_CTSK;
ID tk_cre_tsk(const T_CTSK *value);
ER tk_sta_tsk(ID id,INT code);
ER tk_del_tsk(ID id);
void template_mock_di(UINT *irq);
void template_mock_ei(UINT irq);
#define DI(irq) template_mock_di(&(irq))
#define EI(irq) template_mock_ei(irq)
#endif
