#include "tkernel.h"
#define DEFAULT_SYS_STKSZ 128
void DisableInt(UINT irq);
void ClearInt(UINT irq);
void EnableInt(UINT irq, INT level);
