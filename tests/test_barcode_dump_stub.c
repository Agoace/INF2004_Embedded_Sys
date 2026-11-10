#include "../app_program/subsystem3/barcode_live_source.h"
extern unsigned int source_dumps;
/* Link as a separate object, as S3 does in firmware. */
void barcode_live_source_dump(void)
{
    ++source_dumps;
}
