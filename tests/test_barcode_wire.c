#include <assert.h>
#include <string.h>
#include "../app_program/subsystem1/mqtt_wire.h"
int main(void)
{
    rm_barcode_t v={.flags=RM_BARCODE_DECODED|RM_BARCODE_SHADOW|RM_BARCODE_WIDTHS,
        .letter='F',.reason=1u,.direction=2u,.command=0u,
        .sequence=0x04030201u,.epoch=0x08070605u,.start_ms=0x0c0b0a09u,
        .uptime_ms=0x100f0e0du,.narrow_cmm=445u,.wide_cmm=1335u};
    uint8_t bytes[RM_BARCODE_SIZE];
    const uint8_t expected[]={2,1,13,'F',1,0,2,0,1,2,3,4,5,6,7,8,
        9,10,11,12,13,14,15,16,0xbd,1,0x37,5};
    assert(rm_pack_barcode(bytes,&v));
    assert(sizeof(expected)==RM_BARCODE_SIZE && memcmp(bytes,expected,sizeof(bytes))==0);
    v.flags=RM_BARCODE_FAILED|RM_BARCODE_SHADOW;v.letter=0;v.reason=101;
    v.direction=0;v.narrow_cmm=v.wide_cmm=RM_NO_DATA_U16;
    assert(rm_pack_barcode(bytes,&v));
    assert(bytes[2]==6 && bytes[4]==101 && bytes[24]==255 && bytes[27]==255);
    uint8_t before[RM_BARCODE_SIZE];memcpy(before,bytes,sizeof(bytes));
    v.letter='F';assert(!rm_pack_barcode(bytes,&v));
    assert(memcmp(bytes,before,sizeof(bytes))==0);
    v.letter=0;v.flags|=RM_BARCODE_DECODED;assert(!rm_pack_barcode(bytes,&v));
    assert(!rm_pack_barcode(NULL,&v));assert(!rm_pack_barcode(bytes,NULL));
    return 0;
}
