/* Gate1 only. Outside both startup copy/zero ranges, before heap start. */
SECTIONS
{
  .pil_retained (NOLOAD) : ALIGN(4)
  {
    __pil_retained_start = .;
    KEEP(*(.pil_retained))
    __pil_retained_end = .;
  }
}
INSERT BEFORE .bss;
ASSERT(SIZEOF(.pil_retained) == 112, "Gate1 retained size mismatch")
ASSERT(__pil_retained_start >= ORIGIN(RAM), "Gate1 retained below RAM")
ASSERT(__pil_retained_end <= ORIGIN(RAM)+LENGTH(RAM), "Gate1 beyond RAM")
ASSERT(__pil_retained_start >= __data_end, "Gate1 retained overlaps data")
ASSERT(__pil_retained_end <= __bss_start, "Gate1 retained overlaps clear")
ASSERT(__pil_retained_end <= __bss_end, "Gate1 retained overlaps heap")
