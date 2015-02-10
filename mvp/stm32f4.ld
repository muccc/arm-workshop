MEMORY
{
  ram (rwx) :   ORIGIN = 0x20000000, LENGTH = 20K
  rom (rx) :    ORIGIN = 0x00000000, LENGTH = 128K
}

SECTIONS
{
    .  = 0x0;

    .text : {
        *(vectors)
        *(.text)
        *(.rodata)
    } >rom

    .  = 0x20000000;

    .data : {
        *(.data)
    } >ram AT > rom

    .bss : {
        *(.bss)
    } >ram AT > rom
}  
