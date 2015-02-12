MEMORY
{
  ram (rwx) :   ORIGIN = 0x20000000, LENGTH = 192K
  rom (rx) :    ORIGIN = 0x00000000, LENGTH = 1M
}

ram_top = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
    .text : {
        KEEP(*(.irq-vectors))
        *(.text)
        *(.rodata)
        _etext = .;
    } >rom

    .data :
    {
        _data = .;
        *(.data)
        _edata = .;
    } >ram AT > rom

    .bss : {
        _bss = .;
        *(.bss)
        _ebss = .;
    } >ram

    end = .;

    stack_entry = ram_top;
}
