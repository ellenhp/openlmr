/* specify the memory areas  */
MEMORY
{
    /* Reserve space for bootloader and settings */
    FLASH(rx)    : ORIGIN = 0x0800C000, LENGTH = 1M - 48K - 128K
    /*
     * Note, the small ram starts at 0x10000000 but it is necessary to add the
     * size of the main stack, so it is 0x10000200.
     */
    SMALLRAM(wx) : ORIGIN = 0x10000000, LENGTH = 64K
    RAM(wx) : ORIGIN = 0x20000200, LENGTH = 128K - 512
}

/* if the main stack starts at any other address on startup the bootloader will boot into fw update mode */
_stack_start  = 0x20000200;
