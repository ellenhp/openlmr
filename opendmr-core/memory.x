/*
 * C++ enabled linker script for stm32 (1M FLASH, 192K RAM)
 * Developed by TFT: Terraneo Federico Technologies
 * Optimized for use with the Miosix kernel
 */

/*
 * This chip has an unusual quirk that the RAM is divided in two block mapped
 * at two non contiguous memory addresses. I don't know why they've done that,
 * probably doing the obvious thing would have made writing code too easy...
 * Anyway, since hardware can't be changed, we've got to live with that and
 * try to make use of both RAMs.
 *
 * Given the constraints above, this linker script puts:
 * - read only data and code (.text, .rodata, .eh_*) in FLASH
 * - .data and .bss in the "small" 64KB RAM
 * - the 512Byte main (IRQ) stack, stacks and heap in the "large" 128KB RAM.
 *
 * Unfortunately thread stacks can't be put in the small RAM as Miosix
 * allocates them inside the heap.
 */

/*
 * The main stack is used for interrupt handling by the kernel.
 *
 * *** Readme ***
 * This linker script places the main stack (used by the kernel for interrupts)
 * at the bottom of the ram, instead of the top. This is done for two reasons:
 *
 * - as an optimization for microcontrollers with little ram memory. In fact
 *   the implementation of malloc from newlib requests memory to the OS in 4KB
 *   block (except the first block that can be smaller). This is probably done
 *   for compatibility with OSes with an MMU and paged memory. To see why this
 *   is bad, consider a microcontroller with 8KB of ram: when malloc finishes
 *   up the first 4KB it will call _sbrk_r asking for a 4KB block, but this will
 *   fail because the top part of the ram is used by the main stack. As a
 *   result, the top part of the memory will not be used by malloc, even if
 *   available (and it is nearly *half* the ram on an 8KB mcu). By placing the
 *   main stack at the bottom of the ram, the upper 4KB block will be entirely
 *   free and available as heap space.
 *
 * - In case of main stack overflow the cpu will fault because access to memory
 *   before the beginning of the ram faults. Instead with the default stack
 *   placement the main stack will silently collide with the heap.
 * Note: if increasing the main stack size also increase the ORIGIN value in
 * the MEMORY definitions below accordingly.
 */
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
    RAM(wx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_main_stack_size = 0x00000200;                     /* main stack = 512Bytes */
_stack_start  = 0x20000000 + _main_stack_size;
