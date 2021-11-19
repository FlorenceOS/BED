ENTRY(_start)

BL = 0x94000000;
B  = 0x14000000;

SECTIONS {
    . = 0;
    .blob . : {
        *(.text.start*)
        *(.text*)
        *(.data*)
        *(.bss*)
        *(.rodata*)
        . = ALIGN(0x800);
        evt_base = .;
        # The only reason we assemble shit in the linker is because it refuses to do the relative relocations
        #  because we don't know the address, even when linking position independently. Whyy???????
        . = evt_base + 0x000; LONG(B | (((int_handler - (evt_base + 0x000))/4) & 0x3FFFFFF)); # curr_el_sp0_sync
        . = evt_base + 0x080; LONG(B | (((int_handler - (evt_base + 0x080))/4) & 0x3FFFFFF)); # curr_el_sp0_irq
        . = evt_base + 0x100; LONG(B | (((int_handler - (evt_base + 0x100))/4) & 0x3FFFFFF)); # curr_el_sp0_fiq
        . = evt_base + 0x180; LONG(B | (((int_handler - (evt_base + 0x180))/4) & 0x3FFFFFF)); # curr_el_sp0_serror
        . = evt_base + 0x200; LONG(B | (((spx_handler - (evt_base + 0x200))/4) & 0x3FFFFFF)); # curr_el_spx_sync
        . = evt_base + 0x280; LONG(B | (((spx_handler - (evt_base + 0x280))/4) & 0x3FFFFFF)); # curr_el_spx_irq
        . = evt_base + 0x300; LONG(B | (((spx_handler - (evt_base + 0x300))/4) & 0x3FFFFFF)); # curr_el_spx_fiq
        . = evt_base + 0x380; LONG(B | (((spx_handler - (evt_base + 0x380))/4) & 0x3FFFFFF)); # curr_el_spx_serror
        . = evt_base + 0x400; LONG(B | (((int_handler - (evt_base + 0x400))/4) & 0x3FFFFFF)); # lower_el_sp0_sync
        . = evt_base + 0x480; LONG(B | (((int_handler - (evt_base + 0x480))/4) & 0x3FFFFFF)); # lower_el_sp0_irq
        . = evt_base + 0x500; LONG(B | (((int_handler - (evt_base + 0x500))/4) & 0x3FFFFFF)); # lower_el_sp0_fiq
        . = evt_base + 0x580; LONG(B | (((int_handler - (evt_base + 0x580))/4) & 0x3FFFFFF)); # lower_el_sp0_serror
        . = evt_base + 0x600; LONG(B | (((int_handler - (evt_base + 0x600))/4) & 0x3FFFFFF)); # lower_el_spx_sync
        . = evt_base + 0x680; LONG(B | (((int_handler - (evt_base + 0x680))/4) & 0x3FFFFFF)); # lower_el_spx_irq
        . = evt_base + 0x700; LONG(B | (((int_handler - (evt_base + 0x700))/4) & 0x3FFFFFF)); # lower_el_spx_fiq
        . = evt_base + 0x780; LONG(B | (((int_handler - (evt_base + 0x780))/4) & 0x3FFFFFF)); # lower_el_spx_serror
        __branch_to_handle_interrupt_value = BL | (((handle_interrupt - branch_to_handle_interrupt)/4) & 0x3FFFFFF);
    }
    . += 0x1000;
    int_stack_top = .;

    .shstrtab : {
        *(.shstrtab)
    }

    /DISCARD/ : {
        *(*)
    }
}
