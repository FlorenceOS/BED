const debugger_el = @import("build_options").debugger_el;
const proto = @import("proto");
const host = @import("host");

pub const Frame = proto.aarch64.ElFrame(debugger_el);

fn elReg(comptime prefix: []const u8, comptime el: comptime_int) []const u8 {
    return prefix ++ "_EL" ++ [1]u8{'0' + el};
}

export fn int_handler() callconv(.Naked) noreturn {
    asm volatile (
        \\    MSR SPSel, #1
        \\    STP X0,  X1,  [SP, #-0x10]!
        \\    MOV X0,  #0
        \\    B common_handler
        \\.global spx_handler
        \\spx_handler:
        \\    STP X0,  X1,  [SP, #-0x10]!
        \\    MOV X0,  #1
        \\common_handler:
        \\    STP X2,  X3,  [SP, #-0x10]!
        \\    STP X4,  X5,  [SP, #-0x10]!
        \\    STP X6,  X7,  [SP, #-0x10]!
        \\    STP X8,  X9,  [SP, #-0x10]!
        \\    STP X10, X11, [SP, #-0x10]!
        \\    STP X12, X13, [SP, #-0x10]!
        \\    STP X14, X15, [SP, #-0x10]!
        \\    STP X16, X17, [SP, #-0x10]!
        \\    STP X18, X19, [SP, #-0x10]!
        \\    STP X20, X21, [SP, #-0x10]!
        \\    STP X22, X23, [SP, #-0x10]!
        \\    STP X24, X25, [SP, #-0x10]!
        \\    STP X26, X27, [SP, #-0x10]!
        \\    STP X28, X29, [SP, #-0x10]!
        \\    MRS X1, SP_EL0
        \\    STP X1,  X30, [SP, #-0x10]!
        \\    SUB SP, SP, %[frame_size_offset]
        \\
        \\    MOV X1, SP
        \\.global branch_to_handle_interrupt
        \\.extern __branch_to_handle_interrupt_value
        \\branch_to_handle_interrupt:
        \\    .4byte __branch_to_handle_interrupt_value // BL handle_interrupt
        \\
        \\    ADD SP, SP, %[frame_size_offset]
        \\    LDP X1,  X30, [SP], 0x10
        \\    CBNZ X0, skip_because_spx
        \\    MSR SP_EL0, X1
        \\skip_because_spx:
        \\    LDP X28, X29, [SP], 0x10
        \\    LDP X26, X27, [SP], 0x10
        \\    LDP X24, X25, [SP], 0x10
        \\    LDP X22, X23, [SP], 0x10
        \\    LDP X20, X21, [SP], 0x10
        \\    LDP X18, X19, [SP], 0x10
        \\    LDP X16, X17, [SP], 0x10
        \\    LDP X14, X15, [SP], 0x10
        \\    LDP X12, X13, [SP], 0x10
        \\    LDP X10, X11, [SP], 0x10
        \\    LDP X8,  X9,  [SP], 0x10
        \\    LDP X6,  X7,  [SP], 0x10
        \\    LDP X4,  X5,  [SP], 0x10
        \\    LDP X2,  X3,  [SP], 0x10
        \\    CMP X0, #0
        \\    LDP X0,  X1,  [SP], 0x10
        \\    B.NE eret_on_spx
        \\    MSR SPSel, #0
        \\eret_on_spx:
        \\    ERET
        \\
        \\.global fatal_error
        \\fatal_error:
        \\    WFE
        \\    B fatal_error
        :
        : [frame_size_offset] "i" (@as(u16, @offsetOf(Frame, "gpr")))
    );
    unreachable;
}

fn fillStruct(s_ptr: anytype) callconv(.Inline) void {
    const T = @TypeOf(s_ptr.*);

    inline for (@typeInfo(T).Struct.fields) |f| {
        const value = asm ("MRS %[reg], " ++ f.name ++ T.msr_postfix ++ "\n"
            : [reg] "=r" (-> f.field_type)
        );
        @field(s_ptr.*, f.name) = value;
    }

    asm volatile ("" ::: "memory");
}

fn applyStruct(s: anytype) callconv(.Inline) void {
    const T = @TypeOf(s);

    // This copy is for better code generation (volatile asm writing to the MSRs clobber memory)
    const copy = s;

    inline for (@typeInfo(T).Struct.fields) |f| {
        const value = @field(copy, f.name);
        _ = asm volatile ("MSR " ++ f.name ++ T.msr_postfix ++ ", %[reg]\n"
            : [val] "=r" (-> u64)
            : [reg] "r" (value)
        );
    }
}

pub fn resolveAddr(addr: u64, instr: u32) callconv(.Inline) u64 {
    const instr_loc = asm volatile (
        \\    ADR %[instr_loc], instr_loc
        : [instr_loc] "=r" (-> u64)
    );

    return asm volatile ( // zig fmt: off
        \\    STR W8, [%[instr_loc]] 
        \\    DC CVAU, %[instr_loc]
        \\    DSB NSHST
        \\    IC IVAU, %[instr_loc]
        \\    DSB NSH
        \\    ISB
        \\instr_loc:
        \\    BRK #0
        \\    ISB
        \\    MRS %[result], PAR_EL1
        : [result] "=r" (-> u64)
        : [instr] "{w8}" (instr)
        , [instr_loc] "r" (instr_loc)
        , [_] "{x0}" (addr)
        // zig fmt: on
    );
}

fn sendReg(comptime reg_name: []const u8) u64 {
    const reg = asm volatile ("MRS %[out], " ++ reg_name
        : [out] "=r" (-> u64)
    );

    host.callbacks.send_fn(@ptrCast([*:0]const u8, &reg), 8);

    return reg;
}

fn doOsLock() callconv(.Inline) void {
    // OS lock time. Thanks.
    const OSLSR_EL1 = asm volatile (
        \\ MRS %[oslsr], OSLSR_EL1
        : [oslsr] "=r" (-> u64)
    );

    const OSLM = @truncate(u2, OSLSR_EL1 & 1) | @truncate(u2, (OSLSR_EL1 >> 2) & 2);
    const OSLK = @truncate(u1, OSLSR_EL1 >> 1);

    switch (OSLM) {
        0b00 => return, // OS Lock not implemented
        0b10 => { // OS Lock implemented
            if (OSLK == 0b1) { // OS lock is locked! Gotta unlock it for single stepping!
                asm volatile (
                    \\ MSR OSLAR_EL1, %[zero]
                    :
                    : [zero] "r" (@as(u64, 0))
                );
            }
        },
        else => unreachable,
    }
}

// If it's important that the debuggee cannot interface with
// debugger registers, enable this to trap all debugger registers
// that can be trapped if you're debugging a lower EL
const contain_debuggee = false;

fn setupDebuggerRegs() callconv(.Inline) void {
    if (comptime (debugger_el >= 3)) {
        asm volatile ("MSR MDCR_EL3, %[mdscr]"
            :
            // zig fmt: off
            : [mdscr] "r" (@as(u64,
                (if(comptime(contain_debuggee)) 0
                    | (1 << 6) // TPM
                    | (1 << 9) // TDA
                    | (1 << 10) // TDOSA
                    | (0b00 << 12) // NSPB
                    | (0b11 << 14) // SPD32
                    | (1 << 17) // SPME
                    | (1 << 18) // STE
                    | (1 << 19) // TTRF
                    | (1 << 20) // EDAD
                    | (1 << 21) // EPMAD
                    | (0 << 23) // SCCD
                    | (1 << 27) // TDCC
                    | (1 << 28) // MTPME
                    | (0 << 34) // MCCD
                    | (0 << 35) // MPMX
                    | (0 << 36) // EnPMSN
                else 0
                    | (0 << 6) // TPM
                    | (0 << 9) // TDA
                    | (0 << 10) // TDOSA
                    | (0b11 << 12) // NSPB
                    | (0b11 << 14) // SPD32
                    | (1 << 17) // SPME
                    | (1 << 18) // STE
                    | (0 << 19) // TTRF
                    | (1 << 20) // EDAD
                    | (1 << 21) // EPMAD
                    | (0 << 23) // SCCD
                    | (0 << 27) // TDCC
                    | (1 << 28) // MTPME
                    | (0 << 34) // MCCD
                    | (0 << 35) // MPMX
                    | (1 << 36) // EnPMSN
                )

                | (0 << 16) // SDD
            ))
            // zig fmt: on
        );
    }

    if (comptime (debugger_el >= 2)) {
        // @TODO `contain_debuggee`
    }

    if (comptime (debugger_el >= 1)) {
        // @TODO `contain_debuggee`
        const old_mdscr = asm volatile (
            \\ MRS %[mdscr], MDSCR_EL1
            : [mdscr] "=r" (-> u64)
        );

        // zig fmt: off
        asm volatile (
            \\MSR MDSCR_EL1, %[mdscr]
            :
            : [mdscr] "r" (old_mdscr
                | (1 << 0) // SS, software step enable
                | (1 << 13) // KDE, local kernel debug enable
                | (1 << 15) // MDE, monitor debug events
            )
        );
        // zig fmt: on
    }
}

pub fn install() void {
    const evt_base = asm volatile ("ADR %[reg], evt_base"
        : [reg] "=r" (-> u64)
    );

    asm volatile ("MSR " ++ elReg("VBAR", debugger_el) ++ ", %[evt]"
        :
        : [evt] "r" (evt_base)
    );

    if(comptime(debugger_el < 3)) {
        // We should only unlock the OS lock if
        // we aren't on el3, as it's only needed for
        // software stepping, and we can't do that in EL3
        doOsLock();
    }
    setupDebuggerRegs();

    // Enable debugging at ELd
    asm volatile (
        \\ MSR DAIFclr, #8
        \\
    );

    // Trap into the debugger at the current EL
    switch (debugger_el) {
        1 => asm volatile (
            \\ SVC #42069
        ),
        2 => asm volatile (
            \\ HVC #42069
        ),
        3 => asm volatile (
            \\ SMC #42069
        ),
        else => @compileError("wtf"),
    }
}

export fn handle_interrupt(was_in_curr_el_spx_context: u64, frame: *Frame) callconv(.C) u64 {
    frame.header.current_el = debugger_el;

    if (comptime (@hasField(Frame, "el1")))
        fillStruct(&frame.el1);
    if (comptime (@hasField(Frame, "el2")))
        fillStruct(&frame.el2);
    if (comptime (@hasField(Frame, "el3")))
        fillStruct(&frame.el3);

    if (was_in_curr_el_spx_context != 0) {
        frame.gpr.SP = @ptrToInt(frame) + @sizeOf(Frame);
    }

    host.handle_interrupt(frame);

    if (comptime (@hasField(Frame, "el3")))
        applyStruct(frame.el3);
    if (comptime (@hasField(Frame, "el2")))
        applyStruct(frame.el2);
    if (comptime (@hasField(Frame, "el1")))
        applyStruct(frame.el1);

    return was_in_curr_el_spx_context;
}
