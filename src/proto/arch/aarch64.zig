const proto = @import("../proto.zig");

pub const FrameHeader = extern struct {
    escape_byte: u8 = proto.escape_byte,
    current_el: u8,
    pad: [6]u8 = undefined,
};

pub const EL3Regs = extern struct {
    pub const msr_postfix = "_EL3";

    // Exception handling
    ELR: u64,
    SPSR: u64,
    FAR: u64,
    ESR: u64,

    // Paging
    TTBR0: u64,
    TCR: u64,
    MAIR: u64,

    // System control
    SCTLR: u64,

    // Secure monitor
    SCR: u64,
};

pub const EL2Regs = extern struct {
    pub const msr_postfix = "_EL2";

    // Exception handling
    ELR: u64,
    SPSR: u64,
    FAR: u64,
    ESR: u64,

    // Paging
    TTBR0: u64,
    TCR: u64,
    MAIR: u64,

    // System control
    SCTLR: u64,

    // Hypervisor control
    HCR: u64,
    CPTR: u64,
    HSTR: u64,

    // Hypervisor timer control
    CNTHCTL: u64,
    CNTVOFF: u64,

    // Hypervisor guest paging
    //VTTBR0: u64,
    VTCR: u64,
};

pub const EL1Regs = extern struct {
    pub const msr_postfix = "_EL1";

    // Exception handling
    ELR: u64,
    SPSR: u64,
    FAR: u64,
    ESR: u64,

    // Paging
    TTBR0: u64,
    TTBR1: u64,
    TCR: u64,
    MAIR: u64,

    // System control
    SCTLR: u64,
    CPACR: u64,
};

pub const GPRs = extern struct {
    SP: u64,
    X30: u64,
    X28: u64,
    X29: u64,
    X26: u64,
    X27: u64,
    X24: u64,
    X25: u64,
    X22: u64,
    X23: u64,
    X20: u64,
    X21: u64,
    X18: u64,
    X19: u64,
    X16: u64,
    X17: u64,
    X14: u64,
    X15: u64,
    X12: u64,
    X13: u64,
    X10: u64,
    X11: u64,
    X8: u64,
    X9: u64,
    X6: u64,
    X7: u64,
    X4: u64,
    X5: u64,
    X2: u64,
    X3: u64,
    X0: u64,
    X1: u64,
};

pub fn ElFrame(comptime el: comptime_int) type {
    return switch (el) {
        1 => extern struct {
            header: FrameHeader,
            el1: EL1Regs,
            gpr: GPRs,
        },
        2 => extern struct {
            header: FrameHeader,
            el1: EL1Regs,
            el2: EL2Regs,
            gpr: GPRs,
        },
        3 => extern struct {
            header: FrameHeader,
            el1: EL1Regs,
            el2: EL2Regs,
            el3: EL3Regs,
            gpr: GPRs,
        },
        else => unreachable,
    };
}
