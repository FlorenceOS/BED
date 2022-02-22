const proto = @import("../proto.zig");

pub const FrameHeader = extern struct {
    escape_byte: u8 = proto.escape_byte,
    current_el: u8,
    pad: [6]u8 = undefined,
};

pub const EL3Regs = extern struct {
    // Exception handling
    ELR_EL3: u64,
    SPSR_EL3: u64,
    FAR_EL3: u64,
    ESR_EL3: u64,

    // Paging
    TTBR0_EL3: u64,
    TCR_EL3: u64,
    MAIR_EL3: u64,

    // System control
    SCTLR_EL3: u64,

    // Secure monitor
    SCR_EL3: u64,
};

pub const EL2Regs = extern struct {
    // Exception handling
    ELR_EL2: u64,
    SPSR_EL2: u64,
    FAR_EL2: u64,
    ESR_EL2: u64,

    // Paging
    TTBR0_EL2: u64,
    TCR_EL2: u64,
    MAIR_EL2: u64,

    // System control
    SCTLR_EL2: u64,

    // Hypervisor control
    HCR_EL2: u64,
    CPTR_EL2: u64,
    HSTR_EL2: u64,
    CPACR_EL1: u64,

    // Hypervisor timer control
    CNTHCTL_EL2: u64,
    CNTVOFF_EL2: u64,

    // Hypervisor guest paging
    //VTTBR0_EL2: u64,
    VTCR_EL2: u64,
};

pub const EL1Regs = extern struct {
    // Exception handling
    ELR_EL1: u64,
    SPSR_EL1: u64,
    FAR_EL1: u64,
    ESR_EL1: u64,

    // Paging
    TTBR0_EL1: u64,
    TTBR1_EL1: u64,
    TCR_EL1: u64,
    MAIR_EL1: u64,

    // System control
    SCTLR_EL1: u64,
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
