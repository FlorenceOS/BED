const proto = @import("proto");
const std = @import("std");
const gdb = @import("gdb");
const client = @import("client.zig");

pub const target_xml = @embedFile("aarch64.xml");

pub const num_default_registers = 32;

pub const HookedInstr = [4]u8;

fn readWithEL(comptime EL: comptime_int) !Frame {
    const RemoteFrame = proto.aarch64.ElFrame(EL);
    var frame: RemoteFrame = undefined;

    try client.device_reader.readNoEof(std.mem.asBytes(&frame)[@sizeOf(proto.aarch64.FrameHeader)..]);

    std.log.info("Raw frame: {any}", .{frame});

    return Frame{
        .gpr = frame.gpr,
        .debugger_el = EL,
        .el1 = frame.el1,
        .gdb_halt_message = undefined,
        .el2 = if (comptime (EL < 2)) undefined else frame.el2,
        .el3 = if (comptime (EL < 3)) undefined else frame.el3,
    };
}

pub fn readFrame() !Frame {
    while (true) {
        const first_byte = try client.device_reader.readByte();

        if (first_byte == proto.escape_byte) {
            var frame_header: proto.aarch64.FrameHeader = undefined;
            frame_header.escape_byte = first_byte;

            try client.device_reader.readNoEof(std.mem.asBytes(&frame_header)[1..]);
            std.log.info("Frame header: {any}", .{frame_header});
            var frame = try switch (frame_header.current_el) {
                1 => readWithEL(1),
                2 => readWithEL(2),
                3 => readWithEL(3),
                else => unreachable,
            };

            const ELR = frame.debuggerRegs().ELR.*;

            const hit_step_hook = blk: {
                // We have to translate this back into an address for us
                // since we may be using a different page map than the debuggee
                const current_instr_addr = frame.operableAddr(ELR, .r) catch |err| {
                    switch (err) {
                        error.CannotWrite => unreachable,

                        error.PaddrNotIdentityMapped,
                        error.NotMapped,
                        => break :blk false,

                        else => {},
                    }
                    return err;
                };

                break :blk client.hasStepHookAtAddr(current_instr_addr);
            };

            const ESR = frame.debuggerRegs().ESR.*;
            const ec = @truncate(u6, ESR >> 26) & 0x3F;

            switch (ec) {
                0b010111, // AA64 SMC
                0b010110, // AA64 HVC
                0b010101, // AA64 SVC
                0b010011, // AA32 SVC
                => {
                    if (hit_step_hook) {
                        frame.gdb_halt_message = "S05"; // SIGTRAP

                        // We need to rewind the instruction pointer to
                        // execute the real instruction once it has been restored
                        frame.debuggerRegs().ELR.* -= 4;
                    } else {
                        frame.gdb_halt_message = "T05swbreak:;"; // SIGTRAP from software breakpoint
                    }
                },

                0b001110 => { // Illegal execution state
                    frame.gdb_halt_message = "S04"; // SIGILL
                },

                0b001000 => { // Invalid opcode
                    frame.gdb_halt_message = "S04"; // SIGILL
                },

                0b100010 => { // PC alignment fault
                    frame.gdb_halt_message = "S07"; // SIGBUS
                },

                0b100000, // Page fault on instruction fetch, lower EL
                0b100001, // Page fault on instruction fetch, same EL
                0b100100, // Page fault on data access, lower EL
                0b100101, // Page fault on data access, same EL
                => {
                    frame.gdb_halt_message = "S0B"; // SIGSEGV
                },

                0b111100 => { // BRK instruction, same EL
                    if (hit_step_hook) {
                        frame.gdb_halt_message = "S05"; // SIGTRAP
                    } else {
                        frame.gdb_halt_message = "T05swbreak:;"; // SIGTRAP from software breakpoint
                    }
                },

                else => |v| {
                    std.log.err("BAD ESR??? 0b{b}", .{v});
                    frame.gdb_halt_message = "S02"; // SIGINT
                },
            }

            return frame;
        }
    }
}

fn atOp(
    op1: u3,
    CR_m: u1,
    op2: u3,
    Xt: u5,
) u32 {
    // zig fmt: off
    return 0xD5087800
        | (@as(u32, op1) << 16)
        | (@as(u32, CR_m) << 8)
        | (@as(u32, op2) << 5)
        | (@as(u32, Xt) << 0)
    ;
    // zig fmt: on
}

fn atInstr(
    rw: client.RW, // Require read or write access
    el: enum { el0, el1, el2, el3, el1pan }, // el1pan is EL1 but with PAN checks
    stages: enum { stage1, stages12 }, // Do just stage1 or both stage1 and 2 translation
    Xt: u5, // Register with virtual address
) error{InvalidArgument}!u32 {
    return switch (stages) {
        .stages12 => switch (el) {
            .el0 => switch (rw) {
                .r => atOp(4, 0, 6, Xt),
                .w => atOp(4, 0, 7, Xt),
            },
            .el1 => switch (rw) {
                .r => atOp(4, 0, 4, Xt),
                .w => atOp(4, 0, 5, Xt),
            },
            else => error.InvalidArgument,
        },
        .stage1 => switch (el) {
            .el0 => switch (rw) {
                .r => atOp(0, 0, 2, Xt),
                .w => atOp(0, 0, 3, Xt),
            },
            .el1 => switch (rw) {
                .r => atOp(0, 0, 0, Xt),
                .w => atOp(0, 0, 1, Xt),
            },
            .el1pan => switch (rw) {
                .r => atOp(0, 1, 0, Xt),
                .w => atOp(0, 1, 1, Xt),
            },
            .el2 => switch (rw) {
                .r => atOp(4, 0, 0, Xt),
                .w => atOp(4, 0, 1, Xt),
            },
            .el3 => switch (rw) {
                .r => atOp(6, 0, 0, Xt),
                .w => atOp(6, 0, 1, Xt),
            },
        },
    };
}

fn calcNextAddr(instr_addr: u64, imm: anytype) u64 {
    const imm_len = @bitSizeOf(@TypeOf(imm));

    // First, make the imm signed
    const s_imm = @bitCast(std.meta.Int(.signed, imm_len), imm);

    // Now make it an instruction address offset
    const full_s_imm = @intCast(i64, s_imm) * 4;

    // Wrapping add the values
    return instr_addr +% @bitCast(u64, full_s_imm);
}

fn spsrSavedEl(spsr: u64) u2 {
    return @truncate(u2, spsr >> 2);
}

fn translateVaddr(vaddr: u64, el: u2, op: client.RW) !?u64 {
    // TODO: Do something more reasonable here.
    // Is there any way we can succeed with the page table walk, try to access it and catch the SError
    // in case the address is mapped but not backed by any hardware?
    if (vaddr == 0)
        return null;

    const instr = try atInstr(
        op,
        switch (el) {
            0 => .el0,
            1 => .el1,
            2 => .el2,
            3 => .el3,
        },
        .stage1, // Assume 2 stage translation is disabled for now
        0, // vaddr in X0
    );
    try client.sendDeviceCommand(.{
        .command_type = .resolve_addr,
        .addr = vaddr,
        .size = instr,
    });

    const PAR_EL1 = try client.device_reader.readIntLittle(u64);
    std.log.info("Addr translation of 0x{X} at EL{d} for op {any} -> PAR_EL1=0x{X}", .{ vaddr, el, op, PAR_EL1 });

    const attr = @truncate(u8, PAR_EL1 >> 56);

    const paddr = @truncate(u48, PAR_EL1 & ~@as(u64, 0xFFF));

    const ns = @truncate(u1, PAR_EL1 >> 9);

    const sh = @truncate(u2, PAR_EL1 >> 7);

    const f = @truncate(u1, PAR_EL1);

    if (f != 0) {
        std.log.info("Result: Not mapped!", .{});
        return null;
    }

    std.log.info("Result addr: 0x{X}, attr=0x{X} ns={} sh={X}", .{
        paddr,
        attr,
        ns != 0,
        sh,
    });

    return paddr;
}

pub const Frame = struct {
    gpr: proto.aarch64.GPRs,

    debugger_el: u2,

    fake_cpsr: u64 = undefined, // Thanks GDB for requiring this on aarch64

    el1: proto.aarch64.EL1Regs,
    el2: proto.aarch64.EL2Regs,
    el3: proto.aarch64.EL3Regs,

    gdb_halt_message: []const u8 = undefined,

    pub fn pagingEnabled(self: *@This(), el: u2) bool {
        switch (el) {
            0, 1 => return @truncate(u1, self.el1.SCTLR) == 1,
            2 => return @truncate(u1, self.el2.SCTLR) == 1,
            3 => return @truncate(u1, self.el3.SCTLR) == 1,
        }
    }

    pub fn pageSize(self: *@This(), addr: u64) usize {
        switch (self.savedEl()) {
            0, 1 => return calcPageSize(self.el1.TCR, addr),
            2 => return calcPageSize(self.el2.TCR, addr),
            3 => return calcPageSize(self.el3.TCR, addr),
        }
    }

    fn addrForOpAtEL(self: *@This(), vaddr: u64, other_el: u2, op: client.RW) !u64 {
        // @TODO: Don't ignore self.pagingEnabled() calls below, speed can
        // be gained here. Problem is that I'm not really sure how to tell if
        // 0xFFFFFFFFFFFF0000 et al is a bad address otherwise.

        if (self.debugger_el == other_el) { // Same EL
            // We only need to verify we can do the op on the current EL
            if (true or self.pagingEnabled(self.debugger_el)) {
                if (try translateVaddr(vaddr, self.debugger_el, op)) |_| {
                    return vaddr;
                }
                if (op == .w) { // Check if we failed because of our permissions
                    if (try translateVaddr(vaddr, self.debugger_el, .r)) |_| {
                        return error.CannotWrite;
                    }
                }
                return error.NotMapped;
            }
            return vaddr;
        } else { // Lower EL
            // First we have to get the paddr of the vaddr accessed
            // We only check for read access on the lower EL, we may be able to write even if they can't
            const paddr = if (true or self.pagingEnabled(other_el))
                ((try translateVaddr(vaddr, other_el, .r)) orelse return error.NotMapped)
            else
                vaddr;

            // Now we need a virtual address for us to write to, check if this addr happens to be identity mapped
            // Maybe we could look for it elsewhere, but this is fine for now
            if (false or !self.pagingEnabled(self.debugger_el)) return paddr;

            const debugger_paddr = (try translateVaddr(paddr, self.debugger_el, op)) orelse {
                if (op == .w) {
                    if (try translateVaddr(paddr, self.debugger_el, .r)) |_| {
                        return error.CannotWrite;
                    }
                }
                return error.PaddrNotIdentityMapped;
            };

            if (paddr != debugger_paddr) return error.PaddrNotIdentityMapped;

            return paddr;
        }
    }

    pub fn operableAddr(self: *@This(), vaddr: u64, op: client.RW) !u64 {
        return self.addrForOpAtEL(vaddr, self.savedEl(), op);
    }

    fn debuggerRegs(self: *@This()) struct {
        ESR: *u64,
        ELR: *u64,
        FAR: *u64,
        SPSR: *u64,
    } {
        return switch (self.debugger_el) {
            1 => .{
                .ESR = &self.el1.ESR,
                .ELR = &self.el1.ELR,
                .FAR = &self.el1.FAR,
                .SPSR = &self.el1.SPSR,
            },
            2 => .{
                .ESR = &self.el2.ESR,
                .ELR = &self.el2.ELR,
                .FAR = &self.el2.FAR,
                .SPSR = &self.el2.SPSR,
            },
            3 => .{
                .ESR = &self.el3.ESR,
                .ELR = &self.el3.ELR,
                .FAR = &self.el3.FAR,
                .SPSR = &self.el3.SPSR,
            },
            else => unreachable,
        };
    }

    fn savedEl(self: *@This()) u2 {
        return spsrSavedEl(self.debuggerRegs().SPSR.*);
    }

    // Get a register identified by "regnum" in target_xml
    pub fn getGdbReg(self: *@This(), regnum: usize) ?*u64 {
        return switch (regnum) {
            // zig fmt: off
            0  => &self.gpr.X0,
            1  => &self.gpr.X1,
            2  => &self.gpr.X2,
            3  => &self.gpr.X3,
            4  => &self.gpr.X4,
            5  => &self.gpr.X5,
            6  => &self.gpr.X6,
            7  => &self.gpr.X7,
            8  => &self.gpr.X8,
            9  => &self.gpr.X9,
            10 => &self.gpr.X10,
            11 => &self.gpr.X11,
            12 => &self.gpr.X12,
            13 => &self.gpr.X13,
            14 => &self.gpr.X14,
            15 => &self.gpr.X15,
            16 => &self.gpr.X16,
            17 => &self.gpr.X17,
            18 => &self.gpr.X18,
            19 => &self.gpr.X19,
            20 => &self.gpr.X20,
            21 => &self.gpr.X21,
            22 => &self.gpr.X22,
            23 => &self.gpr.X23,
            24 => &self.gpr.X24,
            25 => &self.gpr.X25,
            26 => &self.gpr.X26,
            27 => &self.gpr.X27,
            28 => &self.gpr.X28,
            29 => &self.gpr.X29,
            30 => &self.gpr.X30,
            31 => &self.gpr.SP,
            // zig fmt: on
            32 => self.debuggerRegs().ELR,
            33 => {
                // zig fmt: off
                self.fake_cpsr = 0
                    | (self.debuggerRegs().SPSR.* & (0
                        | (7 << 6) // AIF
                        | (0x1F << 27) // NZCVQ
                        | (1 << 22) // PAN
                        | (1 << 9) // E
                    ))
                    | (1 << 4) // RES1
                    | @as(u64, switch (self.debuggerRegs().SPSR.* & 0xF) { // M
                        0b0000 => 0b0000, // EL0t => User
                        0b0100 => 0b1111, // EL1t => System
                        0b0101 => 0b0010, // EL1h => IRQ
                        0b1000 => 0b1010, // EL2t => Hypervisor
                        0b1001 => 0b1010, // EL2h => Hypervisor
                        0b1100 => 0b0110, // EL3t => Monitor
                        0b1101 => 0b0110, // EL3h => Monitor
                        else => undefined,
                    })
                ;
                // zig fmt: on

                std.log.info("Fake CPSR: 0x{X}", .{self.fake_cpsr});

                return &self.fake_cpsr;
            },

            34 => if (self.debugger_el < 3) null else &self.el3.ELR,
            35 => if (self.debugger_el < 3) null else &self.el3.SPSR,
            36 => if (self.debugger_el < 3) null else &self.el3.FAR,
            37 => if (self.debugger_el < 3) null else &self.el3.ESR,

            38 => if (self.debugger_el < 2) null else &self.el2.ESR,
            39 => if (self.debugger_el < 2) null else &self.el2.SPSR,
            40 => if (self.debugger_el < 2) null else &self.el2.FAR,
            41 => if (self.debugger_el < 2) null else &self.el2.ESR,

            42 => if (self.debugger_el < 1) null else &self.el1.ESR,
            43 => if (self.debugger_el < 1) null else &self.el1.SPSR,
            44 => if (self.debugger_el < 1) null else &self.el1.FAR,
            45 => if (self.debugger_el < 1) null else &self.el1.ESR,

            46 => if (self.debugger_el < 3) null else &self.el3.TTBR0,
            47 => if (self.debugger_el < 3) null else &self.el3.TCR,
            48 => if (self.debugger_el < 3) null else &self.el3.MAIR,

            49 => if (self.debugger_el < 2) null else &self.el2.TTBR0,
            50 => if (self.debugger_el < 2) null else &self.el2.TCR,
            51 => if (self.debugger_el < 2) null else &self.el2.MAIR,

            52 => if (self.debugger_el < 1) null else &self.el1.TTBR0,
            53 => if (self.debugger_el < 1) null else &self.el1.TTBR1,
            54 => if (self.debugger_el < 1) null else &self.el1.TCR,
            55 => if (self.debugger_el < 1) null else &self.el1.MAIR,

            56 => if (self.debugger_el < 3) null else &self.el3.SCTLR,
            57 => if (self.debugger_el < 3) null else &self.el3.SCR,

            58 => if (self.debugger_el < 2) null else &self.el2.SCTLR,
            59 => if (self.debugger_el < 2) null else &self.el2.HCR,
            60 => if (self.debugger_el < 2) null else &self.el2.CPTR,
            61 => if (self.debugger_el < 2) null else &self.el2.HSTR,
            62 => if (self.debugger_el < 2) null else &self.el2.CNTHCTL,
            63 => if (self.debugger_el < 2) null else &self.el2.CNTVOFF,
            64 => if (self.debugger_el < 2) null else &self.el2.VTCR,

            65 => if (self.debugger_el < 1) null else &self.el1.SCTLR,
            66 => if (self.debugger_el < 1) null else &self.el1.CPACR,

            else => null,
        };
    }

    pub fn informHalt(self: *@This(), gdb_stream: *gdb.Stream) !void {
        try gdb_stream.send(self.gdb_halt_message);
    }

    fn hookWithInstrAtAddrForEL(self: *@This(), addr: u64, EL: u2, instr: [4]u8, save_callback: anytype) !void {
        // Make sure the address is writeable
        const our_addr = self.addrForOpAtEL(addr, EL, .w) catch |err| {
            std.log.err("Cannot write hook at address 0x{X} at EL{d}!! Cause: {}", .{ addr, EL, err });
            return;
        };

        // Save the current instruction
        try save_callback(our_addr, try client.readBytes(our_addr, 4));

        // Write the new one
        try client.writeMemory(our_addr, &instr);
    }

    fn swHookWithInstrAtAddr(self: *@This(), addr: u64, instr: [4]u8) !void {
        return self.hookWithInstrAtAddrForEL(addr, self.savedEl(), instr, client.swBreakpointHook);
    }

    fn stepHookWithInstrAtAddr(self: *@This(), addr: u64, instr: [4]u8) !void {
        return self.hookWithInstrAtAddrForEL(addr, self.savedEl(), instr, client.singleStepHook);
    }

    fn hookWithInstr(self: *@This(), instr: [4]u8) !void {
        const current_addr = self.debuggerRegs().ELR.*;

        const curr_instr_bytes = try client.readBytes(current_addr, 4);
        const curr_instr = std.mem.readIntLittle(u32, &curr_instr_bytes);

        // zig fmt: off
        switch (@truncate(u8, curr_instr >> 24)) {
            // Conditional branch to immediate (B.cond)
            // 0b0101010x
            0b01010100, 0b01010101,

            // Compare and branch to immediate (CB{N,}Z)
            // 0bx011010x
            0b00110100, 0b00110101,
            0b10110100, 0b10110101,
            => {
                const imm = @truncate(u19, curr_instr >> 5);
                try self.stepHookWithInstrAtAddr(calcNextAddr(current_addr, imm), instr);
                try self.stepHookWithInstrAtAddr(current_addr + 4, instr);
            },

            // Unconditional branch to register (BR, BLR, RET, ...)
            // 0b1101011x
            0b11010110, 0b11010111,
            => {
                const opc = @truncate(u4, curr_instr >> 21);
                const op2 = @truncate(u5, curr_instr >> 16);
                const op3 = @truncate(u6, curr_instr >> 10);
                const Rn = @truncate(u5, curr_instr >> 5);
                const op4 = @truncate(u5, curr_instr);

                if (opc == 0b0100 and op2 == 0b11111 and op3 == 0b000000 and Rn == 0b11111 and op4 == 0b00000) { // ERET
                    const debuggee_elr = switch (self.savedEl()) {
                        0 => {
                            std.log.err("EL0 can't ERET!", .{});
                            return;
                        },
                        1 => self.el1.ELR,
                        2 => self.el2.ELR,
                        3 => self.el3.ELR,
                    };

                    const debuggee_spsr = switch (self.savedEl()) {
                        0 => unreachable,
                        1 => self.el1.SPSR,
                        2 => self.el2.SPSR,
                        3 => self.el3.SPSR,
                    };

                    const debuggee_target_el = spsrSavedEl(debuggee_spsr);

                    try self.hookWithInstrAtAddrForEL(debuggee_elr, debuggee_target_el, instr, client.singleStepHook);
                } else {
                    // Probably just an unconditional branch to the value of Rn
                    // If the register number is 31, that encodes XZR and not SP
                    const reg_val = if (Rn == 31) 0 else (self.getGdbReg(Rn) orelse unreachable).*;

                    try self.stepHookWithInstrAtAddr(reg_val, instr);
                }
            },

            // Unconditional branch to immediate (B, BL)
            // 0bx00101xx
            0b00010100, 0b00010101, 0b00010110, 0b00010111,
            0b10010100, 0b10010101, 0b10010110, 0b10010111,
            => {
                const imm = @truncate(u26, curr_instr);
                try self.stepHookWithInstrAtAddr(calcNextAddr(current_addr, imm), instr);
            },

            // Test and branch to immediate (TB{N,}Z)
            // 0bx011011x
            0b00110110, 0b00110111,
            0b10110110, 0b10110111,
            => {
                const imm = @truncate(u14, curr_instr >> 5);
                try self.stepHookWithInstrAtAddr(calcNextAddr(current_addr, imm), instr);
                try self.stepHookWithInstrAtAddr(current_addr + 4, instr);
            },

            // We assume everything that isn't a branch will just end up at PC + 4.
            // I... Think that's correct...?
            else => {
                try self.stepHookWithInstrAtAddr(current_addr + 4, instr);
            },
        }
        // zig fmt: on
    }

    fn chooseInstructionForBreakpoint(self: *@This()) [4]u8 {
        // Oh boy, stepping with a debugger in EL3 is a bit of a mess.
        // Debug exceptions except BRK are not available. This means that
        // if we want to trap from a lower EL into EL3, we have to use an
        // SMC instruction. That's fine and all except for the fact that
        // they advance the PC by 4 _BEFORE_ triggering the exception
        // for obvious reasons. That means we have to rewind the PC by 4
        // on one of these before resuming.

        // First, let's figure out if we're running at EL3 or a lower EL.

        if (self.savedEl() < 3) {
            // We need to do an `smc` call to get back into the debugger.
            return .{ 0xA3, 0x8A, 0x14, 0xD4 }; // SMC #42069
        } else {
            // We could do either an `smc` or `brk` in struction to get back into the debugger
            return .{ 0xA0, 0x8A, 0x34, 0xD4 }; // BRK #42069
        }
    }

    pub fn setSwBreak(self: *@This(), addr: u64) !void {
        try self.swHookWithInstrAtAddr(addr, self.chooseInstructionForBreakpoint());
    }

    pub fn doStep(self: *@This(), step: bool) !void {
        if (step and self.debugger_el == 3) {
            try self.hookWithInstr(self.chooseInstructionForBreakpoint());
        } else {
            const ss_bit: u64 = 1 << 21;

            if (step) {
                self.debuggerRegs().SPSR.* |= ss_bit;
            } else {
                self.debuggerRegs().SPSR.* &= ~ss_bit;
            }
        }

        try client.resumeExecution();

        try switch (self.debugger_el) {
            1 => self.writeFrameAtEL(1),
            2 => self.writeFrameAtEL(2),
            3 => self.writeFrameAtEL(3),
            else => unreachable,
        };
    }

    fn writeFrameAtEL(self: *@This(), comptime el: u2) !void {
        const RemoteFrame = proto.aarch64.ElFrame(el);
        var frame = std.mem.zeroes(RemoteFrame);
        frame.el1 = self.el1;
        frame.gpr = self.gpr;
        if (comptime (el >= 2)) {
            frame.el2 = self.el2;
        }
        if (comptime (el >= 3)) {
            frame.el3 = self.el3;
        }

        try client.device_writer.writeIntLittle(u32, @intCast(u32, @sizeOf(RemoteFrame)));
        try client.device_writer.writeAll(std.mem.asBytes(&frame));
    }
};
