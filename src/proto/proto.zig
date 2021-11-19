pub const escape_byte: u8 = 3; // ASCII: End of text

pub const aarch64 = @import("arch/aarch64.zig");

// Protocol:
// Debuggee sends a platform frame, starting with `escape_byte`.
// Then a sequence of commands are sent, terminated by a resume_execution command.
// If the resume execution command informs that it should read all registers, the same (but modified)
// frame should be sent back.

pub const Command = packed struct {
    addr: u64 = undefined,
    size: u32 = undefined,
    command_type: enum(u8) {
        // expects platform specified frame to be sent after
        resume_execution = 0,

        // expects `size` bytes to be read after
        read_memory = 1,

        // expects `size` bytes to be sent after
        write_memory = 2,

        // addr: vaddr, reply: 4 bytes, value read
        read_u32 = 3,

        // addr: vaddr, size: value, reply: none
        write_u32 = 4,

        // aarch64:
        //   addr: vaddr
        //   size: AT instruction
        //   reply: 8 bytes, PAR_EL1 value
        resolve_addr = 5,
    },
};
