const arch = @import("root");
const proto = @import("proto");

pub export var callbacks: extern struct {
    send_fn: fn ([*c]const u8, usize) callconv(.C) void,
    recv_fn: fn ([*c]u8, usize) callconv(.C) void,
} = undefined;

export fn _start(send: @TypeOf(callbacks.send_fn), recv: @TypeOf(callbacks.recv_fn)) linksection(".text.start") void {
    callbacks.send_fn = send;
    callbacks.recv_fn = recv;

    @call(.{ .modifier = .always_inline }, arch.install, .{});
}

pub fn handle_interrupt(frame: *arch.Frame) callconv(.Inline) void {
    frame.header.escape_byte = proto.escape_byte;
    callbacks.send_fn(@ptrCast([*c]const u8, frame), @sizeOf(arch.Frame));
    while (true) {
        var cmd: proto.Command = undefined;
        callbacks.recv_fn(@ptrCast([*c]u8, &cmd), @sizeOf(@TypeOf(cmd)));
        switch (cmd.command_type) {
            .resume_execution => {
                callbacks.recv_fn(@ptrCast([*c]u8, frame), @sizeOf(arch.Frame));
                return;
            },
            .read_memory => callbacks.send_fn(@intToPtr([*c]const u8, cmd.addr), cmd.size),
            .write_memory => callbacks.recv_fn(@intToPtr([*c]u8, cmd.addr), cmd.size),
            .read_u32 => {
                const result: u32 = @intToPtr(*const volatile u32, cmd.addr).*;
                callbacks.send_fn(@ptrCast([*c]const u8, &result), @sizeOf(u32));
            },
            .write_u32 => @intToPtr(*volatile u32, cmd.addr).* = cmd.size,
            .resolve_addr => {
                var result = arch.resolveAddr(cmd.addr, cmd.size);
                callbacks.send_fn(@ptrCast([*c]const u8, &result), @sizeOf(@TypeOf(result)));
            },
            //else => unreachable,
        }
    }
}
