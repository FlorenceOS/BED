const platform = @import("platform");
const proto = @import("proto");
const gdb = @import("gdb");
const std = @import("std");

pub const RW = enum { r, w };

pub var device_reader: std.fs.File.Reader = undefined;
pub var device_writer: std.fs.File.Writer = undefined;

/// When single stepping, you may need to hook some addresses.
/// Call this with every address that could possibly be reached.
pub fn singleStepHook(addr: u64, current_instr: platform.HookedInstr) !void {
    std.log.info("Hooking at 0x{X} for single stepping", .{addr});
    try singleStepHooks.append(std.heap.page_allocator, .{
        .replaced_bytes = current_instr,
        .addr = addr,
    });
}

/// Send a command to the device. You may be expected to recieve
/// or send additional data after this operation.
pub fn sendDeviceCommand(cmd: proto.Command) !void {
    try device_writer.writeIntLittle(u32, @intCast(u32, @sizeOf(proto.Command)));
    std.log.info("Sent device command: {any}", .{cmd});
    try device_writer.writeAll(std.mem.asBytes(&cmd));
}

/// Start a memory write operation.
/// You're expected to write `len` bytes to `device_writer` after this.
pub fn startMemoryWrite(addr: u64, len: u32) !void {
    try sendDeviceCommand(.{
        .command_type = .write_memory,
        .addr = addr,
        .size = len,
    });
    try device_writer.writeIntLittle(u32, len);
}

/// Start a memory read operation.
/// You're expected to read `len` bytes from `device_reader` after this.
pub fn startMemoryRead(addr: u64, len: u32) !void {
    try sendDeviceCommand(.{
        .command_type = .read_memory,
        .addr = addr,
        .size = len,
    });
}

/// Resume the execution on the remote device. You are expected to send a frame back to the device.
pub fn resumeExecution() !void {
    try sendDeviceCommand(.{
        .command_type = .resume_execution,
    });
}

/// Write the bytes in `data` at address `addr`
pub fn writeMemory(addr: u64, data: []const u8) !void {
    try startMemoryWrite(addr, @intCast(u32, data.len));
    for (data) |b| {
        try device_writer.writeByte(b);
    }
}

/// Read `num_bytes` at `addr` and return them by value
pub fn readBytes(addr: u64, comptime num_bytes: usize) ![num_bytes]u8 {
    var result: [num_bytes]u8 = undefined;
    try readBytesInto(addr, &result);
    return result;
}

/// Fill `buf` with bytes from address `addr`
pub fn readBytesInto(addr: u64, buf: []u8) !void {
    try startMemoryRead(addr, @intCast(u32, buf.len));
    for (buf) |*b| {
        b.* = try device_reader.readByte();
    }
}

/// Check if a hook is placed at a given address `addr`
pub fn hasStepHookAtAddr(addr: u64) bool {
    for (singleStepHooks.items) |hook| {
        if (hook.addr == addr)
            return true;
    }
    return false;
}

var frame: ?platform.Frame = undefined;

const Hook = struct {
    addr: u64,
    replaced_bytes: platform.HookedInstr,
};

var singleStepHooks: std.ArrayListUnmanaged(Hook) = .{};

const minified_target_xml = blk: {
    @setEvalBranchQuota(99999999);
    comptime var result: [:0]const u8 = "";
    comptime var had_newline: bool = false;

    inline for (platform.target_xml) |c| {
        // Very simple yet effective minification:
        // After newlines we remove any spaces and newlines
        // until the next non-space or non-newline char
        const curr_str = [_]u8{c};
        if (had_newline) {
            switch (c) {
                ' ',
                '\n',
                => {},
                else => {
                    result = result ++ curr_str;
                    had_newline = false;
                },
            }
        } else {
            if (c == '\n') {
                had_newline = true;
            } else {
                result = result ++ curr_str;
            }
        }
    }

    break :blk result;
};

fn newFrame() !platform.Frame {
    const next_frame = platform.readFrame();
    for (singleStepHooks.items) |hook| {
        std.log.info("Clearing hook at 0x{X}", .{hook.addr});
        try writeMemory(hook.addr, &hook.replaced_bytes);
    }
    singleStepHooks.clearRetainingCapacity();
    std.log.info("Read frame: {any}", .{next_frame});
    return next_frame;
}

fn sendRegisters(gdb_stream: *gdb.Stream, first_register: usize, last_register: usize) !void {
    var tries: usize = 0;
    while (tries < 10) : (tries += 1) {
        var session = try gdb_stream.startSendSession();

        var current_register = first_register;
        while (true) : (current_register += 1) {
            if (frame.?.getGdbReg(current_register)) |regval| {
                var reg_buf: [16]u8 = undefined;
                _ = try std.fmt.bufPrint(&reg_buf, "{X:0>16}", .{std.mem.nativeToBig(u64, regval.*)});
                try session.send(&reg_buf);
            } else {
                try session.send("x" ** 16);
            }

            if (current_register == last_register)
                break;
        }

        session.finalize() catch |err| {
            switch (err) {
                // error.GdbDeniedPacket => continue,
                else => return err,
            }
        };

        break;
    }
}

// fn checkAddrRange(addr: u64, size: u64, access: RW) void {
//     // @TODO: All of this

//     var mmio_register_mode: usize = 1;
//     if(frame.?.pagingEnabled()) {
//         const page_size = frame.?.pageSize(addr);
//         var curr_addr = @divTrunc(addr, page_size);

//         while(curr_addr < addr + size) : (curr_addr += page_size) {
//             const perms = frame.?.permissionsAtVaddr(addr);
//             switch(access) {
//                 .read => if(!perms.allowsReading()) return .AccessDenied,
//                 .write => if(!perms.allowsWriting()) return .AccessDenied,
//             }
//             mmio_register_mode = std.math.max(mmio_register_mode, perms.register_mode);
//         }
//     } else {
//         if(addr > 0x40000000) { // Probably the highest addr you'll access without paging enabled
//             return .AccessDenied;
//         }
//     }
//     return switch(mmio_register_mode) {
//         1 => .AccessNormally,
//         4 => .AccessAsVolatileU32,
//         else => {
//             std.log.crit("Unimplemented register mode: {d}", .{mmio_register_mode});
//             @panic("");
//         },
//     };
// }

fn handlePacket(pkt: []const u8, gdb_stream: *gdb.Stream) !void {
    if (frame == null) {
        frame = try newFrame();
        std.log.info("Got frame from device: {any}", .{frame});
    }

    // This becomes unlegible if packed tightly
    // zig fmt: off

    if (std.mem.eql(u8, pkt, "vMustReplyEmpty")) {
        try gdb_stream.send("");
    }

    // Stop reason packet
    else if (std.mem.eql(u8, pkt, "?")) {
        try frame.?.informHalt(gdb_stream);
    }

    // General query packets
    else if (std.mem.startsWith(u8, pkt, "qSupported")) {
        // 1M max packet size
        try gdb_stream.send("PacketSize=1048576;qXfer:features:read+;swbreak+;hwbreak+");
    }
    else if(std.mem.startsWith(u8, pkt, "qXfer:features:read:target.xml:")) { // Read target xml
        const request = pkt[31..]; // We cut off the 31 chars above
        const comma_idx = std.mem.indexOfScalar(u8, request, ',').?;
        const offset = try std.fmt.parseUnsigned(usize, request[0..comma_idx], 16);
        const max_size = try std.fmt.parseUnsigned(usize, request[comma_idx + 1..], 16);

        const data_head = minified_target_xml[offset..]; // Send minified XML
        //const data_head = platform.target_xml[offset..]; // Send XML source
        const read_size = std.math.min(data_head.len, max_size);

        var session = try gdb_stream.startSendSession();

        // What does "l" and "m" stand for? "l"ast and "m"ore?
        try session.send(if(read_size == data_head.len) "l" else "m");
        try session.send(data_head[0..read_size]);
        try session.finalize();
    }
    else if (std.mem.startsWith(u8, pkt, "qAttached")) {
        // Existing process
        try gdb_stream.send("1");
    }
    else if (std.mem.eql(u8, pkt, "qTStatus")) {
        try gdb_stream.send("");
    }
    else if (std.mem.eql(u8, pkt, "qfThreadInfo")) {
        // Get a list of active threads
        try gdb_stream.send("");
    }
    else if (std.mem.startsWith(u8, pkt, "qL")) {
        // Thread information, just junk
        try gdb_stream.send("");
    }
    else if (std.mem.startsWith(u8, pkt, "qC")) {
        // Get current thread ID
        try gdb_stream.send("QC 1");
    }

    // Set thread for operation
    else if (pkt.len == 3 and pkt[0] == 'H' and pkt[2] == '0') { // Hg0 et al
        try gdb_stream.send("OK");
    }
    else if (std.mem.startsWith(u8, pkt, "Hc")) {
        try gdb_stream.send("");
    }

    // Detach
    else if (std.mem.startsWith(u8, pkt, "D")) {
        try gdb_stream.send("OK");
    }

    else if(std.mem.startsWith(u8, pkt, "m") or std.mem.startsWith(u8, pkt, "x")) { // Read memory
        const reply_in_binary = pkt[0] == 'x';

        const comma_idx = std.mem.indexOfScalar(u8, pkt, ',').?;
        const addr = try std.fmt.parseUnsigned(usize, pkt[1..comma_idx], 16);
        const size = try std.fmt.parseUnsigned(usize, pkt[comma_idx + 1 ..], 16);

        // Assume the entire read is within the same page for now

        const read_addr = frame.?.operableAddr(addr, .r) catch |err| {
            switch(err) {
                error.CannotWrite => unreachable,
                error.NotMapped => {
                    std.log.warn("vaddr 0x{X} in debuggee isn't mapped", .{addr});
                    try gdb_stream.send("E14"); // EFAULT, Bad address
                },
                error.PaddrNotIdentityMapped => {
                    std.log.warn("paddr behind vaddr 0x{X} isn't accessible by the debugger", .{addr});
                    try gdb_stream.send("E14"); // EFAULT, Bad address
                },
                else => |e| return e,
            }
            return;
        };

        var tries: usize = 0;
        while(tries < 5) {
            try startMemoryRead(read_addr, @intCast(u32, size));
            var session = try gdb_stream.startSendSession();

            var i: usize = 0;
            while(i < size) : (i += 1) {
                if(reply_in_binary) {
                    try session.send(&[_]u8{try device_reader.readByte()});
                } else {
                    var buf: [2]u8 = undefined;
                    _ = std.fmt.bufPrint(&buf, "{X:0>2}", .{try device_reader.readByte()}) catch unreachable;
                    try session.send(&buf);
                }
            }

            session.finalize() catch |err| {
                switch(err) {
                    // error.GdbDeniedPacket => continue,
                    else => return err,
                }
            };

            break;
        }
    }

    else if(std.mem.startsWith(u8, pkt, "M") or std.mem.startsWith(u8, pkt, "X")) { // Write memory
        // @TODO: Sometimes GDB uses this to write breakpoints
        // on aarch64 by writing a BRK #0 instruction. This only works
        // if we're debugging the current EL and we should do something
        // about it if that is what they're doing.

        const read_in_binary = pkt[0] == 'X';

        const comma_idx = std.mem.indexOfScalar(u8, pkt, ',').?;
        const colon_idx = std.mem.indexOfScalar(u8, pkt, ':').?;
        const addr = try std.fmt.parseUnsigned(usize, pkt[1..comma_idx], 16);
        const size = try std.fmt.parseUnsigned(usize, pkt[comma_idx + 1 .. colon_idx], 16);
        const data = pkt[colon_idx + 1..];

        // Assume the entire write is within the same page for now

        const write_addr = frame.?.operableAddr(addr, .w) catch |err| {
            switch(err) {
                error.CannotWrite => {
                    std.log.warn("paddr behind vaddr 0x{X} isn't writeable by the debugger", .{addr});
                    try gdb_stream.send("E01"); // EPERM, Operation not permitted
                },
                error.NotMapped => {
                    std.log.warn("vaddr 0x{X} in debuggee isn't mapped", .{addr});
                    try gdb_stream.send("E14"); // EFAULT, Bad address
                },
                error.PaddrNotIdentityMapped => {
                    std.log.warn("paddr behind vaddr 0x{X} isn't accessible by the debugger", .{addr});
                    try gdb_stream.send("E14"); // EFAULT, Bad address
                },
                else => |e| return e,
            }
            return;
        };

        try startMemoryWrite(write_addr, @intCast(u32, size));

        var i: usize = 0;
        if(read_in_binary) {
            std.debug.assert(size == data.len);
            for(data) |b| {
                try device_writer.writeByte(b);
            }
        } else {
            std.debug.assert(size == @divExact(data.len, 2));
            while(i < size * 2) : (i += 2) {
                const b = std.fmt.parseUnsigned(u8, data[i..][0..2], 16) catch |err| blk: {
                    std.log.crit("Cannot parse hex encoded byte '{s}' in M packet. Error: {}!", .{data[i..][0..2], err});
                    break :blk 0xAA;
                };
                try device_writer.writeByte(b);
            }
        }
        try gdb_stream.send("");
    }

    else if(std.mem.eql(u8, pkt, "g")) { // Read registers list
        try sendRegisters(gdb_stream, 0, platform.num_default_registers);
    }

    else if(std.mem.startsWith(u8, pkt, "p")) { // Read single register
        const regnum = try std.fmt.parseUnsigned(usize, pkt[1..], 16);
        try sendRegisters(gdb_stream, regnum, regnum);
    }

    else if (std.mem.startsWith(u8, pkt, "c") or std.mem.startsWith(u8, pkt, "s")) { // Continue and step
        try frame.?.doStep(std.mem.startsWith(u8, pkt, "s"));

        frame = try newFrame();
        std.log.info("Got frame from device: {any}", .{frame});
        try frame.?.informHalt(gdb_stream);
    }

    else {
        std.log.info("Unknown gdb packet: '{s}', replying empty.", .{pkt});
        try gdb_stream.send("");
    }
    // zig fmt: on
}

/// Reads packets from a gdb instance, processing them one by one
fn processGdbStream(stream: *gdb.Stream) !void {
    var packet_buffer: [4096]u8 = undefined;
    while (true) {
        const pkt = try stream.readInto(&packet_buffer);
        try handlePacket(pkt, stream);
    }
}

/// Starts a gdb server and starts listening for connections
pub fn run(listen_address: std.net.Address) !void {
    device_writer = std.io.getStdOut().writer();
    device_reader = std.io.getStdIn().reader();

    var server = std.net.StreamServer.init(.{
        .kernel_backlog = 0,
        .reuse_address = true,
    });

    try server.listen(listen_address);
    defer server.close();

    std.log.info("Listening on {}", .{listen_address});

    while (true) {
        const conn = try server.accept();
        defer conn.stream.close();

        std.log.info("Accepted connection from {}", .{conn.address});

        var gdb_stream = gdb.Stream.init(conn.stream);

        processGdbStream(&gdb_stream) catch |err| {
            switch (err) {
                error.EndOfStream => continue,
                else => |e| return e,
            }
        };

        std.log.info("Lost connection from {}", .{conn.address});
    }
}
