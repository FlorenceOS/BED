const std = @import("std");

fn calculateChecksum(packet_data: []const u8) u8 {
    var result: u8 = 0;
    for (packet_data) |c| result +%= c;
    return result;
}

fn verifyChecksum(packet_data: []const u8, checksum: u8) !void {
    if (calculateChecksum(packet_data) != checksum)
        return error.BadChecksum;
    return;
}

pub const Stream = struct {
    reader: std.net.Stream.Reader,
    writer: std.net.Stream.Writer,

    pub fn init(stream: std.net.Stream) @This() {
        return .{
            .reader = stream.reader(),
            .writer = stream.writer(),
        };
    }

    pub const ReadSession = struct {};

    pub fn readInto(self: @This(), buffer: []u8) ![]u8 {
        errdefer self.writer.writeByte('-') catch @panic("Couldn't send NAK!");
        var first_byte = try self.reader.readByte();

        while (first_byte != '$') {
            switch (first_byte) {
                '-' => std.log.err("Whoops, must have sent something valid bcuz gdb be mad bro", .{}),
                '+' => {},
                else => std.log.err("Bad packet byte: 0x{X}", .{first_byte}),
            }
            first_byte = try self.reader.readByte();
        }

        var used_size: usize = 0;
        while (true) {
            const b = try self.reader.readByte();

            if (b == '#') {
                const cs = try std.fmt.parseUnsigned(u8, &try self.reader.readBytesNoEof(2), 16);

                const result = buffer[0..used_size];

                try verifyChecksum(result, cs);
                try self.writer.writeByte('+');

                std.log.info("->'{s}'", .{result});
                //std.log.info("<-'+'", .{});

                return result;
            }

            if (used_size == buffer.len) {
                return error.BufferFull;
            }

            buffer[used_size] = if (b == 0x7D) (try self.reader.readByte()) ^ 0x20 else b;
            used_size += 1;
        }
    }

    /// Send `data` as a gdb packet, retries until you probably should treat it as a fatal error.
    pub fn send(self: *@This(), data: []const u8) !void {
        var tries: usize = 0;
        while (tries < 1000) : (tries += 1) {
            var session = try self.startSendSession();
            session.send(data) catch |err| {
                session.trash() catch {}; // Try to trash but don't care if it fails
                return err;
            };
            session.finalize() catch |err| {
                switch (err) {
                    //error.GdbDeniedPacket => continue, // Try sending the packet again
                    else => return err,
                }
            };
            return;
        }
    }

    pub const SendSession = struct {
        stream: *Stream,
        checksum: u8 = 0,

        fn doSendByte(self: *@This(), b: u8) callconv(.Inline) !void {
            try self.stream.writer.writeByte(b);
            self.checksum +%= b;
        }

        /// Add some bytes to this packet
        pub fn send(self: *@This(), data: []const u8) !void {
            std.log.info("<- Continue: '{s}'", .{data});

            for (data) |b| {
                switch (b) {
                    // Check if the byte needs escaping
                    // https://sourceware.org/gdb/onlinedocs/gdb/Overview.html#Binary-Data
                    0x23, 0x24, 0x2A, 0x7D => {
                        try self.doSendByte(0x7D);
                        try self.doSendByte(b ^ 0x20);
                    },
                    else => {
                        try self.doSendByte(b);
                    },
                }
            }
        }

        fn sendChecksum(self: @This(), checksum: u8) callconv(.Inline) !void {
            try self.stream.writer.writeByte('#');
            var checksum_buffer: [2]u8 = undefined;
            _ = std.fmt.bufPrint(&checksum_buffer, "{X:0>2}", .{checksum}) catch unreachable;
            try self.stream.writer.writeAll(&checksum_buffer);
        }

        /// Finish sending a packet
        pub fn finalize(self: @This()) !void {
            try self.sendChecksum(self.checksum);
            std.log.info("<- Finish", .{});

            // We can't do the response reading yet, we need a "reader thread" or something similar to make sure
            // we can grab a byte here when there's another packet waiting, since we have to read that first.

            // const reply = try self.stream.reader.readByte();
            // switch(reply) {
            //     '-' => return error.GdbDeniedPacket,
            //     '+' => return,
            //     else => {
            //         @panic("Bad GDB reply byte!");
            //     },
            // }
        }

        /// Cancel sending a packet, GDB _should_ reject it since it has an invalid checksum
        /// A session is no longer valid after this.
        pub fn trash(self: @This()) !void {
            try self.sendChecksum(~self.checksum); // Send invalid checksum
            std.log.info("<- Cancel", .{});
            // We should recieve a reply here too later
        }
    };

    /// Session to send multiple slices of data in one gdb packet. Sending may fail.
    pub fn startSendSession(self: *@This()) !SendSession {
        std.log.info("<- Start", .{});

        try self.writer.writeByte('$');

        return SendSession{
            .stream = self,
        };
    }
};

// pub fn sendPacket(data: []const u8, reader: *std.net.Stream.Reader, writer: *std.net.Stream.Writer) !void {

//     var tries: usize = 0;

//     while (tries < 10000) : (tries += 1) {
//         try writer.writeByte('$');
//         try writer.writeAll(data);
//         try writer.writeByte('#');
//         try writer.writeAll(&checksum_buffer);

//         std.log.info("<-'{s}'", .{data});
//         _ = reader;
//         return;

//         // const reply = try reader.readByte();
//         // switch(reply) {
//         //     '-' => continue,
//         //     '+' => {
//         //
//         //         return;
//         //     },
//         //     else => unreachable,
//         // }
//     }

//     return error.TooManyRetries;
// }
