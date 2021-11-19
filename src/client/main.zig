const proto = @import("proto");
const std = @import("std");
const client = @import("client.zig");

fn parsePort(str: ?[:0]const u8) !u16 {
    if (str) |s|
        return try std.fmt.parseUnsigned(u16, s, 10);
    return 1337;
}

pub fn main() !void {
    var arg_it = std.process.args().inner;
    _ = arg_it.skip();

    const port_arg = arg_it.next();
    const port = try parsePort(port_arg);

    try client.run(std.net.Address.initIp4([4]u8{ 0, 0, 0, 0 }, port));
}
