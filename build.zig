const std = @import("std");

const proto_pkg = std.build.Pkg{
    .name = "proto",
    .path = .{ .path = "src/proto/proto.zig" },
    .dependencies = &[_]std.build.Pkg{},
};

const gdb_pkg = std.build.Pkg{
    .name = "gdb",
    .path = .{ .path = "src/lib/gdb.zig" },
    .dependencies = &[_]std.build.Pkg{},
};

const host_pkg = std.build.Pkg{
    .name = "host",
    .path = .{ .path = "src/host/host.zig" },
    .dependencies = &[_]std.build.Pkg{
        proto_pkg,
    },
};

fn buildArmHostEL(b: *std.build.Builder, el: usize) !void {
    const exec = b.addExecutable(b.fmt("host_aarch64_EL{d}", .{el}), "src/host/arch/aarch64/main.zig");
    const options = b.addOptions();
    options.addOption(usize, "debugger_el", el);
    exec.addPackage(options.getPackage("build_options"));
    exec.code_model = .tiny;
    exec.setLinkerScriptPath(.{.path = "src/host/arch/aarch64/linker.ld"});
    exec.setMainPkgPath("src/");
    exec.setBuildMode(.ReleaseSmall);
    exec.addPackage(host_pkg);
    exec.addPackage(proto_pkg);

    var disabled_features = std.Target.Cpu.Feature.Set.empty;
    var enabled_feautres = std.Target.Cpu.Feature.Set.empty;

    {
        const features = std.Target.aarch64.Feature;
        disabled_features.addFeature(@enumToInt(features.fp_armv8));
        disabled_features.addFeature(@enumToInt(features.crypto));
        disabled_features.addFeature(@enumToInt(features.neon));
    }

    exec.setTarget(.{
        .cpu_arch = .aarch64,
        .os_tag = .freestanding,
        .abi = .none,
        .cpu_features_sub = disabled_features,
        .cpu_features_add = enabled_feautres,
    });

    exec.install();

    const blob_step = exec.installRaw(b.fmt("host_aarch64_EL{d}.bin", .{el}), .{
        .format = .bin,
        .only_section_name = ".blob", 
        .pad_to_size = 0x1000,
    });

    b.default_step.dependOn(&exec.install_step.?.step);
    b.default_step.dependOn(&blob_step.step);

    const build_step = b.step(b.fmt("host-aarch64-EL{d}", .{el}), b.fmt("Build the aarch64 host for EL{d}", .{el}));
    build_step.dependOn(&blob_step.step);
}

fn buildClient(b: *std.build.Builder, target_arch: std.Target.Cpu.Arch) !void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();

    const platform_pkg = std.build.Pkg{
        .name = "platform",
        .path = .{ .path = b.fmt("src/client/{s}.zig", .{@tagName(target_arch)}) },
        .dependencies = &[_]std.build.Pkg{
            gdb_pkg,
            proto_pkg,
        },
    };

    const client = b.addExecutable(
        b.fmt("client_{s}", .{@tagName(target_arch)}),
        "src/client/main.zig",
    );

    client.setTarget(target);
    client.setBuildMode(mode);
    client.install();

    client.addPackage(gdb_pkg);
    client.addPackage(proto_pkg);
    client.addPackage(platform_pkg);

    b.default_step.dependOn(&client.step);

    const run_cmd = client.run();
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step(b.fmt("run-{s}", .{@tagName(target_arch)}), b.fmt("Run the {s} client", .{@tagName(target_arch)}));
    run_step.dependOn(&run_cmd.step);
}

fn buildArm(b: *std.build.Builder) !void {
    try buildArmHostEL(b, 1);
    try buildArmHostEL(b, 2);
    try buildArmHostEL(b, 3);
    try buildClient(b, .aarch64);
}

pub fn build(b: *std.build.Builder) !void {
    try buildArm(b);
}
