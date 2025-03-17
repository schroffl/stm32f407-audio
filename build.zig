const std = @import("std");
const microzig = @import("microzig");
const MicroBuild = microzig.MicroBuild(.{});

pub fn build(b: *std.Build) void {
    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;

    const optimize = b.standardOptimizeOption(.{
        .preferred_optimize_mode = .ReleaseSmall,
    });

    const target_query = std.Target.Query{
        .cpu_arch = .thumb,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
        .cpu_features_add = std.Target.arm.featureSet(&.{.vfp4}),
        .os_tag = .freestanding,
        .abi = .eabi,
    };

    const KiB = 1024;
    const target = microzig.Target{
        .dep = mz_dep,
        .preferred_binary_format = .elf,
        .chip = .{
            .name = "STM32F407",
            .cpu = target_query,
            .memory_regions = &.{
                .{ .offset = 0x08000000, .length = 1024 * KiB, .kind = .flash },
                .{ .offset = 0x20000000, .length = 128 * KiB, .kind = .ram },
                .{ .offset = 0x10000000, .length = 64 * KiB, .kind = .ram }, // CCM RAM
            },
            .register_definition = .{
                .svd = b.path("./STM32F407.svd"),
            },
        },
    };

    const firmware = mb.add_firmware(.{
        .name = "my-firmware",
        .target = &target,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    const asm_step = b.addInstallFile(firmware.artifact.getEmittedAsm(), "my-firmware.s");
    b.getInstallStep().dependOn(&asm_step.step);

    mb.install_firmware(firmware, .{ .format = .elf });
    mb.install_firmware(firmware, .{ .format = .bin });

    const stflash_path = b.option([]const u8, "st-flash", "The path to the st-flash binary") orelse "st-flash";
    const flash_cmd = b.addSystemCommand(&.{ stflash_path, "--reset", "--connect-under-reset", "write" });
    flash_cmd.addFileArg(firmware.get_emitted_bin(.bin));
    flash_cmd.addArg("0x8000000");

    const flash_step = b.step("flash", "Flash the binary to the microcontroller");
    flash_step.dependOn(&flash_cmd.step);
}
