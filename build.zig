const std = @import("std");
const mz = @import("microzig/build");

pub fn build(b: *std.Build) void {
    const microzig = mz.init(b, .{});
    const optimize = b.standardOptimizeOption(.{
        .preferred_optimize_mode = .ReleaseSmall,
    });

    // Get the compiler to emit FPU instructions
    const cpu = blk: {
        var cm4 = mz.cpus.cortex_m4;
        cm4.target.cpu_features_add.addFeature(@intFromEnum(std.Target.arm.Feature.vfp4));

        break :blk cm4;
    };

    const KiB = 1024;
    const target = mz.Target{
        .preferred_format = .elf,
        .chip = .{
            .name = "STM32F407",
            .cpu = cpu,
            .memory_regions = &.{
                .{ .offset = 0x08000000, .length = 1024 * KiB, .kind = .flash },
                .{ .offset = 0x20000000, .length = 128 * KiB, .kind = .ram },
                .{ .offset = 0x10000000, .length = 64 * KiB, .kind = .ram }, // CCM RAM
            },
            .register_definition = .{
                .svd = .{ .path = "./STM32F407.svd" },
            },
        },
    };

    const firmware = microzig.add_firmware(b, .{
        .name = "my-firmware",
        .target = target,
        .optimize = optimize,
        .root_source_file = .{ .path = "src/main.zig" },
    });

    const asm_step = b.addInstallFile(firmware.artifact.getEmittedAsm(), "my-firmware.s");
    b.getInstallStep().dependOn(&asm_step.step);

    microzig.install_firmware(b, firmware, .{ .format = .elf });
    microzig.install_firmware(b, firmware, .{ .format = .bin });

    const stflash_path = b.option([]const u8, "st-flash", "The path to the st-flash binary") orelse "st-flash";
    const flash_cmd = b.addSystemCommand(&.{ stflash_path, "--reset", "--connect-under-reset", "write" });
    flash_cmd.addFileArg(firmware.get_emitted_bin(.bin));
    flash_cmd.addArg("0x8000000");

    const flash_step = b.step("flash", "Flash the binary to the microcontroller");
    flash_step.dependOn(&flash_cmd.step);
}
