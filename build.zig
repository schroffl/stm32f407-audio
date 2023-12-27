const std = @import("std");
const stm32 = @import("stmicro_stm32");
const mz = @import("microzig");

pub fn build(b: *std.Build) void {
    const microzig = mz.init(b, "microzig");
    const optimize = b.standardOptimizeOption(.{});

    const KiB = 1024;
    const target = mz.Target{
        .preferred_format = .elf,
        .chip = .{
            .name = "STM32F407",
            .cpu = .cortex_m4,
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

    const firmware = microzig.addFirmware(b, .{
        .name = "my-firmware",
        // .target = stm32.boards.stm32f4discovery,
        .target = target,
        .optimize = optimize,
        .source_file = .{ .path = "src/main.zig" },
    });

    microzig.installFirmware(b, firmware, .{});
    microzig.installFirmware(b, firmware, .{ .format = .elf });
    microzig.installFirmware(b, firmware, .{ .format = .bin });

    const stflash_path = b.option([]const u8, "st-flash", "The path to the st-flash binary") orelse "st-flash";
    const flash_cmd = b.addSystemCommand(&.{ stflash_path, "--reset", "write" });
    flash_cmd.addFileArg(firmware.getEmittedBin(.bin));
    flash_cmd.addArg("0x8000000");

    const flash_step = b.step("flash", "Flash the binary to the microcontroller");
    flash_step.dependOn(&flash_cmd.step);
    flash_step.dependOn(b.default_step);
}
