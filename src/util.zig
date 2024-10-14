const std = @import("std");

pub inline fn floatToInt(comptime T: type, value: f32) T {
    const clamp_result = false;

    return switch (@typeInfo(T)) {
        .Int => blk: {
            const min = std.math.minInt(T);
            const max = std.math.maxInt(T);

            const normalized = if (clamp_result)
                std.math.clamp(value * 0.5 + 0.5, 0.0, 1.0)
            else
                value * 0.5 + 0.5;

            const mapped: T = @intFromFloat(normalized * (max - min) + min);

            break :blk mapped;
        },
        else => @compileError("Cannot convert float to type " ++ @typeName(T)),
    };
}
