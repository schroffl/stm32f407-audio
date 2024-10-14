const std = @import("std");
const Self = @This();

table: []const f32,

pub const Phasor = struct {
    /// The sampling rate is fixed to 48kHz, because I don't need it to be configurable.
    pub const SamplingFrequency: f32 = 47991.07142857143;

    table_size: f32 = 0,
    step_size: f32 = 0.0,
    position: f32 = 0.0,

    pub fn init(table_size: usize, frequency: f32) Phasor {
        var result = Phasor{
            .table_size = @floatFromInt(table_size),
        };

        result.setFrequency(frequency);

        return result;
    }

    pub inline fn setFrequency(self: *Phasor, frequency: f32) void {
        self.step_size = @as(f32, self.table_size) * frequency / SamplingFrequency;
    }

    pub fn normalizedPosition(self: Phasor) f32 {
        return self.position / self.table_size;
    }

    pub inline fn advance(self: *Phasor) void {
        self.position += self.step_size;

        if (self.position >= self.table_size) {
            self.position -= self.table_size;
        }
    }
};

/// Create a phasor pre-configured with the correct table size with the given
/// frequency.
pub fn phasor(self: Self, frequency: f32) Phasor {
    return Phasor.init(self.table.len, frequency);
}

/// Sample the wavetable at the given position. The caller is expected to make
/// sure that the position is not greater than the table length.
pub inline fn sample(self: Self, position: f32) f32 {
    const idx_a = @as(usize, @intFromFloat(@trunc(position))) % self.table.len;
    const idx_b = (idx_a + 1) % self.table.len;

    const a = self.table[idx_a];
    const b = self.table[idx_b];

    const t = position - @trunc(position);

    return a * (1 - t) + b * t;
}

/// Generate a lookup table for a sine with the given amount of samples.
/// Exactly one cycle will be stored in the table. Because this calculation is
/// assumed to be made at compile time, double precision floating point numbers
/// are used instead of single floats.
pub fn generateSine(comptime Size: usize) [Size]f32 {
    @setEvalBranchQuota(1024 * 1024);

    const float_size: f64 = @floatFromInt(Size);
    var out: [Size]f32 = undefined;

    for (0..Size) |i| {
        const float_i: f64 = @floatFromInt(i);
        const t = float_i / float_size;

        const value = std.math.sin(t * std.math.pi * 2);

        out[i] = @floatCast(value);
    }

    return out;
}

/// Generate a lookup table for a square wave with the given amount of samples.
pub fn generateSquare(comptime Size: usize, comptime harmonics: usize) [Size]f32 {
    @setEvalBranchQuota(1024 * harmonics * 100);

    const float_size: f64 = @floatFromInt(Size);
    var out: [Size]f32 = undefined;

    for (0..Size) |i| {
        const float_i: f64 = @floatFromInt(i);
        const t = float_i / float_size;

        var value: f64 = 0.0;

        for (0..harmonics) |k| {
            const float_k: f64 = @floatFromInt(k);

            const two_k_plus_one = 2 * (float_k + 1) - 1;
            value += (1 / two_k_plus_one) * std.math.sin(t * std.math.pi * 2 * two_k_plus_one);
        }

        out[i] = @floatCast(value);
    }

    return out;
}
