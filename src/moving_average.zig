const Self = @This();
const BufferSize = 4;

buffer: [BufferSize]f32 = [_]f32{0} ** BufferSize,
idx: usize = 0,

pub inline fn process(self: *Self, sample: f32) f32 {
    self.buffer[self.idx] = sample;
    self.idx = (self.idx + 1) % self.buffer.len;

    const factor = 1.0 / @as(comptime_float, BufferSize);
    var sum: f32 = 0;

    for (self.buffer) |v| sum += v * factor;

    return sum;
}
