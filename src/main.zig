const std = @import("std");
const microzig = @import("microzig");
const regs = microzig.chip.peripherals;
const cpu = microzig.cpu;

const UART = @import("./uart.zig");
const I2C = @import("./i2c.zig");
const I2S = @import("./i2s.zig");

const Wavetable = @import("./wavetable.zig");

var uart: UART = undefined;

var sample_index: usize = 0;
var top_buffer_half: bool = false;

var prng = std.rand.DefaultPrng.init(0);

const wavetable = Wavetable{
    .table = Wavetable.generateSine(256)[0..],
};

var phasor_left = wavetable.phasor(187.5 * 1);
var phasor_right = wavetable.phasor(10);

pub const microzig_options = struct {
    pub const interrupts = struct {
        pub fn USART2() void {
            uart.handleInterrupt();
        }

        pub fn DMA1_Stream5() void {
            const flags = regs.DMA1.HISR.read();
            const log = std.log.scoped(.i2s_int);
            _ = log;

            if (flags.DMEIF5 == 1) {
                @panic("DMEIF5 was set");
            }

            // If the half transfer interrupt bit is set, fill the other half
            if (flags.HTIF5 == 1 or flags.TCIF5 == 1) {
                var full = I2S.buffer[0..];
                var out = if (flags.HTIF5 == 1)
                    full[0 .. full.len / 2]
                else
                    full[full.len / 2 ..];

                top_buffer_half = !top_buffer_half;

                fillAudioBuffer(out);
            }

            // Toggle an LED for debugging purposes
            regs.GPIOD.BSRR.modify(.{
                .BS15 = @as(u1, @bitCast(top_buffer_half)),
                .BR15 = @as(u1, @bitCast(!top_buffer_half)),
            });

            // Clear interrupt flags
            regs.DMA1.HIFCR.modify(.{
                .CTEIF5 = 1,
                .CHTIF5 = 1,
                .CTCIF5 = 1,
                .CDMEIF5 = 1,
                .CFEIF5 = 1,
            });
        }
    };
};

pub const std_options = struct {
    pub const log_level = .debug;

    // Uart logger
    pub fn logFn(
        comptime level: std.log.Level,
        comptime scope: @TypeOf(.EnumLiteral),
        comptime format: []const u8,
        args: anytype,
    ) void {
        const level_txt = comptime level.asText();
        const prefix2 = if (scope == .default) ": " else "(" ++ @tagName(scope) ++ "): ";

        var writer = uart.write_buffer.writer();
        writer.print(level_txt ++ prefix2 ++ format ++ "\r\n", args) catch return;
        uart.ensureTransmission();
    }
};

fn hackyDelay(cycles: usize) void {
    var i: usize = 0;
    while (i < cycles) : (i += 1) microzig.cpu.nop();
}

pub fn main() !void {
    // Enable FPU
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b11 });

    // Enable HSI
    regs.RCC.CR.modify(.{ .HSION = 1 });

    // Wait for HSI ready
    while (regs.RCC.CR.read().HSIRDY != 1) {}

    // Turn green LED on
    {
        regs.RCC.AHB1ENR.modify(.{ .GPIODEN = 1 });

        regs.GPIOD.MODER.modify(.{
            .MODER12 = 1,
            .MODER13 = 1,
            .MODER14 = 1,
            .MODER15 = 1,
        });

        regs.GPIOD.BSRR.modify(.{ .BS12 = 1 });
    }

    // Initialize the uart and enable relevant interrupts.
    {
        uart = UART.init(.{ .baud = 115_200 });
        uart.enableInterrupts();
    }

    try uart.write_buffer.write("\r\n=============\r\n");
    uart.ensureTransmission();

    // Reset the CS43 vi PD4
    {
        regs.GPIOD.OTYPER.modify(.{ .OT4 = 0 });
        regs.GPIOD.MODER.modify(.{ .MODER4 = 1 });
        regs.GPIOD.PUPDR.modify(.{ .PUPDR4 = 0b10 });
        regs.GPIOD.BSRR.modify(.{ .BS4 = 1 });
    }

    try I2C.init();

    // Read the chip id and revision number from the CS43L22, which should be 0b1110011
    {
        var out: [3]u8 = undefined;
        try I2C.send(0x4a, &.{0x01 | 128});
        try I2C.read(0x4a, &out);

        const id = out[0] >> 3;
        const rev = out[0] & 0b111;

        std.log.debug("CS43L22: ID {} REVISION {}", .{ id, rev });
    }

    // Startup sequence for the CS43L22 according to its manual
    {
        try I2C.send(0x4a, &.{ 0x99, 0x00 });
        try I2C.send(0x4a, &.{ 0x80, 0x47 });
        try I2C.send(0x4a, &.{ 0x32, 0b1000000 });
        try I2C.send(0x4a, &.{ 0x32, 0b0000000 });
        try I2C.send(0x4a, &.{ 0x00, 0x00 });

        // Configure 16 Bit word length
        try I2C.send(0x4a, &.{ 0x06, 0b11 });

        // Set SCLK = MCLK
        // try I2C.send(0x4a, &.{ 0x06, 0b11 });

        const dsp_mode: u1 = 0;
        const interface_format: u2 = 0b01; // I2S
        const word_length: u2 = 0b11; // 24 bit

        try I2C.send(0x4a, &.{
            0x05,
            ((0 << 7) | (0b01 << 5)),
        });

        try I2C.send(0x4a, &.{
            0x06,
            ((@as(u8, dsp_mode) << 4) | @as(u8, interface_format) << 2) | (@as(u8, word_length) << 0),
        });

        // Power on
        try I2C.send(0x4a, &.{ 0x02, 0x9e });
    }

    var i2s = I2S.init();
    std.mem.doNotOptimizeAway(i2s);

    uart.ensureTransmission();

    while (true) microzig.cpu.nop();
}

var counter: usize = 0;

fn fillAudioBuffer(out: []u16) void {
    for (0..out.len / 2) |i| {
        const v = wavetable.sample(phasor_left.position);
        const v2 = wavetable.sample(phasor_right.position);

        if (counter < 256) {
            std.log.debug("{d:.3}", .{phasor_left.position});
            counter += 1;
        }

        out[i * 2 + 0] = @bitCast(I2S.floatToInt(i16, v));
        out[i * 2 + 1] = @bitCast(I2S.floatToInt(i16, v2));

        phasor_left.advance();
        phasor_right.advance();
    }
}
