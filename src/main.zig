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
var phasor_frequency = wavetable.phasor(1);
// var phasor_right = wavetable.phasor(10);

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

    // const table = Wavetable{
    //     .samples = [_]f32{0.5} ** 256,
    // };

    // var scanner = Wavetable.Scanner{ .step_size = 1, .position = 153.79 };
    // const value0 = table.sample(scanner);
    // _ = value0;
    // scanner.step();
    // const value2 = table.sample(scanner);
    // _ = value2;

    var i2s = I2S.init();
    std.mem.doNotOptimizeAway(i2s);

    //     for (wavetable.table, 0..) |sample, i| {
    //         if (i > 0) try uart.write_buffer.write(", ");
    //         try uart.write_buffer.writer().print("{d:.5}", .{sample});
    //     }

    uart.ensureTransmission();

    // Configure the MPU6050
    // try I2C.send(0x68, &.{ 0x6b, 0x00 });
    // try I2C.send(0x4a, &.{ 0x00, 0x99 });

    var rand = prng.random();
    _ = rand;

    while (true) {
        // const v = wavetable.sample(phasor_left.position);
        // _ = v;
        // phasor_left.advance();

        // while (regs.SPI3.SR.read().TXE == 0) {}
        // regs.SPI3.DR.modify(.{ .DR = rand.int(u16) });
        // while (regs.SPI3.SR.read().TXE == 0) {}
        // regs.SPI3.DR.modify(.{ .DR = 0 });

        // top_buffer_half = !top_buffer_half;

        // Toggle an LED for debugging purposes
        // regs.GPIOD.BSRR.modify(.{
        //     .BS15 = @as(u1, @bitCast(top_buffer_half)),
        //     .BR15 = @as(u1, @bitCast(!top_buffer_half)),
        // });
    }

    while (true) {
        // microzig.cpu.nop();

        if (uart.read_buffer.readItem()) |_| {
            // uart.write_buffer.writeItem(char) catch unreachable;
            // uart.ensureTransmission();

            var out: [1]u8 = undefined;
            try I2C.send(0x4a, &.{0x2e | 128});
            try I2C.read(0x4a, &out);
            std.log.debug("{b:0<8}", .{out[0]});
        }
    }

    const SensorReading = packed struct {
        x: i16,
        y: i16,
        z: i16,
    };

    while (true) {

        // Tell the MP6050 to dump its contents
        try I2C.send(0x68, &.{0x3b});

        var out: [@bitSizeOf(SensorReading) / 8]u8 = undefined;
        try I2C.read(0x68, &out);

        var reading: SensorReading = @bitCast(out);
        reading.x = @byteSwap(reading.x);
        reading.y = @byteSwap(reading.y);
        reading.z = @byteSwap(reading.z);

        const ax = @as(f32, @floatFromInt(reading.x)) / 16384 * 9.806;
        const ay = @as(f32, @floatFromInt(reading.y)) / 16384 * 9.806;
        const az = @as(f32, @floatFromInt(reading.z)) / 16384 * 9.806;

        var writer = uart.write_buffer.writer();
        try writer.print("{d:.5},{d:.5},{d:.5}\n", .{ ax, ay, az });
        uart.ensureTransmission();

        hackyDelay(250);
    }

    // try I2C.send(0x94 >> 1, &.{ 0, 0 });

    // try i2cTest();

    while (true) microzig.cpu.nop();

    // var idx: usize = 0;

    // while (true) {
    //     if (uart.read_buffer.readItem()) |char| {
    //         uart.write_buffer.writeItem(char) catch unreachable;
    //         uart.ensureTransmission();
    //     }

    //     if (idx > 60000) {
    //         // std.log.debug("Yooo", .{});
    //         idx = 0;
    //     }

    //     idx += 1;

    //     asm volatile ("nop");
    // }
}

var last_sample: f32 = 0.0;
var counter: usize = 0;

fn fillAudioBuffer(out: []u16) void {
    for (0..out.len / 2) |i| {
        const freq = phasor_frequency.normalizedPosition() * 100 + 50;
        _ = freq;
        phasor_frequency.advance();

        const v = wavetable.sample(phasor_left.position);
        // const v2 = wavetable.sample(phasor_right.position);

        if (counter < 256) {
            std.log.debug("{d:.3}", .{phasor_left.position});
            counter += 1;
        }

        out[i * 2 + 0] = @bitCast(I2S.floatToInt(i16, v));
        out[i * 2 + 1] = 0;

        // out[i * 2 + 0] = @bitCast(I2S.floatToInt(i16, v));
        // out[i * 2 + 1] = @bitCast(I2S.floatToInt(i16, v));

        // if (std.math.fabs(v - last_sample) > 0.3) {
        //     std.log.debug("Jump from {d:.3} to {d:.3} at index {}", .{ last_sample, v, i });
        // }
        // last_sample = v;

        // out[i * 4 + 0] = 0xffff;
        // out[i * 4 + 1] = 0xffff;

        // out[i * 4 + 2] = @intCast(vint >> 16);
        // out[i * 4 + 3] = @intCast(vint & 0xffff);

        // out[i * 4 + 0] = 0xdead;
        // out[i * 4 + 1] = 0xbeef;

        // out[i * 4 + 2] = 0xfaad;
        // out[i * 4 + 3] = 0xeeda;

        // out[i * 4 + 0] = @intCast(vint >> 16);
        // out[i * 4 + 1] = @intCast(vint & 0xffff);

        // out[i * 4 + 2] = @intCast(v2int >> 16);
        // out[i * 4 + 3] = @intCast(v2int & 0xffff);

        // HI Left
        // out[i * 4 + 2] = @intCast(v2int >> 16);
        // LO Left
        // out[i * 4 + 2] = @intCast(v2int & 0xffff);

        // out[i * 4 + 1] = @bitCast(I2S.floatToInt(i16, v));

        // phasor_left.setFrequency(freq);
        phasor_left.advance();
        // phasor_right.advance();
    }
}

fn systemInit() void {
    // This init does these things:
    // - Enables the FPU coprocessor
    // - Sets the external oscillator to achieve a clock frequency of 168MHz
    // - Sets the correct PLL prescalers for that clock frequency
    // - Enables the flash data and instruction cache and sets the correct latency for 168MHz

    // Enable FPU coprocessor
    // WARN: currently not supported in qemu, comment if testing it there
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b11 });

    // Enable HSI
    regs.RCC.CR.modify(.{ .HSION = 1 });

    // Wait for HSI ready
    while (regs.RCC.CR.read().HSIRDY != 1) {}

    // Select HSI as clock source
    regs.RCC.CFGR.modify(.{ .SW0 = 0, .SW1 = 0 });

    // Enable external high-speed oscillator (HSE)
    regs.RCC.CR.modify(.{ .HSEON = 1 });

    // Wait for HSE ready
    while (regs.RCC.CR.read().HSERDY != 1) {}

    // Set prescalers for 168 MHz: HPRE = 0, PPRE1 = DIV_2, PPRE2 = DIV_4
    regs.RCC.CFGR.modify(.{ .HPRE = 0, .PPRE1 = 0b101, .PPRE2 = 0b100 });

    // Disable PLL before changing its configuration
    regs.RCC.CR.modify(.{ .PLLON = 0 });

    // Set PLL prescalers and HSE clock source
    // TODO: change the svd to expose prescalers as packed numbers instead of single bits
    regs.RCC.PLLCFGR.modify(.{
        .PLLSRC = 1,
        // PLLM = 8 = 0b001000
        .PLLM0 = 0,
        .PLLM1 = 0,
        .PLLM2 = 0,
        .PLLM3 = 1,
        .PLLM4 = 0,
        .PLLM5 = 0,
        // PLLN = 336 = 0b101010000
        .PLLN0 = 0,
        .PLLN1 = 0,
        .PLLN2 = 0,
        .PLLN3 = 0,
        .PLLN4 = 1,
        .PLLN5 = 0,
        .PLLN6 = 1,
        .PLLN7 = 0,
        .PLLN8 = 1,
        // PLLP = 2 = 0b10
        .PLLP0 = 0,
        .PLLP1 = 1,
        // PLLQ = 7 = 0b111
        .PLLQ0 = 1,
        .PLLQ1 = 1,
        .PLLQ2 = 1,
    });

    // Enable PLL
    regs.RCC.CR.modify(.{ .PLLON = 1 });

    // Wait for PLL ready
    while (regs.RCC.CR.read().PLLRDY != 1) {}

    // Enable flash data and instruction cache and set flash latency to 5 wait states
    regs.FLASH.ACR.modify(.{ .DCEN = 1, .ICEN = 1, .LATENCY = 5 });

    // Select PLL as clock source
    regs.RCC.CFGR.modify(.{ .SW1 = 1, .SW0 = 0 });

    // Wait for PLL selected as clock source
    var cfgr = regs.RCC.CFGR.read();
    while (cfgr.SWS1 != 1 and cfgr.SWS0 != 0) : (cfgr = regs.RCC.CFGR.read()) {}

    // Disable HSI
    regs.RCC.CR.modify(.{ .HSION = 0 });
}
