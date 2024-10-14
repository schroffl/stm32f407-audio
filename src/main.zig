const std = @import("std");
const microzig = @import("microzig");
const regs = microzig.chip.peripherals;
const cpu = microzig.cpu;

const util = @import("./util.zig");
const UART = @import("./uart.zig");
const I2C = @import("./i2c.zig");
const I2S = @import("./i2s.zig");

const DWT_CTRL: *volatile u32 = @ptrFromInt(0xE0001000);
const DWT_CYCCNT: *volatile u32 = @ptrFromInt(0xE0001004);
const DEMCR: *volatile u32 = @ptrFromInt(0xE000EDFC);

const Wavetable = @import("./wavetable.zig");
const MovingAverage = @import("./moving_average.zig");

var uart: UART = undefined;

var sample_index: usize = 0;

var prng = std.rand.DefaultPrng.init(0);

const wavetable = Wavetable{
    .table = Wavetable.generateSine(4096)[0..],
};

var phasor_left = wavetable.phasor(500);
var phasor_right = wavetable.phasor(1000);

var modulator = wavetable.phasor(1);

var adc_input = MovingAverage{};

pub const microzig_options = .{
    .interrupts = .{
        .USART2 = .{
            .C = struct {
                pub fn USART2() callconv(.C) void {
                    uart.handleInterrupt();
                }
            }.USART2,
        },

        .DMA1_Stream5 = .{
            .C = struct {
                pub fn DMA1_Stream5() callconv(.C) void {
                    const flags = regs.DMA1.HISR.read();

                    // Toggle an LED for debugging and profiling purposes
                    regs.GPIOD.BSRR.modify(.{
                        .BS13 = 1,
                    });

                    if (flags.TEIF5 == 1) {
                        @panic("TEIE5 was set");
                    } else if (flags.DMEIF5 == 1) {
                        @panic("DMEIF5 was set");
                    }

                    regs.ADC1.CR2.modify(.{ .SWSTART = 1 });

                    while (true) {
                        const val = regs.ADC1.SR.read();

                        if (val.EOC == 1) {
                            break;
                        }
                    }

                    const int_value = regs.ADC1.DR.read().DATA;
                    const max_value = std.math.maxInt(u12);

                    const value_raw = @as(f32, @floatFromInt(int_value)) / max_value;
                    const value = adc_input.process(value_raw);

                    phasor_left.setFrequency(100 + 4900 * value);

                    // If the half transfer interrupt bit is set, fill the other half
                    if (flags.HTIF5 == 1 or flags.TCIF5 == 1) {
                        var full = I2S.buffer[0..];
                        const out = if (flags.HTIF5 == 1)
                            full[0 .. full.len / 2]
                        else
                            full[full.len / 2 ..];

                        fillAudioBuffer(out);
                    }

                    // Clear interrupt flags
                    regs.DMA1.HIFCR.modify(.{
                        .CTEIF5 = 1,
                        .CHTIF5 = 1,
                        .CTCIF5 = 1,
                        .CDMEIF5 = 1,
                        .CFEIF5 = 1,
                    });

                    regs.GPIOD.BSRR.modify(.{
                        .BR13 = 1,
                    });
                }
            }.DMA1_Stream5,
        },
    },

    .log_level = .debug,
    .logFn = struct {
        // Uart logger
        pub fn log(
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
    }.log,
};

fn hackyDelay(cycles: usize) void {
    var i: usize = 0;
    while (i < cycles) : (i += 1) microzig.cpu.nop();
}

/// This configures the CPU to run at 168 MHz
pub fn clockSetup() void {
    regs.FLASH.ACR.modify(.{
        .LATENCY = 0b101,

        // Instruction cache
        .ICEN = 1,

        // Data cache
        .DCEN = 0,

        // Prefetch
        .PRFTEN = 0,
    });

    // Enable HSE clock
    regs.RCC.CR.modify(.{
        .HSEON = 1,
    });

    // Wait for HSE ready
    while (regs.RCC.CR.read().HSERDY != 1) {}

    // This is assuming an HSE input frequency of 8 MHz.
    const PLLM: u6 = 8;
    const PLLN: u9 = 336;
    const PLLQ: u4 = 7;
    const PLLP: u2 = 0; // Divide by 2

    regs.RCC.PLLCFGR.modify(.{
        .PLLSRC = 1,

        .PLLM0 = (PLLM >> 0) & 1,
        .PLLM1 = (PLLM >> 1) & 1,
        .PLLM2 = (PLLM >> 2) & 1,
        .PLLM3 = (PLLM >> 3) & 1,
        .PLLM4 = (PLLM >> 4) & 1,
        .PLLM5 = (PLLM >> 5) & 1,

        .PLLN0 = (PLLN >> 0) & 1,
        .PLLN1 = (PLLN >> 1) & 1,
        .PLLN2 = (PLLN >> 2) & 1,
        .PLLN3 = (PLLN >> 3) & 1,
        .PLLN4 = (PLLN >> 4) & 1,
        .PLLN5 = (PLLN >> 5) & 1,
        .PLLN6 = (PLLN >> 6) & 1,
        .PLLN7 = (PLLN >> 7) & 1,
        .PLLN8 = (PLLN >> 8) & 1,

        .PLLQ0 = (PLLQ >> 0) & 1,
        .PLLQ1 = (PLLQ >> 1) & 1,
        .PLLQ2 = (PLLQ >> 2) & 1,
        .PLLQ3 = (PLLQ >> 3) & 1,

        .PLLP0 = (PLLP >> 0) & 1,
        .PLLP1 = (PLLP >> 1) & 1,
    });

    // Enable PLL clock
    regs.RCC.CR.modify(.{
        .PLLON = 1,
    });

    // Wait for PLL ready
    while (regs.RCC.CR.read().PLLRDY != 1) {}

    // Select PLL as system clock
    regs.RCC.CFGR.modify(.{
        // APB1 clock = System clock / 4
        .PPRE1 = 0b101,

        // APB2 clock = System clock / 2
        .PPRE2 = 0b100,

        // AHB clock = System clock
        .HPRE = 0,

        // Setting SW = 0b10 configures the PLL as the system clock source
        .SW1 = 1,
        .SW0 = 0,
    });

    // Wait until the PLL is properly configured as the system clock
    while (true) {
        const rcc_cfgr = regs.RCC.CFGR.read();
        const sws_value = (@as(u2, @intCast(rcc_cfgr.SWS1)) << 1) | rcc_cfgr.SWS0;

        if (sws_value == 0b10) {
            break;
        }
    }

    // Disable the HSI
    regs.RCC.CR.modify(.{
        .HSION = 0,
    });

    const debug_plli2s_clock = false;

    // Output the PLLI2S clock on MCO2 (PC9) for debugging purposes
    if (debug_plli2s_clock) {
        regs.RCC.AHB1ENR.modify(.{
            .GPIOCEN = 1,
        });

        // Select PLLI2S
        regs.RCC.CFGR.modify(.{
            .MCO2 = 0b10,

            // Divide MCO2 output by 4
            .MCO2PRE = 0b110,
        });

        // Configure PC9 as an output pin
        regs.GPIOC.OTYPER.modify(.{
            .OT9 = 0,
        });

        // Configure PC9 as 'very high speed'
        regs.GPIOC.OSPEEDR.modify(.{
            .OSPEEDR9 = 0b11,
        });

        // Configure alternate function mode for PC9
        regs.GPIOC.MODER.modify(.{
            .MODER9 = 0b10,
        });
    }
}

pub fn main() !void {
    clockSetup();

    // Turn green LED on
    {
        regs.RCC.AHB1ENR.modify(.{
            .GPIODEN = 1,
        });

        regs.GPIOD.MODER.modify(.{
            .MODER12 = 1,
            .MODER13 = 1,
            .MODER14 = 1,
            .MODER15 = 1,
        });

        regs.GPIOD.OSPEEDR.modify(.{
            .OSPEEDR15 = 0b11,
        });

        regs.GPIOD.BSRR.modify(.{ .BS12 = 1 });
    }

    // Enable the cycle counter
    {
        // Enable the use of the DWT
        DEMCR.* = DEMCR.* | 0x01000000;

        // Reset cycle counter to 0
        DWT_CYCCNT.* = 0;

        // Enable the cycle counter
        DWT_CTRL.* = DWT_CTRL.* | 1;
    }

    // Enable FPU
    regs.FPU_CPACR.CPACR.modify(.{ .CP = 0b11 });

    // Initialize the uart and enable relevant interrupts.
    {
        uart = UART.init(.{ .baud = 115_200 });
        uart.enableInterrupts();
    }

    try uart.write_buffer.write("\r\n=============\r\n");
    uart.ensureTransmission();

    {
        const buffer_u8 = std.mem.asBytes(I2S.buffer[0..I2S.buffer.len]);
        _ = buffer_u8;

        // for (0..I2S.buffer.len) |i| {
        //     I2S.buffer[i] = @intCast(i);
        // }

        const sine = comptime Wavetable.generateSquare(I2S.buffer.len / 2, 3);

        for (0..I2S.buffer.len / 2) |i| {
            const sine_float = sine[i];
            const signed_16 = util.floatToInt(i16, sine_float);

            I2S.buffer[i * 2 + 0] = @as(u16, @intCast(i % std.math.maxInt(u8))) * 256;
            I2S.buffer[i * 2 + 1] = @bitCast(signed_16);
            // buffer_u8[i * 2 + 1] = @intCast((i * 2) % std.math.maxInt(u8));
        }
    }

    {
        hackyDelay(16_000_00 * 3);

        prng.seed(@intCast(DWT_CYCCNT.*));
        const value = prng.random().float(f32);

        const start = DWT_CYCCNT.*;
        const result = util.floatToInt(i16, value);
        const end = DWT_CYCCNT.*;

        std.mem.doNotOptimizeAway(result);

        std.log.debug("Took {} cycles", .{end - start});
    }

    const cpuid = regs.SCB.CPUID.read();

    std.log.info("CPUID: {}", .{cpuid});

    const device_id_ptr: *[12]u8 = @ptrFromInt(0x1fff_7a10);
    const device_id = std.mem.readInt(u96, device_id_ptr, .big);
    std.log.debug("Unique device ID: {}", .{device_id});

    // const flash_size_ptr: *u16 = @ptrFromInt(0x1fff_7a22);
    // const flash_size = @as(u32, @intCast(flash_size_ptr.*)) * 1024;
    // std.log.debug("Flash Size: {}", .{std.fmt.fmtIntSizeBin(flash_size)});

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

    // while (true) {
    //     if (uart.read_buffer.readItem()) |_| break;
    // }

    // Startup sequence for the CS43L22 according to its manual
    {
        var initial_0x32: [1]u8 = undefined;
        try I2C.send(0x4a, &.{0x32 | 128});
        try I2C.read(0x4a, &initial_0x32);

        std.log.debug("Initial 0x32: {b:0>8}", .{initial_0x32[0]});

        // try I2C.send(0x4a, &.{ 0x99, 0x00 });
        // try I2C.send(0x4a, &.{ 0x80, 0x47 });
        try I2C.send(0x4a, &.{ 0x00, 0x99 });
        try I2C.send(0x4a, &.{ 0x47, 0x80 });
        try I2C.send(0x4a, &.{ 0x32, 0b1000000 | initial_0x32[0] });
        try I2C.send(0x4a, &.{ 0x32, 0b0111111 & initial_0x32[0] });
        try I2C.send(0x4a, &.{ 0x00, 0x00 });

        // Configure 16 Bit word length
        // try I2C.send(0x4a, &.{ 0x06, 0b11 });

        // Set SCLK = MCLK
        // try I2C.send(0x4a, &.{ 0x06, 0b11 });

        const dsp_mode: u1 = 0;
        const interface_format: u2 = 0b01; // I2S
        const word_length: u2 = 0b11; // 16 bit

        try I2C.send(0x4a, &.{
            0x05,
            ((1 << 7) | (0b01 << 5)),
        });

        try I2C.send(0x4a, &.{
            0x06,
            ((@as(u8, dsp_mode) << 4) | @as(u8, interface_format) << 2) | (@as(u8, word_length) << 0),
        });

        // Power on
        try I2C.send(0x4a, &.{ 0x02, 0x9e });
    }

    {
        regs.RCC.APB2ENR.modify(.{
            .ADC1EN = 1,
        });

        regs.RCC.AHB1ENR.modify(.{
            .GPIOAEN = 1,
        });

        regs.GPIOA.MODER.modify(.{
            .MODER1 = 0b11, // Analog mode
        });

        regs.ADC1.CR2.modify(.{
            .ADON = 0,
        });

        regs.ADC1.SQR3.write(.{
            .SQ1 = 1,
            .SQ2 = 0,
            .SQ3 = 0,
            .SQ4 = 0,
            .SQ5 = 0,
            .SQ6 = 0,
            .padding = 0,
        });

        // regs.GPIOA.OTYPER.modify(.{ .OT4 = 0 });
        // regs.GPIOA.MODER.modify(.{ .MODER4 = 1 });
        // regs.GPIOA.PUPDR.modify(.{ .PUPDR4 = 0b10 });
        // regs.GPIOA.BSRR.modify(.{ .BS4 = 1 });

        regs.ADC1.CR2.modify(.{
            .ADON = 1,
        });
    }

    I2S.init();

    while (true) {
        // regs.ADC1.CR2.modify(.{ .SWSTART = 1 });

        // while (true) {
        //     const val = regs.ADC1.SR.read();

        //     if (val.EOC == 1) {
        //         break;
        //     }
        // }

        // const int_value = regs.ADC1.DR.read().DATA;
        // const max_value = std.math.maxInt(u12);

        // const value = @as(f32, @floatFromInt(int_value)) / max_value;

        // phasor_left.setFrequency(0 + 10000 * value);

        // std.log.debug("{d:.6}", .{value});

        // hackyDelay(1_000_00);

        microzig.cpu.nop();
    }
}

var counter: usize = 0;

inline fn fillAudioBuffer(out: []u16) void {
    for (0..out.len / 2) |i| {
        // phasor_left.setFrequency(2000 + 2000 * wavetable.sample(modulator.position));
        // modulator.advance();

        const f1 = wavetable.sample(phasor_left.position);
        const f2 = wavetable.sample(phasor_right.position);

        out[i * 2 + 0] = @bitCast(util.floatToInt(i16, f1));
        out[i * 2 + 1] = @bitCast(util.floatToInt(i16, f2));

        phasor_left.advance();
        phasor_right.advance();
    }
}
