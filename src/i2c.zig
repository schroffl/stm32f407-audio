//! A simple wrapper for I2C communication over I2C1
//!
//! TODO: Rewrite this using interrupts, both as an excercise and
//!       a performance improvement.

const std = @import("std");
const microzig = @import("microzig");
const regs = microzig.chip.peripherals;

const log = std.log.scoped(.i2c);

pub const ReadError = error{
    OutOfMemory,
};

pub fn init() !void {
    // log.debug("==== Beginning I2C setup ====", .{});

    // Setup for the GPIO pins
    {
        // Enable GPIOB clock
        regs.RCC.AHB1ENR.modify(.{
            .GPIOBEN = 1,
        });

        // Set PB6 and PB9 to alternate function
        regs.GPIOB.MODER.modify(.{
            .MODER6 = 0b10,
            .MODER9 = 0b10,
        });

        // PB6 to alternate functions is configured via AFRL.
        // Set them to AF4 (see page 63 in the data sheet).
        regs.GPIOB.AFRL.modify(.{
            .AFRL6 = 4,
        });

        // Set PB9 to alternate function
        regs.GPIOB.AFRH.modify(.{
            .AFRH9 = 4,
        });

        // Select open drain output for both pins
        regs.GPIOB.OTYPER.modify(.{
            .OT6 = 1,
            .OT9 = 1,
        });

        // Configure high speed (2, see section 8.4.3 in the reference manual) for
        // both ports.
        regs.GPIOB.OSPEEDR.modify(.{
            .OSPEEDR6 = 2,
            .OSPEEDR9 = 2,
        });

        // Configure pull up resistors for both pins. RM (reference manual) section 7.4.4
        regs.GPIOB.PUPDR.modify(.{
            .PUPDR6 = 1,
            .PUPDR9 = 1,
        });

        // Enable the I2C1 clock
        regs.RCC.APB1ENR.modify(.{
            .I2C1EN = 1,
        });
    }

    // I2C peripheral setup
    {
        // Reset the peripheral
        {
            regs.I2C1.CR1.modify(.{ .SWRST = 1 });
            regs.I2C1.CR1.modify(.{ .SWRST = 0 });
        }

        // TODO make clock speed and target speed configurable
        const clock_speed_mhz = 16;
        const target_speed = 100_000;

        // Specify the APB1 clock speed in MHz
        regs.I2C1.CR2.modify(.{
            .FREQ = clock_speed_mhz,
        });

        // TODO Verify this
        regs.I2C1.CCR.modify(.{
            .CCR = @as(u12, @intCast(@divFloor(clock_speed_mhz * 1_000_000, target_speed * 2))),
            .F_S = 0,
        });

        // Configure the TRISE time, which apparently is just the clock speed in mhz + 1
        regs.I2C1.TRISE.modify(.{
            .TRISE = clock_speed_mhz + 1,
        });

        // Enable the I2C1 peripheral
        regs.I2C1.CR1.modify(.{
            .PE = 1,
        });
    }

    // log.debug("I2C setup complete:\r\n  CR2: {}\r\n  CCR: {}\r\n  TRISE: {}\r\n======", .{
    //     regs.I2C1.CR2.read(),
    //     regs.I2C1.CCR.read(),
    //     regs.I2C1.TRISE.read(),
    // });
}

pub fn send(address: u7, buffer: []const u8) !void {
    // log.debug("Starting I2C write", .{});

    // Wait for the bus to be free
    while (regs.I2C1.SR2.read().BUSY == 1) {}

    // log.debug("bus is free, sending start condition", .{});

    // Send start condition
    regs.I2C1.CR1.modify(.{ .START = 1 });

    // Wait until
    //
    //  * The start condition was generated
    //  * Master mode is selected
    //
    while (true) {
        const sr1 = regs.I2C1.SR1.read();
        const sr2 = regs.I2C1.SR2.read();

        if (sr1.SB == 1 and sr2.MSL == 1) {
            break;
        }
    }

    // log.debug("Sending address: {b}", .{address});

    regs.I2C1.DR.modify(.{
        .DR = @as(u8, @intCast(address)) << 1,
    });

    // Wait until a device acknowledges the address.
    while (regs.I2C1.SR1.read().ADDR == 0) {}

    // log.debug("Address acknowledged", .{});

    // Clear the ADDR bit, which is described in section 27.6.6
    // ("I2C Status Register 1") in the reference manual.
    std.mem.doNotOptimizeAway(regs.I2C1.SR2.read());

    for (buffer) |b| {
        // log.debug("Sending {}", .{b});

        // Write data byte
        regs.I2C1.DR.modify(.{ .DR = b });

        // Wait for transfer finished
        while (regs.I2C1.SR1.read().BTF == 0) {}
    }

    // log.debug("Sending stop condition", .{});

    // Generate the stop condition
    regs.I2C1.CR1.modify(.{ .STOP = 1 });
    while (regs.I2C1.SR2.read().BUSY == 1) {}

    // log.debug("Finished", .{});
}

pub fn read(address: u7, buffer: []u8) ReadError!void {
    // log.debug("Starting I2C read", .{});

    // Wait for the bus to be free
    while (regs.I2C1.SR2.read().BUSY == 1) {}

    // Send start condition
    regs.I2C1.CR1.modify(.{
        .START = 1,
        .ACK = 1,
    });

    // Wait until
    //
    //  * The start condition was generated
    //  * Master mode is selected
    //
    while (true) {
        const sr1 = regs.I2C1.SR1.read();
        const sr2 = regs.I2C1.SR2.read();

        if (sr1.SB == 1 and sr2.MSL == 1) {
            break;
        }
    }

    // log.debug("Sending address: {b}", .{address});

    regs.I2C1.DR.modify(.{
        .DR = (@as(u8, @intCast(address)) << 1) | 1,
    });

    // Wait until a device acknowledges the address.
    while (regs.I2C1.SR1.read().ADDR == 0) {}

    // Clear the ADDR bit, which is described in section 27.6.6
    // ("I2C Status Register 1") in the reference manual.
    std.mem.doNotOptimizeAway(regs.I2C1.SR2.read());

    for (buffer, 0..) |*b, i| {
        const is_last = i == buffer.len - 1;

        // Only send an ACK until the last byte, which will be terminated with
        // the stop condition.
        if (is_last) {
            regs.I2C1.CR1.modify(.{
                .ACK = 0,
            });
        }

        // Wait for a byte to be received
        while (regs.I2C1.SR1.read().RxNE == 0) {}

        // Write data byte
        b.* = regs.I2C1.DR.read().DR;
    }

    // log.debug("Sending stop condition", .{});

    // Generate the stop condition
    regs.I2C1.CR1.modify(.{ .STOP = 1 });
    while (regs.I2C1.SR2.read().BUSY == 1) {}

    // log.debug("Finished", .{});
}
