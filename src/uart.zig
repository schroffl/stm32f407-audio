//! Wrapper for USART2, which uses PA2 for transmitting and PA3 for receiving.
//! The implementation is interrupt based and uses a FIFO for reading and
//! writing.

const std = @import("std");
const microzig = @import("microzig");
const regs = microzig.chip.peripherals;

const Self = @This();

pub const Options = struct {
    baud: usize = 9600,
    apb1_clock_speed: usize = 42_000_000,
};

const BufferSize = 4096;

read_buffer: std.fifo.LinearFifo(u8, .{ .Static = BufferSize }),
write_buffer: std.fifo.LinearFifo(u8, .{ .Static = BufferSize }),

/// This functions takes care of initializing USART2 for TX and RX.
/// It also enables the USART2 interrupt in the NVIC.
pub fn init(options: Options) Self {
    var self: Self = undefined;
    self.read_buffer = @TypeOf(self.read_buffer).init();
    self.write_buffer = @TypeOf(self.write_buffer).init();

    // Enable the GPIO clock for Port A, which contains the USART2 Pins (PA2 TX, PA3 RX)
    regs.RCC.AHB1ENR.modify(.{ .GPIOAEN = 1 });

    // Enable USART2 Clock
    regs.RCC.APB1ENR.modify(.{ .USART2EN = 1 });

    // Set PA2 and PA3 to alternate function
    regs.GPIOA.MODER.modify(.{
        .MODER2 = 0b10,
        .MODER3 = 0b10,
    });

    // PA2 and PA3 alternate functions are configured via AFRL.
    // Set them to AF7 (see page 62 in the data sheet).
    regs.GPIOA.AFRL.modify(.{
        .AFRL2 = 7,
        .AFRL3 = 7,
    });

    // Configure the baud rate
    const usartdiv = @as(u16, @intCast(@divTrunc(options.apb1_clock_speed, options.baud)));
    regs.USART2.BRR.raw = usartdiv;

    // Enable USART, transmitter and receiver
    regs.USART2.CR1.modify(.{
        .UE = 1,
        .TE = 1,
        .RE = 1,
    });

    return self;
}

pub fn enableInterrupts(self: *Self) void {
    _ = self;

    // Enable USART2 interrupt in the NVIC. This assumes that the caller has
    // properly configure the interrupt.
    {
        const current = regs.NVIC.ISER1.read().SETENA;

        regs.NVIC.ISER1.modify(.{
            .SETENA = current | (1 << 6),
        });
    }

    // Enable the RXNE interrupt. The TXE interrupt stays disabled for now. It
    // is enabled on demand when data needs to be transmitted.
    regs.USART2.CR1.modify(.{
        .TXEIE = 0,
        .RXNEIE = 1,
    });
}

/// Handle the USART2 global interrupt.
pub fn handleInterrupt(self: *Self) void {
    // The current state of the status register.
    const status = regs.USART2.SR.read();

    // Data was received.
    if (status.RXNE == 1) {
        const value = regs.USART2.DR.read();
        const char: u8 = @intCast(value.DR);

        // Write the character to the read buffer
        self.read_buffer.writeItem(char) catch unreachable;
    }

    // We are clear to transmit the next character.
    if (status.TXE == 1) {
        if (self.write_buffer.readItem()) |next| {
            // If we have a character, write it to the data register.
            regs.USART2.DR.modify(.{ .DR = next });
        } else {
            // Otherwise we can disable the TXE interrupt until
            // new data has to be transmitted.
            regs.USART2.CR1.modify(.{ .TXEIE = 0 });
        }
    }
}

/// This function makes sure that the current write_buffer will be flushed.
pub fn ensureTransmission(self: Self) void {
    _ = self;
    regs.USART2.CR1.modify(.{ .TXEIE = 1 });
}
