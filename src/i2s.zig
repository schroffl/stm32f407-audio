const std = @import("std");
const microzig = @import("microzig");
const regs = microzig.chip.peripherals;

const log = std.log.scoped(.i2s);

// pub var buffer: [1024]u16 align(@sizeOf(u16)) = undefined;

const BufferSize = 128;

pub var buffer: [BufferSize * 4]u16 align(4) = undefined;

pub fn init() void {
    log.debug("Beginning I2S intialization", .{});

    // Enable the SPI3 clock
    regs.RCC.APB1ENR.modify(.{
        .SPI3EN = 1,
    });

    // I2S clock setup
    {
        // Set the I2S clock source to PLLI2S
        regs.RCC.CFGR.modify(.{
            .I2SSRC = 0,
        });

        // These values are pulled from Table 127 (section 28.4.4) in the
        // reference manual. They are assuming a PLL input of 1 MHz, which is
        // achieved by setting RCC.PLLCFGR.PLLM to the value of the input
        // oscillator.
        regs.RCC.PLLI2SCFGR.modify(.{
            .PLLI2SNx = 258,
            .PLLI2SRx = 3,
        });

        // I2SDIV and ODD are also copied from Table 127 in the RM.
        regs.SPI3.I2SPR.modify(.{
            .I2SDIV = 3,
            .ODD = 1,
            .MCKOE = 1,
        });

        // Enable PLL I2S clock
        regs.RCC.CR.modify(.{
            .PLLI2SON = 1,
        });
    }

    // Configure the pins
    {
        // Enable clock for GPIO Port A and GPIO Port C
        regs.RCC.AHB1ENR.modify(.{
            .GPIOAEN = 1,
            .GPIOCEN = 1,
        });

        // Set PA4 to alternate function mode
        regs.GPIOA.MODER.modify(.{
            .MODER4 = 0b10,
        });

        // Set PC7, PC10 and PC12 to alternate function mode
        regs.GPIOC.MODER.modify(.{
            .MODER7 = 0b10,
            .MODER10 = 0b10,
            .MODER12 = 0b10,
        });

        // PA4 is WS (word select)
        regs.GPIOA.AFRL.modify(.{
            .AFRL4 = 6,
        });

        // PC7 is MCK (master clock)
        regs.GPIOC.AFRL.modify(.{
            .AFRL7 = 6,
        });

        // PC10 is CK (?)
        // PC12 is SD (serial data)
        regs.GPIOC.AFRH.modify(.{
            .AFRH10 = 6,
            .AFRH12 = 6,
        });

        // Configure pull-up for PA4
        regs.GPIOA.PUPDR.modify(.{
            .PUPDR4 = 0b01,
        });

        // Configure pull-up for PC7, PC10 and PC12
        regs.GPIOC.PUPDR.modify(.{
            .PUPDR7 = 0b01,
            .PUPDR10 = 0b01,
            .PUPDR12 = 0b01,
        });

        // Configure high-speed for PA4
        regs.GPIOA.OSPEEDR.modify(.{
            .OSPEEDR4 = 0b11,
        });

        // Configure high-speed for PC7, PC10 and PC12
        regs.GPIOC.OSPEEDR.modify(.{
            .OSPEEDR7 = 0b11,
            .OSPEEDR10 = 0b11,
            .OSPEEDR12 = 0b11,
        });
    }

    // DMA Setup
    {
        // Enable the DMA 1 clock
        regs.RCC.AHB1ENR.modify(.{
            .DMA1EN = 1,
        });

        // Disable the stream for configuration. For example the SxNDTR
        // register can only be written when the stream is disabled. See
        // section 10.5.6 in the reference manual for more information.
        regs.DMA1.S5CR.modify(.{
            .EN = 0,
        });

        // Wait until the EN bit actually reads 0. Refer to the first step of
        // section 10.3.17 in the reference manual.
        while (regs.DMA1.S5CR.read().EN != 0) {}

        // Configure the peripheral address, which is the data register of I2S3
        // (SPI3) output.
        regs.DMA1.S5PAR.modify(.{
            .PA = @intFromPtr(&regs.SPI3.DR.raw),
        });

        // Configure the memory address
        regs.DMA1.S5M0AR.modify(.{
            .M0A = @intFromPtr(&buffer[0]),
        });

        // Configure the size of the memory region
        regs.DMA1.S5NDTR.modify(.{
            .NDT = buffer.len,
        });

        // Configure DMA 1 Stream 5
        regs.DMA1.S5CR.modify(.{
            .TEIE = 1, // Transfer error interrupt
            .HTIE = 1, // Half transfer interrupt
            .TCIE = 1, // Transfer complete interrupt
            .DMEIE = 1, // Direct mode error interrupt

            .DIR = 0b01, // Memory to peripheral

            .CIRC = 1, // Circular mode

            .MINC = 1, // Memory increment

            // Half word (16 Bit), which is what the I2S data
            // register (DR) operates on.
            .PSIZE = 0b01,
            .MSIZE = 0b01,

            // TODO Find out what priority level is approriate for this...
            .PL = 2,

            // Select channel 0 (SPI3_TX)
            .CHSEL = 0,
        });

        // Clear all interrupt flags before enabling the DMA stream
        regs.DMA1.HIFCR.modify(.{
            .CTEIF5 = 1,
            .CHTIF5 = 1,
            .CTCIF5 = 1,
            .CDMEIF5 = 1,
            .CFEIF5 = 1,
        });

        // Enable SPI3 transmission via DMA
        regs.SPI3.CR2.modify(.{
            .TXDMAEN = 1,
        });

        // Enable the DMA1_Stream5 interrupt vector
        {
            const current = regs.NVIC.ISER0.read().SETENA;

            regs.NVIC.ISER0.modify(.{
                .SETENA = current | (1 << 16),
            });
        }

        // Finally enable the DMA stream
        regs.DMA1.S5CR.modify(.{
            .EN = 1,
        });
    }

    // Select I2S mode for SPI3
    regs.SPI3.I2SCFGR.modify(.{
        .I2SMOD = 1,
        .I2SCFG = 0b10, // Master mode, transmit
        .I2SSTD = 0b00, // I2S Philips
        .CKPOL = 0,
        .DATLEN = 0b00, // 24 bit
        .CHLEN = 0b0, // 32 bit wide
        .PCMSYNC = 0,
    });

    log.debug("Waiting for I2S clock ready", .{});

    while (true) {
        const value = regs.RCC.CR.read();

        if (value.PLLI2SRDY == 1) {
            break;
        }
    }

    log.debug("PLL I2S clock ready", .{});

    // Enable I2S
    regs.SPI3.I2SCFGR.modify(.{
        .I2SE = 1,
    });
}
