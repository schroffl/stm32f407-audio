format long

# The frequency of the external oscillator, which (in case of the
# STM32f407-DISC1 board) is a lot more stable than the internal oscillator and
# thus produces more accurate results.
hse = 8_000_000

# See table 127 in the reference manual.
pll_M = 8
pll_I2SNx = 258
pll_I2SRx = 3
i2s_div = 3
i2s_odd = 1

# The output format
bit_depth = 16
target_Fs = 48_000

# When emitting a master clock, the actual sampling frequency might differ from
# the desired target frequency. This formula is taken from the reference manual
# (section 28.4.4) and calculates the achieved sampling rate. The difference is
# not big, but using the wrong reference in code can cause the audio signal to
# be off by a few Hertz.
Fs = (hse / pll_M * pll_I2SNx / pll_I2SRx) / ( (bit_depth * 2) * (2 * i2s_div + i2s_odd) * 8 )
error_percent = (target_Fs / Fs - 1) * 100

# The amount of _frames_ (!) per interrupt.
buffer_frame_size = 128

# How many times the DMA interrupt triggers per second.
dma_interrupt_frequency = Fs / buffer_frame_size

# This is the maximum amount of time in milliseconds the CPU can take per I2S
# DMA interrupt without causing a buffer underrun.
time_per_interrupt_ms = 1000 / dma_interrupt_frequency
