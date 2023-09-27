const std = @import("std");
// const microzig = @import("microzig");

const c = @cImport({
    @cInclude("libopencm3/stm32/gpio.h");
    @cInclude("libopencm3/stm32/rcc.h");
});

pub fn main() !void {

    // Enable the peripheral clock for GPIOA
    c.rcc_periph_clock_enable(c.RCC_GPIOA);
    // Set GPIO10 as output
    c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_OUTPUT, c.GPIO_PUPD_NONE, c.GPIO10);

    while (true) {
        c.gpio_toggle(c.GPIOA, c.GPIO10);
        for (0..200000) |_value| {
            _ = _value;
            asm volatile ("nop");
        }
    }

    unreachable;
}
