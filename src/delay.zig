const microzig = @import("microzig");

const c = @cImport({
    @cInclude("libopencm3/stm32/rcc.h");
    @cInclude("libopencm3/stm32/timer.h");
});

pub const Timer = struct {
    const TIMER = microzig.chip.peripherals.TIM2;

    pub fn setup(self: *const Timer) void {
        _ = self;
        c.rcc_periph_clock_enable(c.RCC_TIM2);
        c.timer_set_prescaler(c.TIM2, c.rcc_apb1_frequency / 1000 - 1);
        c.timer_set_period(c.TIM2, 0xffff);
        c.timer_one_shot_mode(c.TIM2);
    }

    // Delay a number of millisecons.
    pub fn delay_ms(self: *const Timer, us: u16) void {
        _ = self;
        c.timer_set_period(c.TIM2, us);
        c.timer_generate_event(c.TIM2, c.TIM_EGR_UG);
        c.timer_enable_counter(c.TIM2);

        while (TIMER.CR1.raw & c.TIM_CR1_CEN != 0) {
            // Very important!
            asm volatile ("nop");
        }
    }
};
