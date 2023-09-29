const std = @import("std");
const microzig = @import("microzig");

const c = @cImport({
    @cInclude("libopencm3/stm32/gpio.h");
    @cInclude("libopencm3/stm32/rcc.h");
    @cInclude("libopencm3/stm32/adc.h");
    @cInclude("libopencm3/stm32/usart.h");
});

// The order in which the operations run is important.
fn rcc_setup() void {
    // Enable internal High-speed oscillator
    c.rcc_osc_on(c.RCC_HSI16);

    // Initialize the CPU, AHB and APB bus clocks
    const rcc_config = c.rcc_clock_scale{
        .ahb_frequency = 16_000_000,
        .apb1_frequency = 16_000_000,
        .apb2_frequency = 16_000_000,
        .pll_mul = c.RCC_CFGR_PLLMUL_MUL3,
        .pll_div = c.RCC_CFGR_PLLDIV_DIV2,
        .pll_source = c.RCC_CFGR_PLLSRC_HSI16_CLK,
        .hpre = c.RCC_CFGR_HPRE_NODIV,
        .ppre1 = c.RCC_CFGR_PPRE1_NODIV,
        .ppre2 = c.RCC_CFGR_PPRE2_NODIV,
        .flash_waitstates = 1,
        .msi_range = 0, // msi_range doesn't matter
        .voltage_scale = 0, // Voltage scale 1
    };
    c.rcc_clock_setup_pll(&rcc_config);
    c.rcc_set_usart2_sel(0x00); // PCLK
    c.rcc_set_sysclk_source(c.RCC_HSI16);

    c.rcc_periph_clock_enable(c.RCC_USART2);
    c.rcc_periph_clock_enable(c.RCC_ADC1);
    c.rcc_periph_clock_enable(c.RCC_GPIOA);
}

fn gpio_setup() void {
    // Set GPIO10 as output
    c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_OUTPUT, c.GPIO_PUPD_NONE, c.GPIO10);
    // Set GPIO9 as USART Tx
    c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_AF, c.GPIO_PUPD_NONE, c.GPIO9);

    // Setup USART TX pin as alternate function.
    // See STM32L011 datasheet Table 14
    c.gpio_set_af(c.GPIOA, c.GPIO_AF4, c.GPIO9);
}

fn adc_setup() void {
    c.adc_power_off(c.ADC1);
    while (!c.adc_is_power_off(c.ADC1)) {}
    c.rcc_periph_reset_pulse(c.RST_ADC1);
    // c.adc_set_sample_time_on_all_channels(c.ADC1, c.ADC_SMPR_SMP_1DOT5CYC);
    c.adc_set_single_conversion_mode(c.ADC1);
    c.adc_set_right_aligned(c.ADC1);
    c.adc_set_resolution(c.ADC1, c.ADC_CFGR1_RES_12_BIT);
    c.adc_enable_vrefint();
    // c.adc_enable_temperature_sensor();

    c.adc_calibrate(c.ADC1); // Error
    // Last step
    c.adc_power_on(c.ADC1);
    // I don't need to wait, it's already inside the function

}

fn usart_setup() void {
    // USART needs to be disabled before configuration
    c.usart_disable(c.USART2);

    c.usart_set_baudrate(c.USART2, 115200);
    c.usart_set_databits(c.USART2, 8);
    c.usart_set_parity(c.USART2, c.USART_PARITY_NONE);
    c.usart_set_stopbits(c.USART2, c.USART_STOPBITS_1);
    c.usart_set_mode(c.USART2, c.USART_MODE_TX);
    c.usart_set_flow_control(c.USART2, c.USART_FLOWCONTROL_NONE);
    // By default 16 oversampling is enabled. Keep it this way as it works.

    // Finally enable the USART.
    c.usart_enable(c.USART2);
}

fn println(str: []const u8) void {
    for (str, 0..) |char, i| {
        _ = i;

        const payload: u16 = @intCast(char);
        c.usart_send_blocking(c.USART2, payload);
    }
}

pub fn main() !void {
    rcc_setup();
    gpio_setup();
    adc_setup();
    usart_setup();

    const vrefint_cal_addr: *u32 = @ptrFromInt(0x1FF8_0078);
    const vref_cal: u32 = vrefint_cal_addr.*;
    _ = vref_cal;

    while (true) {
        c.adc_start_conversion_regular(c.ADC1);
        while (!c.adc_eoc(c.ADC1)) {}

        var temp: u16 = @intCast(c.adc_read_regular(c.ADC1));
        _ = std.mem.doNotOptimizeAway(temp);

        const foo = c.rcc_get_usart_clk_freq(c.USART2);
        _ = std.mem.doNotOptimizeAway(foo);

        c.gpio_toggle(c.GPIOA, c.GPIO10);
        println("Hello World!\r");
        for (0..1000000) |_| {
            asm volatile ("nop");
        }
    }

    unreachable;
}
