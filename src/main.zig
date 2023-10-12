const std = @import("std");
const mem = std.mem;
const microzig = @import("microzig");
const delay = @import("delay.zig");

const c = @cImport({
    @cInclude("libopencm3/stm32/gpio.h");
    @cInclude("libopencm3/stm32/rcc.h");
    @cInclude("libopencm3/stm32/adc.h");
    @cInclude("libopencm3/stm32/usart.h");
    @cInclude("libopencm3/stm32/lptimer.h");
});

const RCC = microzig.chip.peripherals.RCC;

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
    c.rcc_set_usart2_sel(0x00); // APB
    c.rcc_set_lptim1_sel(0x00); // APB
    c.rcc_set_sysclk_source(c.RCC_HSI16);

    c.rcc_periph_clock_enable(c.RCC_USART2);
    c.rcc_periph_clock_enable(c.RCC_ADC1);
    c.rcc_periph_clock_enable(c.RCC_GPIOA);
    c.rcc_periph_clock_enable(c.RCC_LPTIM1);
    // Enable system configuration clock. Needed for comparator.
    c.rcc_periph_clock_enable(c.RCC_SYSCFG);
}

fn gpio_setup() void {
    // Set GPIO10 as output
    c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_OUTPUT, c.GPIO_PUPD_NONE, c.GPIO10);

    // TODO. Add all Comparator-related pins' Alternate Functions
    c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_OUTPUT, c.GPIO_PUPD_NONE, c.GPIO4); // Frequency enable
    // c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_OUTPUT, c.GPIO_PUPD_NONE, c.GPIO0); // Comparator out

    c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_AF, c.GPIO_PUPD_NONE, c.GPIO0); // Comparator out
    // c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_ANALOG, c.GPIO_PUPD_NONE, c.GPIO1); // Comparator in
    // c.gpio_mode_setup(c.GPIOA, c.GPIO_MODE_ANALOG, c.GPIO_PUPD_NONE, c.GPIO5); // Comparator in
    c.gpio_set_af(c.GPIOA, c.GPIO_AF7, c.GPIO0); // AF7 is COMP1 out
    // c.gpio_set_af(c.GPIOA, c.GPIO_AF1, c.GPIO5); // PA5 is LPTIM1_IN2

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

fn lptim_setup() void {
    // Any settings need to be configured while timer is disabled.
    c.lptimer_disable(c.LPTIM1);

    c.rcc_set_peripheral_clk_sel(c.LPTIM1, c.RCC_CCIPR_LPTIM1SEL_APB);
    c.rcc_periph_clock_enable(c.RCC_LPTIM1);

    // LPTIMER is driven from comparator out
    c.lptimer_set_external_clock_source(c.LPTIM1);
    c.lptimer_set_prescaler(c.LPTIM1, c.LPTIM_CFGR_PRESC_1);
    c.lptimer_select_trigger_source(c.LPTIM1, c.LPTIM_CFGR_TRIGSEL_EXT_TRIG6);
    c.lptimer_enable_trigger(c.LPTIM1, c.LPTIM_CFGR_TRIGEN_SW);

    c.lptimer_enable(c.LPTIM1);

    // May not be necessary
    // Timer will stop when reaching this value
    c.lptimer_set_period(c.LPTIM1, 0xFFFF);
}

// The comparator is really easy to set up
fn comparator_setup() void {
    // c.rcc_periph_clock_enable(c.RCC_COMP1);
    const COMP1_CSR: *u32 = @ptrFromInt(0x4001_0018);
    // Bit 15   - COMP1POLARITY = 0 (not inverted)
    // Bit 12   - COMP1LPTIMIN1 = 1 (COMP1 out connected to LPTIM1)
    // Bit 8    - COMP1WM = 0 (Plus input of comparator 1 connected to PA1)
    // Bit 5:4  - COMP1INNSEL = 0b11 (COMP1 minus connected to PA5)
    // Bit 0    - COMP1EN = 1 (Comparator 1 enabled)
    // COMP1_CSR.* = 0b0001_0000_0011_0001;
    COMP1_CSR.* = 0x1031;
}

fn print(str: []const u8) void {
    for (str, 0..) |char, i| {
        _ = i;

        const payload: u16 = @intCast(char);
        c.usart_send_blocking(c.USART2, payload);
    }
}

/// Error type for ASCII conversion
const DigitConversionError = error{
    /// The provided buffer was not big enough and cannot fit the number.
    BufferOverflow,
};

/// Converts the given integer into ASCII digits.
/// Supports positive or negative integers.
fn digits(source: anytype, dest: []u8) !usize {
    var buffer: [10]u8 = undefined;
    var i: usize = 0;
    var j: usize = 0;
    var temp = std.math.absCast(source);

    while (temp != 0) : (i += 1) {
        if (i >= dest.len) {
            return DigitConversionError.BufferOverflow;
        }

        var digit: u8 = @truncate(@mod(temp, 10));
        buffer[i] = digit + 0x30; // ASCII

        temp = @divFloor(temp, 10);
    }

    // Add the dash if negative.
    if (source < 0) {
        buffer[i] = 0x2D; // ASCII "-"
        i += 1;
    }

    const length = i;

    // Reverse the contents in the slice.
    while (i != 0) : ({
        i -= 1;
        j += 1;
    }) {
        dest[j] = buffer[i - 1];
    }

    return length;
}

fn println(header: []const u8, val: anytype) !void {
    var bytes: [10]u8 = undefined;
    const len = try digits(val, &bytes);

    print(header);
    print(bytes[0..len]);
    print("\n");
}

pub fn main() !void {
    rcc_setup();
    gpio_setup();
    adc_setup();
    usart_setup();
    comparator_setup();
    lptim_setup();

    const timer = delay.Timer{};
    timer.setup();

    // This is how we enable frequency measurement
    c.gpio_set(c.GPIOA, c.GPIO4);

    const vrefint_cal_addr: *u32 = @ptrFromInt(0x1FF8_0078);
    const vref_cal: u32 = vrefint_cal_addr.*;
    _ = vref_cal;

    const freq: u32 = c.rcc_apb1_frequency;
    print(mem.asBytes(&freq));
    try println("Base frequency: ", freq);

    while (true) {
        c.adc_start_conversion_regular(c.ADC1);
        while (!c.adc_eoc(c.ADC1)) {}

        var temp: u16 = @intCast(c.adc_read_regular(c.ADC1));
        _ = std.mem.doNotOptimizeAway(temp);
        // println(u32, "ADC: ", vref_cal);

        const foo = c.rcc_get_usart_clk_freq(c.USART2);
        _ = std.mem.doNotOptimizeAway(foo);

        c.gpio_toggle(c.GPIOA, c.GPIO10);
        // println("Hello World!\r");

        c.lptimer_enable(c.LPTIM1);
        c.lptimer_start_counter(c.LPTIM1, c.LPTIM_CR_SNGSTRT);

        timer.delay_ms(100);
        const val = c.lptimer_get_counter(c.LPTIM1);
        c.lptimer_disable(c.LPTIM1);

        // For debug purposes I print the frequency in Hertz over searial
        const frequency: u32 = (@as(u32, val) * 1000) / 100;
        try println("Frequency: ", frequency);
    }

    unreachable;
}

test "expect that digits are converted successfully" {
    // Create a buffer to store the converted result.
    var buffer: [8]u8 = undefined;
    var size: usize = 0;

    size = try digits(@as(u8, 123), &buffer);
    try std.testing.expectEqualSlices(u8, "123", buffer[0..size]);
    size = try digits(@as(i8, -123), &buffer);
    try std.testing.expectEqualSlices(u8, "-123", buffer[0..size]);

    size = try digits(@as(u16, 3214), &buffer);
    try std.testing.expectEqualSlices(u8, "3214", buffer[0..size]);
    size = try digits(@as(i16, -3214), &buffer);
    try std.testing.expectEqualSlices(u8, "-3214", buffer[0..size]);

    size = try digits(@as(u32, 696969), &buffer);
    try std.testing.expectEqualSlices(u8, "696969", buffer[0..size]);
    size = try digits(@as(i32, -696969), &buffer);
    try std.testing.expectEqualSlices(u8, "-696969", buffer[0..size]);

    // Test that this function throws a overflow error if my buffer is too small.
    var result = digits(@as(u32, 1_000_000_000), &buffer);
    try std.testing.expectError(DigitConversionError.BufferOverflow, result);
}
