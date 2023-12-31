const std = @import("std");
const mem = std.mem;
const microzig = @import("microzig");
const delay = @import("delay.zig");
const smiz = @import("smiz");

const c = @cImport({
    @cInclude("libopencm3/stm32/gpio.h");
    @cInclude("libopencm3/stm32/rcc.h");
    @cInclude("libopencm3/stm32/adc.h");
    @cInclude("libopencm3/stm32/usart.h");
    @cInclude("libopencm3/stm32/lptimer.h");
    @cInclude("libopencm3/stm32/timer.h");
    @cInclude("libopencm3/stm32/l0/nvic.h");
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
    c.rcc_periph_clock_enable(c.RCC_TIM2);
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
fn digits(source: anytype, allocator: std.mem.Allocator) ![]u8 {
    // var buffer: [10]u8 = undefined;
    var buffer = std.ArrayList(u8).init(allocator);

    var temp = std.math.absCast(source);

    while (temp != 0) {
        var digit: u8 = @truncate(@mod(temp, 10));
        try buffer.append(digit + 0x30); // ASCII

        temp = @divFloor(temp, 10);
    }

    // Add the dash if negative.
    if (source < 0) {
        try buffer.append(0x2D);
    }

    var start: usize = 0;
    var end = buffer.items.len;

    while (start < end) {
        const a = buffer.items[start];
        const b = buffer.items[end - 1];

        buffer.items[start] = b;
        buffer.items[end - 1] = a;
        start += 1;
        end -= 1;
    }

    return try buffer.toOwnedSlice();
}

fn println(header: []const u8, message: []u8) void {
    print(header);
    print(message);
    print("\n");
}

var status = .{
    .measuring_temperature = false,
    .measuring_moisture = false,
};
var frequency: u32 = 0;

const State = enum {
    Sleeping,
    Measuring,
};

// Get status doesn't need to be in events, since those commands are responded instantly
const Event = enum {
    MeasTemp,
    MeasMoisture,
    MeasBoth,
    TimerExpired,
};

fn wait(time_ms: u32) void {
    c.timer_set_period(c.TIM2, time_ms);
    c.timer_set_counter(c.TIM2, 1);

    // The UG event is useful to cause shadow registers to be preloaded before
    // the timer is started to avoid uncertainties in the first cycle in case
    // an update event may never be generated.
    c.timer_generate_event(c.TIM2, c.TIM_EGR_UG); // Magic line
    c.timer_enable_counter(c.TIM2);
}

fn onTransition(from: State, to: State, event: ?Event) void {
    _ = to;
    _ = from;

    c.gpio_toggle(c.GPIOA, c.GPIO10);
    switch (event.?) {
        .MeasTemp => {
            // TODO. Start a temperature measurement
            // status.measuring_temperature = true;
            wait(100); // 100ms
        },
        .MeasMoisture => {},
        .MeasBoth => {},
        .TimerExpired => {
            // c.gpio_toggle(c.GPIOA, c.GPIO10);
        },
    }
}

// zig fmt: off
var sm = smiz.StateMachine(
    .{
        .state_type = State,
        .event_type = Event,
        .initial_state = .Sleeping,
        .transitions = &.{
            .{ .event = .MeasTemp,     .from = .Sleeping,  .to = .Measuring },
            .{ .event = .MeasMoisture, .from = .Sleeping,  .to = .Measuring },
            .{ .event = .MeasBoth,     .from = .Sleeping,  .to = .Measuring },
            .{ .event = .TimerExpired, .from = .Measuring, .to = .Sleeping },
        },
        .handler = &onTransition,
    },
){};
// zig fmt: on
var temperature: u16 = 0;

// TODO! Problem! It seems my event is generated on counter start, not on counter overflow!
pub const microzig_options = struct {
    pub const interrupts = struct {
        pub fn TIM2() void {
            c.timer_clear_flag(c.TIM2, c.TIM_SR_UIF);
            sm.stepWithEvent(.TimerExpired) catch unreachable;
        }
    };
};

fn startMoisture() void {
    // Enable frequency measurement
    c.gpio_set(c.GPIOA, c.GPIO4);
    // TODO: Maybe wait a bit to settle?
    c.lptimer_enable(c.LPTIM1);
    c.lptimer_start_counter(c.LPTIM1, c.LPTIM_CR_SNGSTRT);
}

fn startTemp() void {
    // Enable temperature measurement
    c.gpio_set(c.GPIOA, c.GPIO6);
    // TODO. Maybe wait a bit to settle?
    c.adc_start_conversion_regular(c.ADC1);
}

fn nvic_setup() void {
    c.nvic_enable_irq(c.NVIC_TIM2_IRQ);
    c.nvic_set_priority(c.NVIC_TIM2_IRQ, 1);
}

pub fn main() !void {
    rcc_setup();
    gpio_setup();
    adc_setup();
    usart_setup();
    comparator_setup();
    lptim_setup();
    nvic_setup();

    // Setup Timer

    c.timer_set_counter(c.TIM2, 1);
    c.timer_set_prescaler(c.TIM2, c.rcc_apb1_frequency / 1000);
    c.timer_set_period(c.TIM2, 500); // 500ms
    c.timer_one_shot_mode(c.TIM2);
    c.timer_update_on_overflow(c.TIM2); // Send interrupts only on overflow
    // It's always a good idea to clear interrupt flags before enabling an interrupt source.
    c.timer_clear_flag(c.TIM2, c.TIM_SR_UIF);
    c.timer_enable_irq(c.TIM2, c.TIM_DIER_UIE);

    try sm.stepWithEvent(.MeasTemp);

    while (true) {
        // TODO. Maybe process pending events, then wfi in the future
        asm volatile ("wfi");
    }

    // 16 bytes of "heap" should be overkill for my array operations
    var buf: [16]u8 = undefined;
    var fa = std.heap.FixedBufferAllocator.init(&buf);
    defer fa.reset();

    const allocator = fa.allocator();

    // This is how we enable frequency measurement
    c.gpio_set(c.GPIOA, c.GPIO4);

    const vrefint_cal_addr: *u32 = @ptrFromInt(0x1FF8_0078);
    const vref_cal: u32 = vrefint_cal_addr.*;
    _ = vref_cal;

    // println("Base frequency: ", freq);

    // while (true) {
    //     // process_sm(); // Calls the code for the current state.
    //     asm volatile ("wfi");
    // }

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

        const val = c.lptimer_get_counter(c.LPTIM1);
        c.lptimer_disable(c.LPTIM1);

        // For debug purposes I print the frequency in Hertz over searial
        const moisture: u32 = (@as(u32, val) * 1000) / 100;
        const freq_ascii = try digits(moisture, allocator);
        defer allocator.free(freq_ascii); // Very nice! I have to free this slice or I get a memory leak
        // A common pattern with FixedBufferAllocators, is to reset them and reuse them.
        // This frees all previous allocations and allows the allocator to be reused.
        // defer fa.reset();

        println("Frequency: ", freq_ascii);

        // asm volatile ("wfi");
    }

    unreachable;
}

test "state transitions" {
    // TODO
}

test "expect that digits are converted successfully" {
    const my_slice_u8 = try digits(@as(u8, 123), std.testing.allocator);
    defer std.testing.allocator.free(my_slice_u8);
    try std.testing.expectEqualSlices(u8, "123", my_slice_u8);

    const my_slice_i8 = try digits(@as(i8, -123), std.testing.allocator);
    defer std.testing.allocator.free(my_slice_i8);
    try std.testing.expectEqualSlices(u8, "-123", my_slice_i8);

    const my_slice_u16 = try digits(@as(u16, 12345), std.testing.allocator);
    defer std.testing.allocator.free(my_slice_u16);
    try std.testing.expectEqualSlices(u8, "12345", my_slice_u16);

    const my_slice_i16 = try digits(@as(i16, -12345), std.testing.allocator);
    defer std.testing.allocator.free(my_slice_i16);
    try std.testing.expectEqualSlices(u8, "-12345", my_slice_i16);

    const my_slice_u32 = try digits(@as(u32, 12345678), std.testing.allocator);
    defer std.testing.allocator.free(my_slice_u32);
    try std.testing.expectEqualSlices(u8, "12345678", my_slice_u32);

    const my_slice_i32 = try digits(@as(i32, -12345678), std.testing.allocator);
    defer std.testing.allocator.free(my_slice_i32);
    try std.testing.expectEqualSlices(u8, "-12345678", my_slice_i32);
}
