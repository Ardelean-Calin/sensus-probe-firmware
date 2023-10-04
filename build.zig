const std = @import("std");
const stm32 = @import("stm32");

pub fn build(b: *std.Build) void {
    const microzig = @import("microzig").init(b, "microzig");
    const optimize = .ReleaseSmall; // The others are not really an option on AVR

    // `addFirmware` basically works like addExecutable, but takes a
    // `microzig.Target` for target instead of a `std.zig.CrossTarget`.
    //
    // The target will convey all necessary information on the chip,
    // cpu and potentially the board as well.
    const firmware = microzig.addFirmware(b, .{
        .name = "sensus-probe",
        .target = stm32.chips.stm32l011x4,
        .optimize = optimize,
        .source_file = .{ .path = "src/main.zig" },
    });

    const libopencm3_dep = b.dependency("libopencm3", .{});
    const libopencm3 = libopencm3_dep.artifact("opencm3");

    firmware.artifact.defineCMacroRaw("STM32L0");
    firmware.artifact.linkLibrary(libopencm3);
    firmware.artifact.installLibraryHeaders(libopencm3);

    // firmware.artifact.addLibraryPath(.{ .path = "lib" });
    // firmware.artifact.linkSystemLibrary("opencm3_stm32l0");
    // firmware.artifact.addLibraryPath(.{ .path = "./libopencm3/zig-out/lib/" });
    // firmware.artifact.linkSystemLibrary("opencm3");
    // firmware.addIncludePath(.{ .path = "./libopencm3/include/" });
    // firmware.addCSourceFile(.{ .file = .{ .path = "./libopencm3/lib/stm32/common/gpio_common_all.c" }, .flags = &[_][]const u8{} });
    // firmware.addCSourceFile(.{ .file = .{ .path = "./libopencm3/lib/stm32/common/gpio_common_f0234.c" }, .flags = &[_][]const u8{} });
    // firmware.addCSourceFile(.{ .file = .{ .path = "./libopencm3/lib/stm32/common/rcc_common_all.c" }, .flags = &[_][]const u8{} });
    // firmware.addCSourceFile(.{ .file = .{ .path = "./libopencm3/lib/stm32/l0/rcc.c" }, .flags = &[_][]const u8{} });
    // firmware.addCSourceFile(.{ .file = .{ .path = "empty.c" }, .flags = &[_][]const u8{} });

    // `installFirmware()` is the MicroZig pendant to `Build.installArtifact()`
    // and allows installing the firmware as a typical firmware file.
    //
    // This will also install into `$prefix/firmware` instead of `$prefix/bin`.
    microzig.installFirmware(b, firmware, .{});
    // For debugging, we also always install the firmware as an ELF file
    microzig.installFirmware(b, firmware, .{ .format = .elf });
    // microzig.installFirmware(b, firmware, .{ .format = .hex });

    // const flash_cmd = b.addSystemCommand(&[_][]const u8{ "openocd", "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32l0.cfg", "-c", "'program", firmware.getEmittedElf().getPath(b), "verify reset exit'" });
    // _ = flash_cmd;
}
