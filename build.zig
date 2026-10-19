const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});

    // Rendering is the hot path here, so optimized builds are the default.
    // Pass -Doptimize=Debug to inspect the code with the debugger.
    const optimize = b.option(
        std.builtin.OptimizeMode,
        "optimize",
        "Prioritize performance, safety, or binary size",
    ) orelse .ReleaseFast;

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        // raylib itself is not the hot path. It is built as a shared library
        // because the static archive bundles the system .so files, which LLD
        // rejects when linking a ReleaseFast executable.
        .optimize = .ReleaseSafe,
        .linkage = .dynamic,
        // Audio is unused and would pull in an ALSA link dependency.
        .raudio = false,
    });
    const raylib = raylib_dep.module("raylib");

    const exe = b.addExecutable(.{
        .name = "baricentricas",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "raylib", .module = raylib },
            },
        }),
    });
    b.installArtifact(exe);
    // Ship the shared raylib next to the binary so zig-out/bin/baricentricas
    // runs on its own.
    b.installArtifact(raylib_dep.artifact("raylib"));

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);

    const run_step = b.step("run", "Render triangles with barycentric color interpolation");
    run_step.dependOn(&run_cmd.step);
}
