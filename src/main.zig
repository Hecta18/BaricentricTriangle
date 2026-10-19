//! Barycentric color interpolation over triangles, raytraced with raylib 6.0.
//!
//! One primary ray per pixel is intersected with the triangle using the
//! Moller-Trumbore algorithm, which yields the barycentric coordinates (u, v)
//! of the hit point for free: vertex 1 weighs u, vertex 2 weighs v and vertex 0
//! weighs 1 - u - v. The color of the pixel is the weighted sum of the vertex
//! colors, so every channel is strongest at its own vertex.
//!
//! Performance notes (the only thing this exercises cares about):
//!   * the view matrix is built once per frame by raylib and the camera-space
//!     ray is incremented per pixel instead of being recomputed,
//!   * only the screen-space bounding box of the triangle is traversed,
//!   * the frame is only re-rendered when the camera or the triangle changes,
//!   * the whole image is written to a CPU buffer and pushed to the GPU with a
//!     single texture update per frame.

const std = @import("std");
const rl = @import("raylib");

/// Float vector used in the hot loop; raylib's Vector3 stays at the boundary.
const Vec3 = @Vector(3, f32);

const screen_width = 1280;
const screen_height = 720;
const fov_degrees: f32 = 60.0;
/// Rays closer than this are considered behind the triangle.
const near_plane: f32 = 0.05;
const parallel_epsilon: f32 = 1e-6;
const orbit_speed: f32 = 0.005;
const pitch_limit: f32 = 1.4;

const background: rl.Color = .{ .r = 18, .g = 18, .b = 24, .a = 255 };
const wire_color: rl.Color = .{ .r = 240, .g = 240, .b = 240, .a = 255 };

const Triangle = struct {
    name: [:0]const u8,
    vertices: [3]Vec3,
};

/// Three triangles of different shapes and orientations; the renderer must
/// work for any of them.
const triangles = [_]Triangle{
    .{
        .name = "1. flat triangle",
        .vertices = .{
            .{ -3.0, -2.2, 0.0 },
            .{ 3.0, -2.2, 0.0 },
            .{ 0.0, 2.6, 0.0 },
        },
    },
    .{
        .name = "2. thin sliver",
        .vertices = .{
            .{ -4.0, -1.6, 0.0 },
            .{ 4.0, -1.6, 0.0 },
            .{ 3.4, 1.8, 0.0 },
        },
    },
    .{
        .name = "3. tilted in 3D",
        .vertices = .{
            .{ -2.6, -2.2, 1.4 },
            .{ 2.8, -1.8, -1.2 },
            .{ 0.6, 2.4, 0.5 },
        },
    },
};

const Ray = struct {
    origin: Vec3,
    direction: Vec3,
};

/// Barycentric weights of the hit point: w0 for vertex 0, w1 for vertex 1
/// and w2 for vertex 2.
const Barycentric = struct {
    w1: f32,
    w2: f32,
};

/// Inclusive pixel rectangle, used to skip the parts of the screen the
/// triangle cannot cover.
const ScreenRect = struct {
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
};

inline fn toVec(v: rl.Vector3) Vec3 {
    return .{ v.x, v.y, v.z };
}

inline fn toVector3(v: Vec3) rl.Vector3 {
    return .{ .x = v[0], .y = v[1], .z = v[2] };
}

inline fn vecEquals(a: Vec3, b: Vec3) bool {
    return @reduce(.And, a == b);
}

inline fn cross(a: Vec3, b: Vec3) Vec3 {
    return .{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

/// Ray/Moller-Trumbore intersection (https://en.wikipedia.org/wiki/Moller-Trumbore_intersection_algorithm).
/// `a` is the first vertex, `e1` and `e2` the two edge vectors. Returns null
/// when the ray misses the triangle.
inline fn intersectTriangle(ray: Ray, a: Vec3, e1: Vec3, e2: Vec3) ?Barycentric {
    const p = cross(ray.direction, e2);
    const det = @reduce(.Add, e1 * p);
    if (@abs(det) < parallel_epsilon) return null; // ray is parallel to the triangle

    const inv_det = 1.0 / det;
    const s = ray.origin - a;
    const w1 = @reduce(.Add, s * p) * inv_det;
    if (w1 < 0.0 or w1 > 1.0) return null;

    const q = cross(s, e1);
    const w2 = @reduce(.Add, ray.direction * q) * inv_det;
    if (w2 < 0.0 or w1 + w2 > 1.0) return null;

    if (@reduce(.Add, e2 * q) * inv_det < near_plane) return null;

    return .{ .w1 = w1, .w2 = w2 };
}

/// Barycentric interpolation of the three vertex colors.
inline fn barycentricColor(bary: Barycentric) rl.Color {
    const w0 = 1.0 - bary.w1 - bary.w2;
    return .{
        .r = toChannel(255.0 * w0),
        .g = toChannel(255.0 * bary.w1),
        .b = toChannel(255.0 * bary.w2),
        .a = 255,
    };
}

inline fn toChannel(value: f32) u8 {
    return @intFromFloat(std.math.clamp(value, 0.0, 255.0));
}

/// Color of a single vertex, i.e. the interpolation evaluated at that vertex.
fn vertexColor(index: usize) rl.Color {
    return barycentricColor(switch (index) {
        0 => .{ .w1 = 0.0, .w2 = 0.0 },
        1 => .{ .w1 = 1.0, .w2 = 0.0 },
        else => .{ .w1 = 0.0, .w2 = 1.0 },
    });
}

/// Maps a camera-space direction to world space. `view` is the world->camera
/// matrix raylib builds, so the rotation is simply transposed and the
/// translation is dropped; doing it here saves raylib's per-pixel C call.
inline fn cameraDirection(v: Vec3, view: rl.Matrix) Vec3 {
    return .{
        view.m0 * v[0] + view.m1 * v[1] + view.m2 * v[2],
        view.m4 * v[0] + view.m5 * v[1] + view.m6 * v[2],
        view.m8 * v[0] + view.m9 * v[1] + view.m10 * v[2],
    };
}

/// Projects the vertices with raylib and returns the pixel rectangle that
/// covers the triangle, clamped to the screen. A vertex behind the camera can
/// only inflate the rectangle, never miss a pixel: the distance test in
/// `intersectTriangle` decides what is actually inside.
fn screenBounds(triangle: Triangle, camera: rl.Camera3D) ScreenRect {
    var min_x: i32 = screen_width;
    var min_y: i32 = screen_height;
    var max_x: i32 = 0;
    var max_y: i32 = 0;

    for (triangle.vertices) |vertex| {
        const screen = rl.getWorldToScreenEx(toVector3(vertex), camera, screen_width, screen_height);
        const px: i32 = @intFromFloat(screen.x);
        const py: i32 = @intFromFloat(screen.y);
        min_x = @min(min_x, px);
        min_y = @min(min_y, py);
        max_x = @max(max_x, px);
        max_y = @max(max_y, py);
    }

    return .{
        .x0 = @max(0, min_x - 1),
        .y0 = @max(0, min_y - 1),
        .x1 = @min(screen_width - 1, max_x + 1),
        .y1 = @min(screen_height - 1, max_y + 1),
    };
}

/// Casts one ray per pixel inside the triangle's bounding box and writes the
/// interpolated colors into `pixels`.
fn render(pixels: []rl.Color, camera: rl.Camera3D, triangle: Triangle) void {
    for (pixels) |*pixel| pixel.* = background;

    const bounds = screenBounds(triangle, camera);
    if (bounds.x0 > bounds.x1 or bounds.y0 > bounds.y1) return;

    const view = camera.getMatrix();
    const width_f: f32 = @floatFromInt(screen_width);
    const height_f: f32 = @floatFromInt(screen_height);

    const vertices = triangle.vertices;
    const a = vertices[0];
    // Constant for the whole frame, hoisted out of the pixel loop.
    const e1 = vertices[1] - a;
    const e2 = vertices[2] - a;

    const tan_half_fov = @tan(fov_degrees * std.math.pi / 360.0);
    const half_width = tan_half_fov * (width_f / height_f);
    const step_x = 2.0 * half_width / width_f;

    var ray = Ray{ .origin = toVec(camera.position), .direction = undefined };

    var y = bounds.y0;
    while (y <= bounds.y1) : (y += 1) {
        const ndc_y = (1.0 - 2.0 * (@as(f32, @floatFromInt(y)) + 0.5) / height_f) * tan_half_fov;
        var ndc_x = (2.0 * (@as(f32, @floatFromInt(bounds.x0)) + 0.5) / width_f - 1.0) * half_width;

        var x = bounds.x0;
        while (x <= bounds.x1) : (x += 1) {
            ray.direction = cameraDirection(.{ ndc_x, ndc_y, -1.0 }, view);
            if (intersectTriangle(ray, a, e1, e2)) |bary| {
                pixels[@as(usize, @intCast(y)) * screen_width + @as(usize, @intCast(x))] = barycentricColor(bary);
            }
            ndc_x += step_x;
        }
    }
}

/// Camera position on a sphere around the origin.
fn orbitPosition(yaw: f32, pitch: f32, distance: f32) Vec3 {
    return .{
        distance * @cos(pitch) * @sin(yaw),
        distance * @sin(pitch),
        distance * @cos(pitch) * @cos(yaw),
    };
}

pub fn main() !void {
    rl.setTraceLogLevel(.warning);
    rl.initWindow(screen_width, screen_height, "Barycentric colors");
    defer rl.closeWindow();

    const image = rl.genImageColor(screen_width, screen_height, background);
    const pixel_count = screen_width * screen_height;
    const bytes = @as([*]u8, @ptrCast(image.data))[0 .. pixel_count * @sizeOf(rl.Color)];
    const pixels = std.mem.bytesAsSlice(rl.Color, bytes);
    const texture = try rl.loadTextureFromImage(image);
    defer rl.unloadTexture(texture);
    rl.setTextureFilter(texture, .bilinear);

    var camera = rl.Camera3D{
        .position = toVector3(orbitPosition(0.6, 0.25, 9.0)),
        .target = .zero(),
        .up = .{ .x = 0.0, .y = 1.0, .z = 0.0 },
        .fovy = fov_degrees,
        .projection = .perspective,
    };

    var current: usize = 0;
    var last_triangle: usize = 0;
    var yaw: f32 = 0.6;
    var pitch: f32 = 0.25;
    const distance: f32 = 9.0;

    var last_position = toVec(camera.position);
    var dirty = true;

    while (!rl.windowShouldClose()) {
        if (rl.isKeyDown(.one)) current = 0;
        if (rl.isKeyDown(.two)) current = 1;
        if (rl.isKeyDown(.three)) current = 2;

        if (rl.isMouseButtonDown(.left)) {
            const delta = rl.getMouseDelta();
            yaw -= delta.x * orbit_speed;
            pitch = std.math.clamp(pitch + delta.y * orbit_speed, -pitch_limit, pitch_limit);
        }

        camera.position = toVector3(orbitPosition(yaw, pitch, distance));

        // Nothing moved: reuse the framebuffer already on the GPU.
        if (!vecEquals(toVec(camera.position), last_position) or current != last_triangle) {
            last_position = toVec(camera.position);
            last_triangle = current;
            dirty = true;
        }

        const triangle = triangles[current];

        rl.beginDrawing();
        if (dirty) {
            render(pixels, camera, triangle);
            rl.updateTexture(texture, pixels.ptr);
            dirty = false;
        }

        rl.clearBackground(background);
        rl.drawTexture(texture, 0, 0, .white);

        camera.begin();
        for (triangle.vertices, 0..) |vertex, i| {
            rl.drawLine3D(toVector3(vertex), toVector3(triangle.vertices[(i + 1) % 3]), wire_color);
            rl.drawSphere(toVector3(vertex), 0.07, vertexColor(i));
        }
        camera.end();

        rl.drawText(triangle.name, 12, 12, 20, wire_color);
        rl.drawText("1/2/3: triangle   drag: orbit", 12, 40, 16, .gray);
        rl.drawFPS(12, 66);
        rl.endDrawing();
    }
}
