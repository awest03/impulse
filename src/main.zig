const rl = @import("raylib");
const std = @import("std");
const ss = @import("sparseset.zig");

pub fn main() !void {
    const w = 800;
    const h = 450;

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    rl.initWindow(w, h, "Impulse");
    defer rl.closeWindow();

    rl.setTargetFPS(60);

    _ = try ss.SparseSet(@Vector(2, f32), u16).init(10, arena.allocator());

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.white);

        rl.drawText("First Window", 190, 200, 20, .purple);
    }
}
