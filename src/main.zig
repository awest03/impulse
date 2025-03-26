const rl = @import("raylib");
const std = @import("std");
const sparse_set = @import("sparse_set.zig");

pub fn main() !void {
    const w = 800;
    const h = 450;

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    rl.initWindow(w, h, "Impulse");
    defer rl.closeWindow();

    rl.setTargetFPS(60);

    _ = try sparse_set.SparseSet(@Vector(2, f32), u16).init(arena.allocator(), 10);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.white);

        rl.drawText("First Window", 190, 200, 20, .purple);
    }
}
