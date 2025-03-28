const rl = @import("raylib");
const std = @import("std");
const phys = @import("physics.zig");

pub fn main() !void {
    const w = 800;
    const h = 450;

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();

    var world = try phys.World.init(arena.allocator(), 10);
    defer world.deinit(arena.allocator());
    var updater = phys.FixedStepUpdater.init(1.0 / 60.0);

    const material = phys.Material{
        .density = 100.0,
        .dynamic_friction = 1.0,
        .restitution = 1.0,
        .static_friction = 1.0,
    };

    const body_id = try world.createBody(phys.Vec2{ 200.0, -100.0 }, &material, phys.Shape{ .circle = phys.Circle{ .radius = 50.0 } });
    _ = try world.createBody(phys.Vec2{ 150.0, -300.0 }, &material, phys.Shape{ .aabb = phys.AABB{ .half_extents = phys.Vec2{ 100.0, 50.0 } } });

    rl.initWindow(w, h, "Impulse");
    defer rl.closeWindow();

    rl.setTargetFPS(60.0);

    while (!rl.windowShouldClose()) {
        updater.update(&world, rl.getFrameTime());
        var b = try world.bodies.get(body_id);
        b.force = phys.Vec2{ 0.0, -9.8 * 1.0 / b.inv_mass };

        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.white);

        drawWorld(&world);
    }
}

fn drawWorld(world: *const phys.World) void {
    for (0..world.bodies.size) |i| {
        drawBody(&world.bodies.dense[i], rl.Color.purple);
    }
}

fn drawBody(body: *const phys.Body, colour: rl.Color) void {
    switch (body.shape) {
        phys.Shape.aabb => |aabb| {
            const r = rl.Rectangle{
                .x = body.position[0],
                .y = -body.position[1],
                .width = aabb.half_extents[0] * 2,
                .height = aabb.half_extents[1] * 2,
            };
            rl.drawRectanglePro(r, rl.Vector2{ .x = aabb.half_extents[0], .y = aabb.half_extents[1] }, 0, colour);
        },
        phys.Shape.circle => |circle| {
            rl.drawCircle(@as(i32, @intFromFloat(body.position[0])), @as(i32, @intFromFloat(-body.position[1])), circle.radius, colour);
        },
    }
}
