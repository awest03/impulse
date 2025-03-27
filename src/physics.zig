const rb = @import("ringbuffer.zig");
const ss = @import("sparseset.zig");
const std = @import("std");

pub const Vec2 = @Vector(2, f32);

fn approxEqVec2(a: Vec2, b: Vec2) bool {
    const threshold: f32 = 0.0001;
    return @abs(a.x - b.x) < threshold and @abs(a.y - b.y) < threshold;
}

fn dotVec2(a: Vec2, b: Vec2) Vec2 {
    return @reduce(std.builtin.ReduceOp.Add, a * b);
}

fn normaliseVec2(a: Vec2) Vec2 {
    return a / @sqrt(dotVec2(a, a));
}

pub const Material = struct {
    restitution: f32,
    dynamic_friction: f32,
    static_friction: f32,
    density: f32,
};

pub const AABB = struct {
    half_extents: Vec2,

    pub fn area(self: *@This()) f32 {
        return 4 * self.half_extents[0] * self.half_extents[1];
    }
};

pub const Circle = struct {
    radius: f32,

    pub fn area(self: *@This()) f32 {
        return std.math.pi * self.radius * self.radius;
    }
};

pub const Shape = union(enum) {
    aabb: AABB,
    circle: Circle,

    pub fn area(self: *@This()) f32 {
        switch (self) {
            inline else => |case| return case.area(),
        }
    }
};

pub const Body = struct {
    shape: Shape,
    material: *Material,
    // Linear Properties
    inv_mass: f32,
    position: Vec2,
    velocity: Vec2,
    force: Vec2,
    // Rotational Properties
    inv_inertia: f32,
    angle: f32,
    angular_velocity: f32,
    torque: f32,

    pub fn recalculateMass(self: *Body) void {
        self.inv_mass = 1 / (self.material.density * self.shape.area());
    }
};

pub const Manifold = struct {
    a: *Body,
    b: *Body,
    penetration: f32,
    normal: Vec2,

    pub fn swapBodies(self: *Manifold) void {
        const c = self.a;
        self.a = self.b;
        self.b = c;
    }
};

fn manifoldAABBvsAABB(manifold: *Manifold) bool {
    const a = manifold.a;
    const b = manifold.b;

    const n = b.position - a.position;
    const x_overlap = a.shape.aabb.half_extents.x + b.shape.aabb.half_extents.x - @abs(n.x);
    if (x_overlap > 0) {
        const y_overlap = a.shape.aabb.half_extents.y + b.shape.aabb.half_extents.y - @abs(n.y);
        if (y_overlap > 0) {
            if (x_overlap < y_overlap) {
                manifold.normal = .{ if (n.x < 0) -1.0 else 1.0, 0.0 };
                manifold.penetration = x_overlap;
            } else {
                manifold.normal = .{ 0.0, if (n.y < 0) -1.0 else 1.0 };
                manifold.penetration = y_overlap;
            }
            return true;
        }
    }
    return false;
}

fn manifoldAABBvsCircle(manifold: *Manifold) bool {
    const a = manifold.a;
    const b = manifold.b;

    const n = b.position - a.position;
    const half_extents = a.shape.aabb.half_extents;
    const closest = Vec2{ std.math.clamp(n.x, -half_extents.x, half_extents.x), std.math.clamp(n.y, -half_extents.y, half_extents.y) };
    const inside = if (approxEqVec2(n, closest)) {
        if (@abs(n.x) > @abs(n.y)) { // Find closest axis
            closest.x = if (closest.x > 0) half_extents.x else -half_extents.x;
        } else {
            closest.y = if (closest.y > 0) half_extents.y else -half_extents.y;
        }
        true;
    } else false;

    const normal = n - closest;
    const d2 = dotVec2(normal, normal);
    const r = b.shape.circle.radius;
    if (d2 > r * r and !inside) return false;

    const d = @sqrt(d2);
    manifold.penetration = r - d;
    manifold.normal = normaliseVec2(if (inside) -n else n);

    return true;
}

fn manifoldCirclevsAABB(manifold: *Manifold) bool {
    // Rather than write out the function twice
    manifold.swapBodies();
    defer manifold.swapBodies();
    defer manifold.normal = -manifold.normal;
    return manifoldAABBvsCircle(manifold);
}

fn manifoldCirclevsCircle(manifold: *Manifold) bool {
    const a = manifold.a;
    const b = manifold.b;

    const n = b.position - a.position;
    const r = a.shape.circle.radius + b.shape.circle.radius;
    const d2 = dotVec2(n, n);
    if (d2 > r * r) return false;

    const d = @sqrt(d2);
    if (d != 0) {
        manifold.penetration = r - d;
        manifold.normal = n * (1 / d);
    } else {
        manifold.penetration = a.shape.circle.radius;
        manifold.normal = .{ 1.0, 0.0 };
    }
    return true;
}

const calc_manifold: [2][2]*const fn (manifold: *Manifold) bool = .{ .{ &manifoldAABBvsAABB, &manifoldAABBvsCircle }, .{ &manifoldCirclevsAABB, &manifoldCirclevsCircle } };

pub const IdBuffer = rb.Ringbuffer(u16);
pub const BodySet = ss.SparseSet(Body, u16);

pub const World = struct {
    ids: IdBuffer,
    bodies: BodySet,
    manifolds: []Manifold,
    manifold_count: u32,

    pub fn init(allocator: std.mem.Allocator, body_count: u16) !World {
        const ids = try IdBuffer.init(body_count, allocator);
        errdefer ids.deinit(allocator);
        const bodies = try BodySet.init(body_count, allocator);
        errdefer bodies.deinit(allocator);
        const manifolds = try allocator.alloc(Manifold, body_count * body_count / 2);
        errdefer allocator.free(manifolds);

        return World{
            .ids = try IdBuffer.init(body_count, allocator),
            .bodies = bodies,
            .manifolds = manifolds,
            .manifold_count = 0,
        };
    }

    pub fn deinit(self: *World, allocator: std.mem.Allocator) void {
        self.ids.deinit(allocator);
        self.bodies.deinit(allocator);
        allocator.free(self.manifolds);
    }

    pub fn update(self: *World, dt: f32) void {
        // Apply Forces
        for (0..self.bodies.size) |i| {
            const body = &self.bodies.dense[i];
            // Recalculate Mass
            body.recalculateMass();
            // Integrate Forces
            body.velocity += body.force * dt;
            body.force = .{ 0, 0 };
        }

        self.broadPhase();
    }

    fn broadPhase(self: *World) void {
        var index: u32 = 0;
        for (0..self.bodies.size) |i| {
            for ((i + 1)..self.bodies.size) |j| {
                self.manifolds[index] = Manifold{
                    .a = self.bodies.dense[i],
                    .b = self.bodies.dense[j],
                };
                index += 1;
            }
        }
        self.manifold_count = index;
    }
};

// Updates the physics world at a fixed rate for determinism
pub const FixedStepUpdater = struct {
    const max_accumulated: f32 = 5; // maximum number of frames to accumulate
    dt: f32,
    accumulator: f32,

    pub fn init(dt: f32) FixedStepUpdater {
        return FixedStepUpdater{
            .dt = dt,
            .accumulator = 0,
        };
    }

    pub fn update(self: *FixedStepUpdater, world: *World, frame_time: f32) void {
        self.accumulator += frame_time;
        // Prevent Death Spiral
        if (self.accumulator > max_accumulated * self.dt)
            self.accumulator = max_accumulated * self.dt;
        // Update the world
        while (self.accumulator > self.dt) : (self.accumulator -= self.dt) {
            world.update(self.dt);
        }
    }
};
