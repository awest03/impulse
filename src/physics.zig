const rb = @import("ringbuffer.zig");
const ss = @import("sparseset.zig");
const std = @import("std");

fn pythagoras(a: f32, b: f32) f32 {
    return @sqrt(a * a + b * b);
}

pub const Vec2 = @Vector(2, f32);

fn approxEqVec2(a: Vec2, b: Vec2) bool {
    const threshold: f32 = 0.0001;
    return @abs(a[0] - b[0]) < threshold and @abs(a[1] - b[1]) < threshold;
}

fn dotVec2(a: Vec2, b: Vec2) f32 {
    return @reduce(std.builtin.ReduceOp.Add, a * b);
}

fn normaliseVec2(a: Vec2) Vec2 {
    return scaleVec2(a, 1.0 / @sqrt(dotVec2(a, a)));
}

fn scaleVec2(a: Vec2, scale: f32) Vec2 {
    return a * @as(Vec2, @splat(scale));
}

pub const Material = struct {
    restitution: f32,
    dynamic_friction: f32,
    static_friction: f32,
    density: f32,
};

pub const AABB = struct {
    half_extents: Vec2,

    pub fn area(self: *const @This()) f32 {
        return 4 * @reduce(std.builtin.ReduceOp.Mul, self.half_extents);
    }
};

pub const Circle = struct {
    radius: f32,

    pub fn area(self: *const @This()) f32 {
        return std.math.pi * self.radius * self.radius;
    }
};

pub const Shape = union(enum) {
    aabb: AABB,
    circle: Circle,

    pub fn area(self: *const @This()) f32 {
        switch (self.*) {
            inline else => |case| return case.area(),
        }
    }
};

pub const Body = struct {
    shape: Shape,
    material: *const Material,
    // Linear Properties
    inv_mass: f32,
    position: Vec2,
    velocity: Vec2,
    force: Vec2,

    pub fn recalculateMass(self: *Body) void {
        self.inv_mass = 1.0 / (self.material.density * self.shape.area());
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

    pub fn calculate(self: *Manifold) bool {
        return switch (self.a.shape) {
            Shape.aabb => switch (self.b.shape) {
                Shape.aabb => self.calculateAABBvsAABB(),
                Shape.circle => self.calculateAABBvsCircle(),
            },
            Shape.circle => switch (self.b.shape) {
                Shape.aabb => self.calculateCirclevsAABB(),
                Shape.circle => self.calculateCirclevsCircle(),
            },
        };
    }

    pub fn resolve(self: *Manifold) void {
        var a = self.a;
        var b = self.b;
        const ma = a.material;
        const mb = b.material;

        var rv = b.velocity - a.velocity;
        const v_normal = dotVec2(rv, self.normal);
        if (v_normal > 0.0) return; // Do nothing, velocities are separating
        const e = @min(ma.restitution, mb.restitution);
        const j = -(1 + e) * v_normal / (a.inv_mass + b.inv_mass);
        const impulse = scaleVec2(self.normal, j);
        a.velocity -= scaleVec2(impulse, a.inv_mass);
        b.velocity += scaleVec2(impulse, b.inv_mass);

        // Apply Friction
        rv = b.velocity - a.velocity;
        const tangent = normaliseVec2(rv - (scaleVec2(self.normal, dotVec2(rv, self.normal))));
        const jt = -dotVec2(rv, tangent) / (a.inv_mass + b.inv_mass);
        const mu = pythagoras(ma.static_friction, mb.static_friction);
        const friction_impulse = if (@abs(jt) < j * mu) scaleVec2(tangent, jt) else label: {
            const dynamic_friction = pythagoras(ma.dynamic_friction, mb.dynamic_friction);
            break :label scaleVec2(tangent, -j * dynamic_friction);
        };
        a.velocity -= scaleVec2(friction_impulse, a.inv_mass);
        b.velocity += scaleVec2(friction_impulse, b.inv_mass);
    }

    pub fn positionalCorrection(self: *Manifold) void {
        var a = self.a;
        var b = self.b;

        const percent: f32 = 0.4;
        const slop: f32 = 0.05;

        const correction = scaleVec2(self.normal, percent * @max(self.penetration - slop, 0) / (a.inv_mass + b.inv_mass));
        a.position -= scaleVec2(correction, a.inv_mass);
        b.position += scaleVec2(correction, b.inv_mass);
    }

    fn calculateAABBvsAABB(manifold: *Manifold) bool {
        const a = manifold.a;
        const b = manifold.b;

        const n = b.position - a.position;
        const x_overlap = a.shape.aabb.half_extents[0] + b.shape.aabb.half_extents[0] - @abs(n[0]);
        if (x_overlap > 0.0) {
            const y_overlap = a.shape.aabb.half_extents[1] + b.shape.aabb.half_extents[1] - @abs(n[1]);
            if (y_overlap > 0.0) {
                if (x_overlap < y_overlap) {
                    manifold.normal = Vec2{ if (n[0] < 0.0) -1.0 else 1.0, 0.0 };
                    manifold.penetration = x_overlap;
                } else {
                    manifold.normal = Vec2{ 0.0, if (n[1] < 0.0) -1.0 else 1.0 };
                    manifold.penetration = y_overlap;
                }
                return true;
            }
        }
        return false;
    }

    fn calculateAABBvsCircle(manifold: *Manifold) bool {
        const a = manifold.a;
        const b = manifold.b;

        const n = b.position - a.position;
        const half_extents = a.shape.aabb.half_extents;
        var closest = Vec2{ std.math.clamp(n[0], -half_extents[0], half_extents[0]), std.math.clamp(n[1], -half_extents[1], half_extents[1]) };
        const inside = if (approxEqVec2(n, closest)) label: {
            if (@abs(n[0]) > @abs(n[1])) { // Find closest axis
                closest[0] = if (closest[0] > 0.0) half_extents[0] else -half_extents[0];
            } else {
                closest[1] = if (closest[1] > 0.0) half_extents[1] else -half_extents[1];
            }
            break :label true;
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

    fn calculateCirclevsAABB(manifold: *Manifold) bool {
        // Rather than write out the function twice
        manifold.swapBodies();
        defer manifold.swapBodies();
        defer manifold.normal = -manifold.normal;
        return calculateAABBvsCircle(manifold);
    }

    fn calculateCirclevsCircle(manifold: *Manifold) bool {
        const a = manifold.a;
        const b = manifold.b;

        const n = b.position - a.position;
        const r = a.shape.circle.radius + b.shape.circle.radius;
        const d2 = dotVec2(n, n);
        if (d2 > r * r) return false;

        const d = @sqrt(d2);
        if (d != 0.0) {
            manifold.penetration = r - d;
            manifold.normal = scaleVec2(n, 1.0 / d);
        } else {
            manifold.penetration = a.shape.circle.radius;
            manifold.normal = Vec2{ 1.0, 0.0 };
        }
        return true;
    }
};

pub const IdBuffer = rb.Ringbuffer(usize);
pub const BodySet = ss.SparseSet(Body, usize);

pub const World = struct {
    ids: IdBuffer,
    bodies: BodySet,
    manifolds: []Manifold,
    manifold_count: usize,

    pub fn init(allocator: std.mem.Allocator, body_count: usize) !World {
        var ids = try IdBuffer.init(body_count, allocator);
        errdefer ids.deinit(allocator);
        var bodies = try BodySet.init(body_count, allocator);
        errdefer bodies.deinit(allocator);
        const manifolds = try allocator.alloc(Manifold, body_count * body_count / 2);
        errdefer allocator.free(manifolds);

        for (0..body_count) |i| {
            try ids.push(i);
        }

        return World{
            .ids = ids,
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

    pub fn createBody(self: *World, position: Vec2, material: *const Material, shape: Shape) !usize {
        const id = self.ids.pop().?;
        try self.bodies.insert(id, Body{
            .shape = shape,
            .material = material,
            .inv_mass = 0,
            .position = position,
            .velocity = @splat(0),
            .force = @splat(0),
        });
        (try self.bodies.get(id)).recalculateMass();
        return id;
    }

    pub fn update(self: *World, dt: f32) void {
        // Apply Forces
        for (0..self.bodies.size) |i| {
            var body = &self.bodies.dense[i];
            // Recalculate Mass
            body.recalculateMass();
            // Integrate Forces
            body.velocity += scaleVec2(body.force, body.inv_mass * dt);
            body.force = @splat(0.0);
        }
        self.broadPhase();
        self.calculateManifolds();
        // Resolve Collisions
        for (0..self.manifold_count) |i| {
            self.manifolds[i].resolve();
        }
        // Apply Velocities
        for (0..self.bodies.size) |i| {
            var body = &self.bodies.dense[i];
            body.position += scaleVec2(body.velocity, dt);
        }
        // Apply Positional Correction
        for (0..self.manifold_count) |i| {
            self.manifolds[i].positionalCorrection();
        }
    }

    fn broadPhase(self: *World) void {
        var index: u32 = 0;
        for (0..self.bodies.size) |i| {
            for ((i + 1)..self.bodies.size) |j| {
                self.manifolds[index] = Manifold{ .a = &self.bodies.dense[i], .b = &self.bodies.dense[j], .penetration = 0.0, .normal = @splat(0.0) };
                index += 1;
            }
        }
        self.manifold_count = index;
    }

    fn calculateManifolds(self: *World) void {
        var i: u32 = 0;
        while (i < self.manifold_count) {
            var m = self.manifolds[i];
            if (!m.calculate()) { // No collision, remove manifold
                self.manifold_count -= 1;
                if (self.manifold_count == 0) break;
                self.manifolds[i] = self.manifolds[self.manifold_count];
                continue;
            }
            i += 1;
        }
    }
};

// Updates the physics world at a fixed rate for determinism
pub const FixedStepUpdater = struct {
    const max_accumulated: f32 = 5.0; // maximum number of frames to accumulate
    dt: f32,
    accumulator: f32,

    pub fn init(dt: f32) FixedStepUpdater {
        return FixedStepUpdater{
            .dt = dt,
            .accumulator = 0.0,
        };
    }

    pub fn update(self: *FixedStepUpdater, world: *World, frame_time: f32) void {
        self.accumulator += frame_time;
        // Prevent Death Spiral
        if (self.accumulator > max_accumulated * self.dt)
            self.accumulator = max_accumulated * self.dt;
        // Update the world
        while (self.accumulator > self.dt) {
            world.update(self.dt);
            self.accumulator -= self.dt;
        }
    }
};
