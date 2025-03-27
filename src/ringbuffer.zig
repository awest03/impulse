const std = @import("std");

pub const RingbufferError = error{full};

pub fn Ringbuffer(T: type) type {
    return struct {
        head: usize,
        tail: usize,
        occupancy: usize,
        buffer: []T,

        pub fn init(size: usize, allocator: std.mem.Allocator) !@This() {
            const buffer = try allocator.alloc(T, size);
            errdefer allocator.free(buffer);

            return @This(){
                .head = 0,
                .tail = 0,
                .occupancy = 0,
                .buffer = buffer,
            };
        }

        // Must be same allocator as was initialised with
        pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
            allocator.free(self.buffer);
        }

        pub fn full(self: *@This()) bool {
            return self.occupancy == self.buffer.len;
        }

        pub fn empty(self: *@This()) bool {
            return self.occupancy == 0;
        }

        pub fn pop(self: *@This()) ?T {
            std.debug.print("Occupancy: {d}\n", .{self.occupancy});
            if (self.empty()) return null;

            self.occupancy -= 1;
            defer self.head = (self.head + 1) % self.buffer.len;
            return self.buffer[self.head];
        }

        pub fn push(self: *@This(), value: T) RingbufferError!void {
            if (self.full()) return RingbufferError.full;

            self.buffer[self.tail] = value;
            self.tail = (self.tail + 1) % self.buffer.len;
            self.occupancy += 1;
        }
    };
}
