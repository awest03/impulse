const std = @import("std");

pub const RingbufferError = error{full};

pub fn Ringbuffer(T: type) type {
    return struct {
        head: usize,
        tail: usize,
        buffer: []T,

        pub fn init(size: usize, allocator: std.mem.Allocator) !@This() {
            const buffer = try allocator.alloc(T, size);
            errdefer allocator.free(buffer);

            return @This(){
                .head = 0,
                .tail = 0,
                .buffer = buffer,
            };
        }

        // Must be same allocator as was initialised with
        pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
            allocator.free(self.buffer);
        }

        pub fn full(self: *@This()) bool {
            return (self.tail + 1) % self.buffer.len == self.head;
        }

        pub fn empty(self: *@This()) bool {
            return self.head == self.tail;
        }

        pub fn pop(self: *@This()) ?T {
            if (self.empty()) return null;

            defer self.head = (self.head + 1) % self.buffer.len;
            return self.buffer[self.head];
        }

        pub fn push(self: *@This(), value: T) RingbufferError!void {
            if (self.full()) return RingbufferError.full;

            self.buffer[self.tail] = value;
            self.tail = (self.tail + 1) % self.buffer.len;
        }
    };
}
