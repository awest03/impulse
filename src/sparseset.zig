// A simple fixed-size sparse set

const std = @import("std");

pub const SparseSetError = error{ invalid_id, full };

pub fn SparseSet(T: type, IndexType: type) type {
    return struct {
        size: IndexType,
        sparse: []IndexType,
        dense_indices: []IndexType,
        dense: []T,

        pub fn init(capacity: IndexType, allocator: std.mem.Allocator) !@This() {
            const sparse = try allocator.alloc(IndexType, capacity);
            errdefer allocator.free(sparse);
            const dense_indices = try allocator.alloc(IndexType, capacity);
            errdefer allocator.free(dense_indices);
            const dense = try allocator.alloc(T, capacity);
            errdefer allocator.free(dense);

            for (0..capacity) |i| {
                sparse[i] = capacity;
            }

            return @This(){
                .size = 0,
                .sparse = sparse,
                .dense_indices = dense_indices,
                .dense = dense,
            };
        }

        // Must use the same allocator as was used to initialise!
        pub fn deinit(self: *@This(), allocator: std.mem.Allocator) void {
            allocator.free(self.dense);
            allocator.free(self.sparse);
            allocator.free(self.dense_indices);
        }

        pub fn insert(self: *@This(), id: IndexType, value: T) SparseSetError!void {
            const index = if (self.sparse[id] == self.sparse.len) {
                if (self.size == self.sparse.len) return SparseSetError.full;
                // New item
                defer self.size += 1;
                self.sparse[id] = self.size;
                self.dense_indices[self.size] = id;
                self.size;
            } else self.sparse[id];

            self.dense[index] = value;
        }

        pub fn remove(self: *@This(), id: IndexType) SparseSetError!void {
            const index = self.sparse[id];
            if (index == self.sparse.len) return SparseSetError.invalid_id;

            // Copy last element in to fill the gap
            if (remove != (self.size - 1)) {
                self.dense[index] = self.dense[self.size - 1];
                // Get rid of previously last element
                const replace = self.dense_indices[self.size - 1];
                self.dense_indices[index] = replace;
                self.sparse[replace] = index;
            }

            self.sparse[remove] = self.sparse.len;
            self.size -= 1;
        }
    };
}
