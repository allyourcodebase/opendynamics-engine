//! Collision spaces for broad-phase detection. A space contains geoms and quickly
//! determines which pairs are close enough to warrant narrow-phase testing.
//! Use `collide` to iterate over potentially-overlapping pairs.

const c = @import("c.zig").c;
const Real = c.dReal;
const Geom = @import("Geom.zig");
const collision = @import("collision.zig");

/// Type-erased space handle. Returned by `to_generic` on concrete space types,
/// and accepted by functions that operate on any kind of space.
pub const Generic = struct {
    id: c.dSpaceID,

    pub fn destroy(self: Generic) void { c.dSpaceDestroy(self.id); }
    /// If true, destroying the space also destroys all geoms inside it.
    pub fn set_cleanup(self: Generic, mode: bool) void { c.dSpaceSetCleanup(self.id, @intFromBool(mode)); }
    pub fn get_cleanup(self: Generic) bool { return c.dSpaceGetCleanup(self.id) != 0; }
    /// Set the nesting depth for hierarchical spaces (used internally for optimization).
    pub fn set_sublevel(self: Generic, sublevel: c_int) void { c.dSpaceSetSublevel(self.id, sublevel); }
    pub fn get_sublevel(self: Generic) c_int { return c.dSpaceGetSublevel(self.id); }
    /// When true, dirty geoms are not automatically re-indexed before collision; you must call `clean` manually.
    pub fn set_manual_cleanup(self: Generic, mode: bool) void { c.dSpaceSetManualCleanup(self.id, @intFromBool(mode)); }
    pub fn get_manual_cleanup(self: Generic) bool { return c.dSpaceGetManualCleanup(self.id) != 0; }
    /// Insert a geom into this space. A geom can only be in one space at a time.
    pub fn add(self: Generic, geom: Geom.Generic) void { c.dSpaceAdd(self.id, geom.id); }
    /// Remove a geom from this space.
    pub fn remove(self: Generic, geom: Geom.Generic) void { c.dSpaceRemove(self.id, geom.id); }
    /// Check whether a geom is contained in this space.
    pub fn query(self: Generic, geom: Geom.Generic) bool { return c.dSpaceQuery(self.id, geom.id) != 0; }
    /// Re-index all dirty geoms. Only needed when `manual_cleanup` is enabled.
    pub fn clean(self: Generic) void { c.dSpaceClean(self.id); }
    pub fn get_num_geoms(self: Generic) c_int { return c.dSpaceGetNumGeoms(self.id); }
    /// Access a geom by index (0-based). Order is not guaranteed to be stable.
    pub fn get_geom(self: Generic, i: c_int) Geom.Generic { return .{ .id = c.dSpaceGetGeom(self.id, i) }; }
    /// Returns the internal class identifier for this space type.
    pub fn get_class(self: Generic) c_int { return c.dSpaceGetClass(self.id); }

    /// Run broad-phase collision: invokes `callback` for each pair of geoms whose AABBs overlap.
    pub fn collide(self: Generic, data: ?*anyopaque, callback: collision.NearCallback) void {
        c.dSpaceCollide(self.id, data, callback);
    }

    /// Cast this space to a Geom handle. Spaces are themselves geoms in ODE, so they
    /// can be nested inside other spaces.
    pub fn to_geom(self: Generic) Geom.Generic {
        return .{ .id = @ptrCast(self.id) };
    }
};

/// O(n^2) brute-force space. Tests every pair — only suitable for small numbers of geoms.
pub const Simple = struct {
    id: c.dSpaceID,

    /// Create a simple space, optionally nested inside a parent space.
    pub fn create(space: ?Generic) Simple {
        return .{ .id = c.dSimpleSpaceCreate(if (space) |s| s.id else null) };
    }

    pub fn to_generic(self: Simple) Generic { return .{ .id = self.id }; }
    pub fn destroy(self: Simple) void { c.dSpaceDestroy(self.id); }
    pub fn set_cleanup(self: Simple, mode: bool) void { c.dSpaceSetCleanup(self.id, @intFromBool(mode)); }
    pub fn get_cleanup(self: Simple) bool { return c.dSpaceGetCleanup(self.id) != 0; }
    pub fn add(self: Simple, geom: Geom.Generic) void { c.dSpaceAdd(self.id, geom.id); }
    pub fn remove(self: Simple, geom: Geom.Generic) void { c.dSpaceRemove(self.id, geom.id); }
    pub fn get_num_geoms(self: Simple) c_int { return c.dSpaceGetNumGeoms(self.id); }
    pub fn get_geom(self: Simple, i: c_int) Geom.Generic { return .{ .id = c.dSpaceGetGeom(self.id, i) }; }

    pub fn collide(self: Simple, data: ?*anyopaque, callback: collision.NearCallback) void {
        c.dSpaceCollide(self.id, data, callback);
    }
};

/// Grid-based hash space. Geoms are binned into cells at multiple resolutions.
/// Good general-purpose choice for scenes with objects of varying sizes.
pub const Hash = struct {
    id: c.dSpaceID,

    /// Create a hash space, optionally nested inside a parent space.
    pub fn create(space: ?Generic) Hash {
        return .{ .id = c.dHashSpaceCreate(if (space) |s| s.id else null) };
    }

    pub fn to_generic(self: Hash) Generic { return .{ .id = self.id }; }
    pub fn destroy(self: Hash) void { c.dSpaceDestroy(self.id); }

    /// Set the minimum and maximum cell-size levels (as powers of 2). For example,
    /// levels (-3, 5) gives cell sizes from 2^-3 to 2^5.
    pub fn set_levels(self: Hash, minlevel: c_int, maxlevel: c_int) void {
        c.dHashSpaceSetLevels(self.id, minlevel, maxlevel);
    }

    pub fn get_levels(self: Hash) struct { min: c_int, max: c_int } {
        var min: c_int = undefined;
        var max: c_int = undefined;
        c.dHashSpaceGetLevels(self.id, &min, &max);
        return .{ .min = min, .max = max };
    }

    pub fn set_cleanup(self: Hash, mode: bool) void { c.dSpaceSetCleanup(self.id, @intFromBool(mode)); }
    pub fn get_cleanup(self: Hash) bool { return c.dSpaceGetCleanup(self.id) != 0; }
    pub fn add(self: Hash, geom: Geom.Generic) void { c.dSpaceAdd(self.id, geom.id); }
    pub fn remove(self: Hash, geom: Geom.Generic) void { c.dSpaceRemove(self.id, geom.id); }
    pub fn get_num_geoms(self: Hash) c_int { return c.dSpaceGetNumGeoms(self.id); }
    pub fn get_geom(self: Hash, i: c_int) Geom.Generic { return .{ .id = c.dSpaceGetGeom(self.id, i) }; }

    pub fn collide(self: Hash, data: ?*anyopaque, callback: collision.NearCallback) void {
        c.dSpaceCollide(self.id, data, callback);
    }
};

/// Quad-tree space for static scenes. Best when most geoms don't move, as reinsertion is expensive.
/// Requires a known bounding volume at creation.
pub const QuadTree = struct {
    id: c.dSpaceID,

    /// Create a quadtree space with a fixed bounding volume. `center` and `extents` define
    /// the root AABB, and `depth` controls the tree subdivision levels.
    pub fn create(space: ?Generic, center: [3]Real, extents: [3]Real, depth: c_int) QuadTree {
        return .{ .id = c.dQuadTreeSpaceCreate(
            if (space) |s| s.id else null,
            &.{ center[0], center[1], center[2], 0 },
            &.{ extents[0], extents[1], extents[2], 0 },
            depth,
        ) };
    }

    pub fn to_generic(self: QuadTree) Generic { return .{ .id = self.id }; }
    pub fn destroy(self: QuadTree) void { c.dSpaceDestroy(self.id); }
    pub fn set_cleanup(self: QuadTree, mode: bool) void { c.dSpaceSetCleanup(self.id, @intFromBool(mode)); }
    pub fn get_cleanup(self: QuadTree) bool { return c.dSpaceGetCleanup(self.id) != 0; }
    pub fn add(self: QuadTree, geom: Geom.Generic) void { c.dSpaceAdd(self.id, geom.id); }
    pub fn remove(self: QuadTree, geom: Geom.Generic) void { c.dSpaceRemove(self.id, geom.id); }
    pub fn get_num_geoms(self: QuadTree) c_int { return c.dSpaceGetNumGeoms(self.id); }
    pub fn get_geom(self: QuadTree, i: c_int) Geom.Generic { return .{ .id = c.dSpaceGetGeom(self.id, i) }; }

    pub fn collide(self: QuadTree, data: ?*anyopaque, callback: collision.NearCallback) void {
        c.dSpaceCollide(self.id, data, callback);
    }
};

/// Sweep-and-prune (SAP) space. Maintains sorted axis lists and is very efficient for
/// scenes where objects move incrementally between frames. Choose the axis order that
/// best distributes your objects.
pub const SweepAndPrune = struct {
    id: c.dSpaceID,

    /// Primary sort axis ordering. Choose based on which axis has the most spread.
    pub const AxisOrder = enum(c_int) {
        xyz = 0,
        xzy = 1,
        yxz = 2,
        yzx = 3,
        zxy = 4,
        zyx = 5,
    };

    /// Create a SAP space with the given sort axis order.
    pub fn create(space: ?Generic, axis_order: AxisOrder) SweepAndPrune {
        return .{ .id = c.dSweepAndPruneSpaceCreate(if (space) |s| s.id else null, @intFromEnum(axis_order)) };
    }

    pub fn to_generic(self: SweepAndPrune) Generic { return .{ .id = self.id }; }
    pub fn destroy(self: SweepAndPrune) void { c.dSpaceDestroy(self.id); }
    pub fn set_cleanup(self: SweepAndPrune, mode: bool) void { c.dSpaceSetCleanup(self.id, @intFromBool(mode)); }
    pub fn get_cleanup(self: SweepAndPrune) bool { return c.dSpaceGetCleanup(self.id) != 0; }
    pub fn add(self: SweepAndPrune, geom: Geom.Generic) void { c.dSpaceAdd(self.id, geom.id); }
    pub fn remove(self: SweepAndPrune, geom: Geom.Generic) void { c.dSpaceRemove(self.id, geom.id); }
    pub fn get_num_geoms(self: SweepAndPrune) c_int { return c.dSpaceGetNumGeoms(self.id); }
    pub fn get_geom(self: SweepAndPrune, i: c_int) Geom.Generic { return .{ .id = c.dSpaceGetGeom(self.id, i) }; }

    pub fn collide(self: SweepAndPrune, data: ?*anyopaque, callback: collision.NearCallback) void {
        c.dSpaceCollide(self.id, data, callback);
    }
};
