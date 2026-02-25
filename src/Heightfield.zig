//! Heightfield collision geometry: a regular 2D grid of height samples forming terrain.
//! Build a `Data` handle from height arrays or a callback, then create a Heightfield geom from it.

const c = @import("c.zig").c;
const Real = c.dReal;
const Geom = @import("Geom.zig");
const Space = @import("Space.zig");

/// Stores the height sample data. One Data can be shared by multiple Heightfield geoms.
pub const Data = struct {
    id: c.dHeightfieldDataID,

    pub fn create() Data {
        return .{ .id = c.dGeomHeightfieldDataCreate() };
    }

    pub fn destroy(self: Data) void {
        c.dGeomHeightfieldDataDestroy(self.id);
    }

    /// Signature for the per-sample callback used by `build_callback`.
    /// Returns the height at grid coordinates (x, z).
    pub const HeightCallback = *const fn (userdata: ?*anyopaque, x: c_int, z: c_int) callconv(.c) Real;

    /// Build height data from a callback that returns the height at each grid cell.
    /// `width`/`depth` are the world-space dimensions, `width_samples`/`depth_samples` are the grid resolution.
    /// `scale` multiplies the callback return value, `offset` shifts it vertically.
    /// `thickness` adds a solid shell below the surface for objects that tunnel through.
    /// `wrap` tiles the heightfield infinitely if true.
    pub fn build_callback(
        self: Data,
        userdata: ?*anyopaque,
        callback: HeightCallback,
        width: Real,
        depth: Real,
        width_samples: c_int,
        depth_samples: c_int,
        scale: Real,
        offset: Real,
        thickness: Real,
        wrap: bool,
    ) void {
        c.dGeomHeightfieldDataBuildCallback(self.id, userdata, callback, width, depth, width_samples, depth_samples, scale, offset, thickness, @intFromBool(wrap));
    }

    /// Build height data from an array of u8 height samples. If `copy` is true, ODE
    /// makes an internal copy; otherwise the pointer must remain valid for the Data's lifetime.
    pub fn build_byte(
        self: Data,
        height_data: [*]const u8,
        copy: bool,
        width: Real,
        depth: Real,
        width_samples: c_int,
        depth_samples: c_int,
        scale: Real,
        offset: Real,
        thickness: Real,
        wrap: bool,
    ) void {
        c.dGeomHeightfieldDataBuildByte(self.id, height_data, @intFromBool(copy), width, depth, width_samples, depth_samples, scale, offset, thickness, @intFromBool(wrap));
    }

    /// Build height data from an array of i16 height samples.
    pub fn build_short(
        self: Data,
        height_data: [*]const i16,
        copy: bool,
        width: Real,
        depth: Real,
        width_samples: c_int,
        depth_samples: c_int,
        scale: Real,
        offset: Real,
        thickness: Real,
        wrap: bool,
    ) void {
        c.dGeomHeightfieldDataBuildShort(self.id, height_data, @intFromBool(copy), width, depth, width_samples, depth_samples, scale, offset, thickness, @intFromBool(wrap));
    }

    /// Build height data from an array of f32 height samples.
    pub fn build_single(
        self: Data,
        height_data: [*]const f32,
        copy: bool,
        width: Real,
        depth: Real,
        width_samples: c_int,
        depth_samples: c_int,
        scale: Real,
        offset: Real,
        thickness: Real,
        wrap: bool,
    ) void {
        c.dGeomHeightfieldDataBuildSingle(self.id, height_data, @intFromBool(copy), width, depth, width_samples, depth_samples, scale, offset, thickness, @intFromBool(wrap));
    }

    /// Build height data from an array of f64 height samples.
    pub fn build_double(
        self: Data,
        height_data: [*]const f64,
        copy: bool,
        width: Real,
        depth: Real,
        width_samples: c_int,
        depth_samples: c_int,
        scale: Real,
        offset: Real,
        thickness: Real,
        wrap: bool,
    ) void {
        c.dGeomHeightfieldDataBuildDouble(self.id, height_data, @intFromBool(copy), width, depth, width_samples, depth_samples, scale, offset, thickness, @intFromBool(wrap));
    }

    /// Override the automatically computed min/max height bounds. Useful when the AABB
    /// computed from sample data is too conservative.
    pub fn set_bounds(self: Data, min_height: Real, max_height: Real) void {
        c.dGeomHeightfieldDataSetBounds(self.id, min_height, max_height);
    }
};

id: c.dGeomID,

const Self = @This();

/// Create a heightfield geom from pre-built Data. If `placeable` is true the heightfield
/// can be positioned/rotated freely; otherwise it is fixed at the origin (more efficient).
pub fn create(space: ?Space.Generic, data: Data, placeable: bool) Self {
    return .{ .id = c.dCreateHeightfield(
        if (space) |s| s.id else null,
        data.id,
        @intFromBool(placeable),
    ) };
}

/// Replace the height data used by this geom.
pub fn set_data(self: Self, data: Data) void {
    c.dGeomHeightfieldSetHeightfieldData(self.id, data.id);
}

/// Get the height data currently used by this geom.
pub fn get_data(self: Self) Data {
    return .{ .id = c.dGeomHeightfieldGetHeightfieldData(self.id) };
}

/// Convert to a generic Geom handle for use with collision functions and space operations.
pub fn to_geom(self: Self) Geom.Generic {
    return .{ .id = self.id };
}
