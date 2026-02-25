//! Collision geometry (geom) types and operations.
//!
//! Geoms are the fundamental collision shapes in ODE. They can be added to a Space
//! for broad-phase collision culling, attached to a Body to follow its motion, or
//! placed statically in the world. Each concrete type (Sphere, Box, etc.) wraps a
//! type-erased `Generic` handle and provides shape-specific methods alongside the
//! common geom interface.

const c = @import("c.zig").c;
const Real = c.dReal;
const Body = @import("Body.zig");
const Space = @import("Space.zig");

/// Returns a struct of common geom methods parameterized for the given type.
/// Every geom struct (Generic, Sphere, Box, etc.) re-exports these as its own methods.
fn GeomMethods(comptime Self: type) type {
    return struct {
        /// Destroys the geom, removing it from any space it belongs to and freeing its resources.
        pub fn destroy(self: Self) void { c.dGeomDestroy(self.id); }
        /// Attaches this geom to a rigid body so it follows the body's motion, or pass null to detach.
        pub fn set_body(self: Self, body: ?Body) void { c.dGeomSetBody(self.id, if (body) |b| b.id else null); }
        /// Returns the body this geom is attached to, or null if it is a static geom.
        pub fn get_body(self: Self) ?Body { const b = c.dGeomGetBody(self.id); return if (b != null) .{ .id = b } else null; }
        /// Sets the geom's position in world coordinates.
        pub fn set_position(self: Self, pos: [3]Real) void { c.dGeomSetPosition(self.id, pos[0], pos[1], pos[2]); }
        /// Returns the geom's position in world coordinates.
        pub fn get_position(self: Self) [3]Real { const p = c.dGeomGetPosition(self.id); return .{ p[0], p[1], p[2] }; }
        /// Sets the geom's orientation as a 3x4 rotation matrix (row-major, 12 elements).
        pub fn set_rotation(self: Self, r: *const [12]Real) void { c.dGeomSetRotation(self.id, r); }
        /// Returns a pointer to the geom's 3x4 rotation matrix (row-major, 12 elements).
        pub fn get_rotation(self: Self) *const [12]Real { return c.dGeomGetRotation(self.id); }
        /// Sets the geom's orientation as a quaternion [w, x, y, z].
        pub fn set_quaternion(self: Self, q: *const [4]Real) void { c.dGeomSetQuaternion(self.id, q); }
        /// Returns the geom's orientation as a quaternion [w, x, y, z].
        pub fn get_quaternion(self: Self) [4]Real { var q: c.dQuaternion = undefined; c.dGeomGetQuaternion(self.id, &q); return q; }
        /// Returns the axis-aligned bounding box as [minx, maxx, miny, maxy, minz, maxz].
        pub fn get_aabb(self: Self) [6]Real { var aabb: [6]Real = undefined; c.dGeomGetAABB(self.id, &aabb); return aabb; }
        /// Stores an arbitrary user-data pointer on the geom.
        pub fn set_data(self: Self, data: ?*anyopaque) void { c.dGeomSetData(self.id, data); }
        /// Retrieves the user-data pointer previously set with `set_data`.
        pub fn get_data(self: Self) ?*anyopaque { return c.dGeomGetData(self.id); }
        /// Returns true if this geom handle actually represents a Space (spaces are also geoms in ODE).
        pub fn is_space(self: Self) bool { return c.dGeomIsSpace(self.id) != 0; }
        /// Returns the space this geom belongs to, or null if it has not been added to any space.
        pub fn get_space(self: Self) ?Space.Generic { const s = c.dGeomGetSpace(self.id); return if (s != null) .{ .id = s } else null; }
        /// Returns the ODE class identifier for this geom's shape type.
        pub fn get_class(self: Self) c_int { return c.dGeomGetClass(self.id); }
        /// Sets the category bitmask that identifies which collision group(s) this geom belongs to.
        pub fn set_category_bits(self: Self, bits: c_ulong) void { c.dGeomSetCategoryBits(self.id, bits); }
        /// Returns the category bitmask identifying which collision group(s) this geom belongs to.
        pub fn get_category_bits(self: Self) c_ulong { return c.dGeomGetCategoryBits(self.id); }
        /// Sets the collide bitmask controlling which categories this geom can collide with.
        pub fn set_collide_bits(self: Self, bits: c_ulong) void { c.dGeomSetCollideBits(self.id, bits); }
        /// Returns the collide bitmask controlling which categories this geom can collide with.
        pub fn get_collide_bits(self: Self) c_ulong { return c.dGeomGetCollideBits(self.id); }
        /// Enables this geom so it participates in collision detection.
        pub fn enable(self: Self) void { c.dGeomEnable(self.id); }
        /// Disables this geom so it is skipped during collision detection.
        pub fn disable(self: Self) void { c.dGeomDisable(self.id); }
        /// Returns true if this geom is enabled for collision detection.
        pub fn is_enabled(self: Self) bool { return c.dGeomIsEnabled(self.id) != 0; }
        /// Sets the geom's position offset relative to its attached body (local frame).
        pub fn set_offset_position(self: Self, pos: [3]Real) void { c.dGeomSetOffsetPosition(self.id, pos[0], pos[1], pos[2]); }
        /// Returns the geom's position offset relative to its attached body (local frame).
        pub fn get_offset_position(self: Self) [3]Real { const p = c.dGeomGetOffsetPosition(self.id); return .{ p[0], p[1], p[2] }; }
        /// Sets the geom's rotation offset relative to its attached body as a 3x4 matrix.
        pub fn set_offset_rotation(self: Self, r: *const [12]Real) void { c.dGeomSetOffsetRotation(self.id, r); }
        /// Returns a pointer to the geom's rotation offset relative to its attached body.
        pub fn get_offset_rotation(self: Self) *const [12]Real { return c.dGeomGetOffsetRotation(self.id); }
        /// Sets the geom's rotation offset relative to its attached body as a quaternion.
        pub fn set_offset_quaternion(self: Self, q: *const [4]Real) void { c.dGeomSetOffsetQuaternion(self.id, q); }
        /// Returns the geom's rotation offset relative to its attached body as a quaternion.
        pub fn get_offset_quaternion(self: Self) [4]Real { var q: c.dQuaternion = undefined; c.dGeomGetOffsetQuaternion(self.id, &q); return q; }
        /// Sets the geom's offset so that it ends up at the given world position (computes the local offset from the body).
        pub fn set_offset_world_position(self: Self, pos: [3]Real) void { c.dGeomSetOffsetWorldPosition(self.id, pos[0], pos[1], pos[2]); }
        /// Sets the geom's offset so that it ends up at the given world rotation (computes the local offset from the body).
        pub fn set_offset_world_rotation(self: Self, r: *const [12]Real) void { c.dGeomSetOffsetWorldRotation(self.id, r); }
        /// Sets the geom's offset so that it ends up at the given world quaternion (computes the local offset from the body).
        pub fn set_offset_world_quaternion(self: Self, q: *const [4]Real) void { c.dGeomSetOffsetWorldQuaternion(self.id, q); }
        /// Removes any body-relative offset, resetting the geom to be centered on its body.
        pub fn clear_offset(self: Self) void { c.dGeomClearOffset(self.id); }
        /// Returns true if this geom has a body-relative offset transform applied.
        pub fn is_offset(self: Self) bool { return c.dGeomIsOffset(self.id) != 0; }
        /// Converts this typed geom handle into a type-erased `Generic` handle.
        pub fn to_generic(self: Self) Generic { return .{ .id = self.id }; }
    };
}

/// Type-erased geom handle. Can represent any collision shape. Use this when
/// the concrete shape type is unknown or irrelevant (e.g., in callbacks).
pub const Generic = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Generic);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
};

/// Sphere collision shape defined by a center and radius.
pub const Sphere = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Sphere);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a sphere geom with the given radius, optionally inserting it into a space.
    pub fn create(space: ?Space.Generic, radius: Real) Sphere {
        return .{ .id = c.dCreateSphere(if (space) |s| s.id else null, radius) };
    }

    /// Changes the sphere's radius.
    pub fn set_radius(self: Sphere, radius: Real) void {
        c.dGeomSphereSetRadius(self.id, radius);
    }

    /// Returns the sphere's radius.
    pub fn get_radius(self: Sphere) Real {
        return c.dGeomSphereGetRadius(self.id);
    }

    /// Returns how deep the point `p` is inside the sphere (positive = inside, negative = outside).
    pub fn point_depth(self: Sphere, p: [3]Real) Real {
        return c.dGeomSpherePointDepth(self.id, p[0], p[1], p[2]);
    }
};

/// Axis-aligned box collision shape defined by side lengths along each axis.
pub const Box = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Box);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a box geom with the given side lengths (lx, ly, lz), optionally inserting it into a space.
    pub fn create(space: ?Space.Generic, lx: Real, ly: Real, lz: Real) Box {
        return .{ .id = c.dCreateBox(if (space) |s| s.id else null, lx, ly, lz) };
    }

    /// Changes the box's side lengths.
    pub fn set_lengths(self: Box, lx: Real, ly: Real, lz: Real) void {
        c.dGeomBoxSetLengths(self.id, lx, ly, lz);
    }

    /// Returns the box's side lengths as [lx, ly, lz].
    pub fn get_lengths(self: Box) [3]Real {
        var result: c.dVector3 = undefined;
        c.dGeomBoxGetLengths(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Returns how deep the point `p` is inside the box (positive = inside, negative = outside).
    pub fn point_depth(self: Box, p: [3]Real) Real {
        return c.dGeomBoxPointDepth(self.id, p[0], p[1], p[2]);
    }
};

/// Capsule (capped cylinder) collision shape -- a cylinder with hemispherical end caps,
/// defined by a radius and the length of the cylindrical section along the local Z axis.
pub const Capsule = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Capsule);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a capsule geom with the given radius and cylinder length, optionally inserting it into a space.
    pub fn create(space: ?Space.Generic, radius: Real, length: Real) Capsule {
        return .{ .id = c.dCreateCapsule(if (space) |s| s.id else null, radius, length) };
    }

    /// Changes the capsule's radius and cylinder section length.
    pub fn set_params(self: Capsule, radius: Real, length: Real) void {
        c.dGeomCapsuleSetParams(self.id, radius, length);
    }

    /// Returns the capsule's radius and cylinder section length.
    pub fn get_params(self: Capsule) struct { radius: Real, length: Real } {
        var radius: Real = undefined;
        var length: Real = undefined;
        c.dGeomCapsuleGetParams(self.id, &radius, &length);
        return .{ .radius = radius, .length = length };
    }

    /// Returns how deep the point `p` is inside the capsule (positive = inside, negative = outside).
    pub fn point_depth(self: Capsule, p: [3]Real) Real {
        return c.dGeomCapsulePointDepth(self.id, p[0], p[1], p[2]);
    }
};

/// Flat-ended cylinder collision shape defined by a radius and length along the local Z axis.
/// Unlike a Capsule, this has flat circular end caps rather than hemispherical ones.
pub const Cylinder = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Cylinder);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a cylinder geom with the given radius and length, optionally inserting it into a space.
    pub fn create(space: ?Space.Generic, radius: Real, length: Real) Cylinder {
        return .{ .id = c.dCreateCylinder(if (space) |s| s.id else null, radius, length) };
    }

    /// Changes the cylinder's radius and length.
    pub fn set_params(self: Cylinder, radius: Real, length: Real) void {
        c.dGeomCylinderSetParams(self.id, radius, length);
    }

    /// Returns the cylinder's radius and length.
    pub fn get_params(self: Cylinder) struct { radius: Real, length: Real } {
        var radius: Real = undefined;
        var length: Real = undefined;
        c.dGeomCylinderGetParams(self.id, &radius, &length);
        return .{ .radius = radius, .length = length };
    }
};

/// Infinite, non-placeable plane defined by the equation ax + by + cz = d.
/// Planes are always static (cannot be attached to a body) and have no position
/// or rotation -- they are specified entirely by their normal (a, b, c) and distance d.
pub const Plane = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Plane);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a plane geom defined by the equation ax + by + cz = d, optionally inserting it into a space.
    /// The normal (a, b, c) does not need to be unit-length -- ODE will normalize it.
    pub fn create(space: ?Space.Generic, a: Real, b: Real, _c: Real, d: Real) Plane {
        return .{ .id = c.dCreatePlane(if (space) |s| s.id else null, a, b, _c, d) };
    }

    /// Changes the plane equation parameters (a, b, c, d).
    pub fn set_params(self: Plane, a: Real, b: Real, _c: Real, d: Real) void {
        c.dGeomPlaneSetParams(self.id, a, b, _c, d);
    }

    /// Returns the plane equation parameters as [a, b, c, d].
    pub fn get_params(self: Plane) [4]Real {
        var result: c.dVector4 = undefined;
        c.dGeomPlaneGetParams(self.id, &result);
        return result;
    }

    /// Returns the signed distance from the point `p` to the plane surface (positive = on the normal side).
    pub fn point_depth(self: Plane, p: [3]Real) Real {
        return c.dGeomPlanePointDepth(self.id, p[0], p[1], p[2]);
    }
};

/// Ray collision shape used for raycasting. Defined by an origin, direction, and length.
/// Rays are one-directional and infinitely thin.
pub const Ray = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Ray);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a ray geom with the given maximum length, optionally inserting it into a space.
    pub fn create(space: ?Space.Generic, length: Real) Ray {
        return .{ .id = c.dCreateRay(if (space) |s| s.id else null, length) };
    }

    /// Changes the ray's maximum cast length.
    pub fn set_length(self: Ray, length: Real) void {
        c.dGeomRaySetLength(self.id, length);
    }

    /// Returns the ray's maximum cast length.
    pub fn get_length(self: Ray) Real {
        return c.dGeomRayGetLength(self.id);
    }

    /// Sets the ray's origin point and direction vector simultaneously.
    pub fn set(self: Ray, origin: [3]Real, direction: [3]Real) void {
        c.dGeomRaySet(self.id, origin[0], origin[1], origin[2], direction[0], direction[1], direction[2]);
    }

    /// Returns the ray's origin point and direction vector.
    pub fn get(self: Ray) struct { origin: [3]Real, direction: [3]Real } {
        var start: c.dVector3 = undefined;
        var dir: c.dVector3 = undefined;
        c.dGeomRayGet(self.id, &start, &dir);
        return .{
            .origin = .{ start[0], start[1], start[2] },
            .direction = .{ dir[0], dir[1], dir[2] },
        };
    }

    /// When true, the ray stops at the first contact instead of searching for all contacts.
    pub fn set_first_contact(self: Ray, first_contact: bool) void {
        c.dGeomRaySetFirstContact(self.id, @intFromBool(first_contact));
    }

    /// Returns whether first-contact mode is enabled.
    pub fn get_first_contact(self: Ray) bool {
        return c.dGeomRayGetFirstContact(self.id) != 0;
    }

    /// When true, contacts with backfacing triangles are ignored during raycasting.
    pub fn set_backface_cull(self: Ray, backface_cull: bool) void {
        c.dGeomRaySetBackfaceCull(self.id, @intFromBool(backface_cull));
    }

    /// Returns whether backface culling is enabled for this ray.
    pub fn get_backface_cull(self: Ray) bool {
        return c.dGeomRayGetBackfaceCull(self.id) != 0;
    }

    /// When true, only the closest hit along the ray is reported.
    pub fn set_closest_hit(self: Ray, closest_hit: bool) void {
        c.dGeomRaySetClosestHit(self.id, @intFromBool(closest_hit));
    }

    /// Returns whether closest-hit mode is enabled.
    pub fn get_closest_hit(self: Ray) bool {
        return c.dGeomRayGetClosestHit(self.id) != 0;
    }
};

/// Convex hull collision shape defined by a set of bounding planes, vertices, and
/// polygon connectivity. Suitable for arbitrary convex polyhedra.
pub const Convex = struct {
    id: c.dGeomID,

    const methods = GeomMethods(Convex);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a convex hull geom from bounding planes, vertex positions, and polygon index data.
    /// `polygons` encodes face connectivity: each face is prefixed by its vertex count followed by indices.
    pub fn create(
        space: ?Space.Generic,
        planes: [*]const Real,
        plane_count: c_uint,
        points: [*]const Real,
        point_count: c_uint,
        polygons: [*]const c_uint,
    ) Convex {
        return .{ .id = c.dCreateConvex(
            if (space) |s| s.id else null,
            planes,
            plane_count,
            points,
            point_count,
            polygons,
        ) };
    }

    /// Replaces this convex hull's geometry data (planes, points, and polygon connectivity).
    pub fn set_convex(
        self: Convex,
        planes: [*]const Real,
        plane_count: c_uint,
        points: [*]const Real,
        point_count: c_uint,
        polygons: [*]const c_uint,
    ) void {
        c.dGeomSetConvex(self.id, planes, plane_count, points, point_count, polygons);
    }
};

/// Triangle mesh collision shape built from indexed vertex data.
/// Suitable for complex static environments or detailed collision geometry.
pub const TriMesh = struct {
    id: c.dGeomID,

    const methods = GeomMethods(TriMesh);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Opaque handle to pre-built triangle mesh data (vertices, indices, and optional normals).
    /// Must be created and populated before constructing a TriMesh geom.
    pub const Data = struct {
        id: c.dTriMeshDataID,

        /// Allocates a new empty trimesh data object.
        pub fn create() Data {
            return .{ .id = c.dGeomTriMeshDataCreate() };
        }

        /// Frees the trimesh data object and its internal storage.
        pub fn destroy_data(self: Data) void {
            c.dGeomTriMeshDataDestroy(self.id);
        }

        /// Fills the trimesh data from single-precision (float) vertex and index arrays.
        pub fn build_single(
            self: Data,
            vertices: *const anyopaque,
            vertex_stride: c_int,
            vertex_count: c_int,
            indices: *const anyopaque,
            index_count: c_int,
            tri_stride: c_int,
        ) void {
            c.dGeomTriMeshDataBuildSingle(self.id, vertices, vertex_stride, vertex_count, indices, index_count, tri_stride);
        }

        /// Like `build_single` but also accepts a per-triangle normals array for faster collision.
        pub fn build_single1(
            self: Data,
            vertices: *const anyopaque,
            vertex_stride: c_int,
            vertex_count: c_int,
            indices: *const anyopaque,
            index_count: c_int,
            tri_stride: c_int,
            normals: *const anyopaque,
        ) void {
            c.dGeomTriMeshDataBuildSingle1(self.id, vertices, vertex_stride, vertex_count, indices, index_count, tri_stride, normals);
        }

        /// Fills the trimesh data from double-precision (f64) vertex and index arrays.
        pub fn build_double(
            self: Data,
            vertices: *const anyopaque,
            vertex_stride: c_int,
            vertex_count: c_int,
            indices: *const anyopaque,
            index_count: c_int,
            tri_stride: c_int,
        ) void {
            c.dGeomTriMeshDataBuildDouble(self.id, vertices, vertex_stride, vertex_count, indices, index_count, tri_stride);
        }

        /// Preprocesses the mesh data to build internal acceleration structures. Returns true on success.
        pub fn preprocess(self: Data) bool {
            return c.dGeomTriMeshDataPreprocess(self.id) != 0;
        }
    };

    /// Creates a trimesh geom from pre-built mesh data, optionally inserting it into a space.
    pub fn create_trimesh(space: ?Space.Generic, data: Data) TriMesh {
        return .{ .id = c.dCreateTriMesh(
            if (space) |s| s.id else null,
            data.id,
            null,
            null,
            null,
        ) };
    }

    /// Replaces the trimesh data associated with this geom.
    pub fn set_trimesh_data(self: TriMesh, data: Data) void {
        c.dGeomTriMeshSetData(self.id, data.id);
    }

    /// Returns the trimesh data handle associated with this geom.
    pub fn get_trimesh_data(self: TriMesh) Data {
        return .{ .id = c.dGeomTriMeshGetData(self.id) };
    }

    /// Enables or disables temporal coherence caching for collisions against a specific geom class.
    /// This can speed up repeated collision checks between the same pair of geoms.
    pub fn enable_tc(self: TriMesh, geom_class: c_int, en: bool) void {
        c.dGeomTriMeshEnableTC(self.id, geom_class, @intFromBool(en));
    }

    /// Returns whether temporal coherence caching is enabled for the given geom class.
    pub fn is_tc_enabled(self: TriMesh, geom_class: c_int) bool {
        return c.dGeomTriMeshIsTCEnabled(self.id, geom_class) != 0;
    }

    /// Clears the temporal coherence cache, forcing fresh collision computation on the next step.
    pub fn clear_tc_cache(self: TriMesh) void {
        c.dGeomTriMeshClearTCCache(self.id);
    }

    /// Returns the total number of triangles in this trimesh.
    pub fn get_triangle_count(self: TriMesh) c_int {
        return c.dGeomTriMeshGetTriangleCount(self.id);
    }

    /// Computes a point on a triangle using barycentric coordinates (u, v).
    /// `index` is the triangle index; the returned point is in world coordinates.
    pub fn get_point(self: TriMesh, index: c_int, u: Real, v: Real) [3]Real {
        var out: c.dVector3 = undefined;
        c.dGeomTriMeshGetPoint(self.id, index, u, v, &out);
        return .{ out[0], out[1], out[2] };
    }
};

/// Deprecated wrapper that applies a relative transform to another geom.
/// Prefer using the body-relative offset functions (`set_offset_position`, etc.) instead.
pub const GeomTransform = struct {
    id: c.dGeomID,

    const methods = GeomMethods(GeomTransform);
    pub const destroy = methods.destroy;
    pub const set_body = methods.set_body;
    pub const get_body = methods.get_body;
    pub const set_position = methods.set_position;
    pub const get_position = methods.get_position;
    pub const set_rotation = methods.set_rotation;
    pub const get_rotation = methods.get_rotation;
    pub const set_quaternion = methods.set_quaternion;
    pub const get_quaternion = methods.get_quaternion;
    pub const get_aabb = methods.get_aabb;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const is_space = methods.is_space;
    pub const get_space = methods.get_space;
    pub const get_class = methods.get_class;
    pub const set_category_bits = methods.set_category_bits;
    pub const get_category_bits = methods.get_category_bits;
    pub const set_collide_bits = methods.set_collide_bits;
    pub const get_collide_bits = methods.get_collide_bits;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const set_offset_position = methods.set_offset_position;
    pub const get_offset_position = methods.get_offset_position;
    pub const set_offset_rotation = methods.set_offset_rotation;
    pub const get_offset_rotation = methods.get_offset_rotation;
    pub const set_offset_quaternion = methods.set_offset_quaternion;
    pub const get_offset_quaternion = methods.get_offset_quaternion;
    pub const set_offset_world_position = methods.set_offset_world_position;
    pub const set_offset_world_rotation = methods.set_offset_world_rotation;
    pub const set_offset_world_quaternion = methods.set_offset_world_quaternion;
    pub const clear_offset = methods.clear_offset;
    pub const is_offset = methods.is_offset;
    pub const to_generic = methods.to_generic;

    /// Creates a geom transform wrapper, optionally inserting it into a space.
    pub fn create_transform(space: ?Space.Generic) GeomTransform {
        return .{ .id = c.dCreateGeomTransform(if (space) |s| s.id else null) };
    }

    /// Sets the child geom whose collision shape is used, positioned relative to this transform.
    pub fn set_geom(self: GeomTransform, geom: Generic) void {
        c.dGeomTransformSetGeom(self.id, geom.id);
    }

    /// Returns the child geom wrapped by this transform.
    pub fn get_geom(self: GeomTransform) Generic {
        return .{ .id = c.dGeomTransformGetGeom(self.id) };
    }

    /// When cleanup is true, destroying this GeomTransform also destroys the child geom.
    pub fn set_cleanup(self: GeomTransform, mode: bool) void {
        c.dGeomTransformSetCleanup(self.id, @intFromBool(mode));
    }

    /// Returns whether auto-cleanup of the child geom is enabled.
    pub fn get_cleanup(self: GeomTransform) bool {
        return c.dGeomTransformGetCleanup(self.id) != 0;
    }

    /// Sets the info mode, controlling what information is returned for collisions (0 or 1).
    pub fn set_info(self: GeomTransform, mode: c_int) void {
        c.dGeomTransformSetInfo(self.id, mode);
    }

    /// Returns the current info mode.
    pub fn get_info(self: GeomTransform) c_int {
        return c.dGeomTransformGetInfo(self.id);
    }
};
