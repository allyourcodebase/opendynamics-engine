//! Mass distribution properties for a rigid body: total mass, center of gravity, and
//! 3x3 inertia tensor. Create from a shape (sphere, box, etc.) or set directly.
//! This is a value type — methods take `*Self` because ODE mutates the struct in place.

const c = @import("c.zig").c;
const Real = c.dReal;
const Geom = @import("Geom.zig");

raw: c.dMass,

const Self = @This();

/// Create a zeroed-out mass (total mass = 0, identity-like inertia). Useful as a starting
/// point before combining multiple masses with `add`.
pub fn zero() Self {
    var m: c.dMass = undefined;
    c.dMassSetZero(&m);
    return .{ .raw = m };
}

/// Set all mass parameters explicitly: total mass, center of gravity offset, and the 6
/// unique elements of the symmetric 3x3 inertia tensor (I11, I22, I33, I12, I13, I23).
pub fn set_parameters(self: *Self, themass: Real, cg: [3]Real, inertia_11: Real, inertia_22: Real, inertia_33: Real, inertia_12: Real, inertia_13: Real, inertia_23: Real) void {
    c.dMassSetParameters(&self.raw, themass, cg[0], cg[1], cg[2], inertia_11, inertia_22, inertia_33, inertia_12, inertia_13, inertia_23);
}

/// Mass of a solid sphere given uniform density and radius.
pub fn sphere(density: Real, radius: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetSphere(&m, density, radius);
    return .{ .raw = m };
}

/// Mass of a solid sphere given total mass and radius (density is computed internally).
pub fn sphere_total(total_mass: Real, radius: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetSphereTotal(&m, total_mass, radius);
    return .{ .raw = m };
}

/// Mass of a capsule (cylinder with hemispherical caps). `direction` is the long axis:
/// 1 = X, 2 = Y, 3 = Z.
pub fn capsule(density: Real, direction: c_int, radius: Real, length: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetCapsule(&m, density, direction, radius, length);
    return .{ .raw = m };
}

/// Mass of a capsule given total mass instead of density.
pub fn capsule_total(total_mass: Real, direction: c_int, radius: Real, length: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetCapsuleTotal(&m, total_mass, direction, radius, length);
    return .{ .raw = m };
}

/// Mass of a solid cylinder. `direction` is the long axis: 1 = X, 2 = Y, 3 = Z.
pub fn cylinder(density: Real, direction: c_int, radius: Real, length: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetCylinder(&m, density, direction, radius, length);
    return .{ .raw = m };
}

/// Mass of a solid cylinder given total mass instead of density.
pub fn cylinder_total(total_mass: Real, direction: c_int, radius: Real, length: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetCylinderTotal(&m, total_mass, direction, radius, length);
    return .{ .raw = m };
}

/// Mass of a solid box given uniform density and side lengths along each axis.
pub fn box(density: Real, lx: Real, ly: Real, lz: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetBox(&m, density, lx, ly, lz);
    return .{ .raw = m };
}

/// Mass of a solid box given total mass instead of density.
pub fn box_total(total_mass: Real, lx: Real, ly: Real, lz: Real) Self {
    var m: c.dMass = undefined;
    c.dMassSetBoxTotal(&m, total_mass, lx, ly, lz);
    return .{ .raw = m };
}

/// Compute mass from a triangle mesh geometry, assuming uniform density.
/// The mesh must have a valid TriMesh.Data attached.
pub fn trimesh(density: Real, geom: Geom.Generic) Self {
    var m: c.dMass = undefined;
    c.dMassSetTrimesh(&m, density, geom.id);
    return .{ .raw = m };
}

/// Compute mass from a triangle mesh given total mass instead of density.
pub fn trimesh_total(total_mass: Real, geom: Geom.Generic) Self {
    var m: c.dMass = undefined;
    c.dMassSetTrimeshTotal(&m, total_mass, geom.id);
    return .{ .raw = m };
}

/// Validate that the mass parameters are physically plausible (positive mass,
/// positive-definite inertia tensor).
pub fn check(self: *const Self) bool {
    return c.dMassCheck(&self.raw) != 0;
}

/// Scale the inertia tensor to match a new total mass value, preserving the shape of
/// the distribution.
pub fn adjust(self: *Self, newmass: Real) void {
    c.dMassAdjust(&self.raw, newmass);
}

/// Shift the center of gravity by the given offset. This also updates the inertia tensor
/// via the parallel axis theorem.
pub fn translate(self: *Self, t: [3]Real) void {
    c.dMassTranslate(&self.raw, t[0], t[1], t[2]);
}

/// Rotate the inertia tensor by a 3x3 rotation matrix.
pub fn rotate(self: *Self, r: *const [12]Real) void {
    c.dMassRotate(&self.raw, r);
}

/// Add another mass distribution to this one. Used to build composite bodies
/// from multiple primitive shapes.
pub fn add(self: *Self, other: *const Self) void {
    c.dMassAdd(&self.raw, &other.raw);
}

/// Read the scalar total mass.
pub fn get_mass(self: *const Self) Real {
    return self.raw.mass;
}

/// Read the center of gravity offset relative to the body's position.
pub fn get_center(self: *const Self) [3]Real {
    return .{ self.raw.c[0], self.raw.c[1], self.raw.c[2] };
}

/// Read the 3x3 inertia tensor, stored as a 3x4 row-major matrix (12 elements, with padding).
pub fn get_inertia(self: *const Self) [12]Real {
    return self.raw.I;
}
