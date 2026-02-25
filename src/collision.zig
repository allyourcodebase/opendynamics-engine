//! Narrow-phase collision detection: tests pairs of geometries for intersection and
//! produces contact points. Also defines the contact data structures used to create
//! contact joints for the simulation step.

const c = @import("c.zig").c;
const Real = c.dReal;
const Geom = @import("Geom.zig");
const Space = @import("Space.zig");

/// Physical surface properties at a contact point (friction, bounce, softness, slip).
/// Passed inside a `Contact` to `Joint.Contact.create` to control the collision response.
pub const SurfaceParameters = c.dSurfaceParameters;

/// Geometric description of a single contact point: position, normal, penetration depth,
/// and which two geoms produced it.
pub const ContactGeom = c.dContactGeom;

/// Complete contact description combining surface parameters, geometry, and friction direction.
/// Pass to `Joint.Contact.create` to generate a contact constraint for the solver.
pub const Contact = c.dContact;

/// Force/torque feedback from a joint. Assign to a joint with `set_feedback` to record
/// the constraint forces applied each step (useful for breakable joints or diagnostics).
pub const JointFeedback = c.dJointFeedback;

/// Callback signature for `space_collide` and `space_collide2`. Called for each potentially
/// overlapping pair of geoms; you then call `collide` inside to get actual contacts.
pub const NearCallback = *const fn (data: ?*anyopaque, o1: c.dGeomID, o2: c.dGeomID) callconv(.c) void;

/// Test two geoms for intersection and fill the contacts slice with contact points.
/// Returns a sub-slice of the input with only the generated contacts (may be empty).
pub fn collide(o1: Geom.Generic, o2: Geom.Generic, contacts: []ContactGeom) []ContactGeom {
    const n = c.dCollide(
        o1.id,
        o2.id,
        @intCast(contacts.len),
        if (contacts.len > 0) &contacts[0] else null,
        @sizeOf(ContactGeom),
    );
    return if (n > 0) contacts[0..@intCast(n)] else contacts[0..0];
}

/// Broad-phase: iterate over all potentially-overlapping geom pairs within a single space
/// and invoke `callback` for each pair. You typically call `collide` inside the callback.
pub fn space_collide(space: Space.Generic, data: ?*anyopaque, callback: NearCallback) void {
    c.dSpaceCollide(space.id, data, callback);
}

/// Test all geoms in one object against all geoms in another. Either argument can be a
/// space (tests all contained geoms) or a single geom. Useful for testing two separate spaces.
pub fn space_collide2(o1: Geom.Generic, o2: Geom.Generic, data: ?*anyopaque, callback: NearCallback) void {
    c.dSpaceCollide2(o1.id, o2.id, data, callback);
}

// Contact surface mode flags — combine with bitwise OR in SurfaceParameters.mode.
/// Use `mu2` for the second friction direction (otherwise `mu` is used for both).
pub const contact_mu2 = c.dContactMu2;
/// Friction coefficients are axis-dependent (requires `contact_fdir1`).
pub const contact_axis_dep = c.dContactAxisDep;
/// `fdir1` field in Contact specifies the first friction direction (otherwise auto-computed).
pub const contact_fdir1 = c.dContactFDir1;
/// Enable restitution; set `bounce` and `bounce_vel` in SurfaceParameters.
pub const contact_bounce = c.dContactBounce;
/// Use soft constraint ERP for this contact (set `soft_erp` in SurfaceParameters).
pub const contact_soft_erp = c.dContactSoftERP;
/// Use soft constraint CFM for this contact (set `soft_cfm` in SurfaceParameters).
pub const contact_soft_cfm = c.dContactSoftCFM;
/// Surface velocity in friction direction 1 (conveyor belt effect).
pub const contact_motion1 = c.dContactMotion1;
/// Surface velocity in friction direction 2.
pub const contact_motion2 = c.dContactMotion2;
/// Surface velocity in the contact normal direction.
pub const contact_motion_n = c.dContactMotionN;
/// Force-dependent slip in friction direction 1 (like tire slip).
pub const contact_slip1 = c.dContactSlip1;
/// Force-dependent slip in friction direction 2.
pub const contact_slip2 = c.dContactSlip2;
/// Enable rolling friction (set `rho`, `rho2`, `rhoN` in SurfaceParameters).
pub const contact_rolling = c.dContactRolling;
/// Use exact friction model (not a friction pyramid approximation).
pub const contact_approx0 = c.dContactApprox0;
/// Friction pyramid approximation in direction 1.
pub const contact_approx1_1 = c.dContactApprox1_1;
/// Friction pyramid approximation in direction 2.
pub const contact_approx1_2 = c.dContactApprox1_2;
/// Friction pyramid approximation in the normal direction.
pub const contact_approx1_n = c.dContactApprox1_N;
/// Friction pyramid approximation in all directions (combines approx1_1, approx1_2, approx1_n).
pub const contact_approx1 = c.dContactApprox1;
