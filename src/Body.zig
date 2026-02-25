//! A rigid body in the simulation world. Has position, orientation, linear/angular velocity,
//! mass properties, and accumulated forces. Attach collision geometry (Geom) and connect
//! to other bodies with joints to build articulated structures.

const c = @import("c.zig").c;
const Real = c.dReal;
const Mass = @import("Mass.zig");
const World = @import("World.zig");
const Joint = @import("Joint.zig");
const Geom = @import("Geom.zig");

id: c.dBodyID,

const Self = @This();

/// Create a new body in the given world, initially at the origin with zero velocity.
pub fn create(world: World) Self {
    return .{ .id = c.dBodyCreate(world.id) };
}

/// Remove this body from the world and free its resources. Any attached joints
/// will be put into limbo (still exist but have no effect until re-attached).
pub fn destroy(self: Self) void {
    c.dBodyDestroy(self.id);
}

/// Get the world this body belongs to.
pub fn get_world(self: Self) World {
    return .{ .id = c.dBodyGetWorld(self.id) };
}

/// Attach an arbitrary user pointer (e.g. your game entity) to this body.
pub fn set_data(self: Self, data: ?*anyopaque) void {
    c.dBodySetData(self.id, data);
}

pub fn get_data(self: Self) ?*anyopaque {
    return c.dBodyGetData(self.id);
}

// --- Position & orientation ---

/// Set the body's world-space position.
pub fn set_position(self: Self, pos: [3]Real) void {
    c.dBodySetPosition(self.id, pos[0], pos[1], pos[2]);
}

/// Get the body's world-space position. Returns a pointer into ODE's internal state;
/// valid until the next step or position change.
pub fn get_position(self: Self) [3]Real {
    const p = c.dBodyGetPosition(self.id);
    return .{ p[0], p[1], p[2] };
}

/// Like `get_position` but copies the data, so it's safe to store.
pub fn copy_position(self: Self) [3]Real {
    var pos: c.dVector3 = undefined;
    c.dBodyCopyPosition(self.id, &pos);
    return .{ pos[0], pos[1], pos[2] };
}

/// Set orientation as a 3x3 rotation matrix (12 elements with padding).
pub fn set_rotation(self: Self, r: *const [12]Real) void {
    c.dBodySetRotation(self.id, r);
}

/// Get the orientation as a pointer to ODE's internal 3x3 rotation matrix.
pub fn get_rotation(self: Self) *const [12]Real {
    return c.dBodyGetRotation(self.id);
}

/// Copy the rotation matrix into a local value.
pub fn copy_rotation(self: Self) [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dBodyCopyRotation(self.id, &r);
    return r;
}

/// Set orientation as a quaternion (w, x, y, z).
pub fn set_quaternion(self: Self, q: *const [4]Real) void {
    c.dBodySetQuaternion(self.id, q);
}

/// Get orientation as a quaternion (w, x, y, z).
pub fn get_quaternion(self: Self) [4]Real {
    const q = c.dBodyGetQuaternion(self.id);
    return .{ q[0], q[1], q[2], q[3] };
}

/// Copy the quaternion into a local value.
pub fn copy_quaternion(self: Self) [4]Real {
    var q: c.dQuaternion = undefined;
    c.dBodyCopyQuaternion(self.id, &q);
    return q;
}

// --- Velocity ---

/// Set the body's linear velocity in world-space (meters/second).
pub fn set_linear_vel(self: Self, v: [3]Real) void {
    c.dBodySetLinearVel(self.id, v[0], v[1], v[2]);
}

pub fn get_linear_vel(self: Self) [3]Real {
    const v = c.dBodyGetLinearVel(self.id);
    return .{ v[0], v[1], v[2] };
}

/// Set the body's angular velocity in world-space (radians/second).
pub fn set_angular_vel(self: Self, v: [3]Real) void {
    c.dBodySetAngularVel(self.id, v[0], v[1], v[2]);
}

pub fn get_angular_vel(self: Self) [3]Real {
    const v = c.dBodyGetAngularVel(self.id);
    return .{ v[0], v[1], v[2] };
}

// --- Mass ---

/// Assign mass properties to this body. The mass must be valid (positive mass,
/// positive-definite inertia tensor) or the simulation may become unstable.
pub fn set_mass(self: Self, mass: *const Mass) void {
    c.dBodySetMass(self.id, &mass.raw);
}

/// Read back the body's current mass properties.
pub fn get_mass(self: Self) Mass {
    var m: c.dMass = undefined;
    c.dBodyGetMass(self.id, &m);
    return .{ .raw = m };
}

// --- Forces & torques ---

/// Add a force (in world coordinates) to the body's center of mass.
/// Forces accumulate and are applied during the next step, then reset to zero.
pub fn add_force(self: Self, f: [3]Real) void {
    c.dBodyAddForce(self.id, f[0], f[1], f[2]);
}

/// Add a torque (in world coordinates).
pub fn add_torque(self: Self, t: [3]Real) void {
    c.dBodyAddTorque(self.id, t[0], t[1], t[2]);
}

/// Add a force expressed in the body's local coordinate frame.
pub fn add_rel_force(self: Self, f: [3]Real) void {
    c.dBodyAddRelForce(self.id, f[0], f[1], f[2]);
}

/// Add a torque expressed in the body's local coordinate frame.
pub fn add_rel_torque(self: Self, t: [3]Real) void {
    c.dBodyAddRelTorque(self.id, t[0], t[1], t[2]);
}

/// Add a force (world coords) at a specific world-space point. This can generate both
/// linear force and torque if the point is not at the center of mass.
pub fn add_force_at_pos(self: Self, f: [3]Real, p: [3]Real) void {
    c.dBodyAddForceAtPos(self.id, f[0], f[1], f[2], p[0], p[1], p[2]);
}

/// Add a force (world coords) at a body-relative point.
pub fn add_force_at_rel_pos(self: Self, f: [3]Real, p: [3]Real) void {
    c.dBodyAddForceAtRelPos(self.id, f[0], f[1], f[2], p[0], p[1], p[2]);
}

/// Add a body-relative force at a world-space point.
pub fn add_rel_force_at_pos(self: Self, f: [3]Real, p: [3]Real) void {
    c.dBodyAddRelForceAtPos(self.id, f[0], f[1], f[2], p[0], p[1], p[2]);
}

/// Add a body-relative force at a body-relative point.
pub fn add_rel_force_at_rel_pos(self: Self, f: [3]Real, p: [3]Real) void {
    c.dBodyAddRelForceAtRelPos(self.id, f[0], f[1], f[2], p[0], p[1], p[2]);
}

/// Read the accumulated force vector (world coords). Reset to zero after each step.
pub fn get_force(self: Self) [3]Real {
    const f = c.dBodyGetForce(self.id);
    return .{ f[0], f[1], f[2] };
}

/// Read the accumulated torque vector (world coords). Reset to zero after each step.
pub fn get_torque(self: Self) [3]Real {
    const t = c.dBodyGetTorque(self.id);
    return .{ t[0], t[1], t[2] };
}

/// Directly set the accumulated force (world coords). Normally you should use `add_force`.
pub fn set_force(self: Self, f: [3]Real) void {
    c.dBodySetForce(self.id, f[0], f[1], f[2]);
}

/// Directly set the accumulated torque (world coords). Normally you should use `add_torque`.
pub fn set_torque(self: Self, t: [3]Real) void {
    c.dBodySetTorque(self.id, t[0], t[1], t[2]);
}

// --- Coordinate conversions ---

/// Transform a body-relative point to world coordinates.
pub fn get_rel_point_pos(self: Self, p: [3]Real) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyGetRelPointPos(self.id, p[0], p[1], p[2], &result);
    return .{ result[0], result[1], result[2] };
}

/// Get the world-space velocity of a body-relative point (accounts for both linear
/// and angular velocity).
pub fn get_rel_point_vel(self: Self, p: [3]Real) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyGetRelPointVel(self.id, p[0], p[1], p[2], &result);
    return .{ result[0], result[1], result[2] };
}

/// Get the world-space velocity of a world-space point on this body.
pub fn get_point_vel(self: Self, p: [3]Real) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyGetPointVel(self.id, p[0], p[1], p[2], &result);
    return .{ result[0], result[1], result[2] };
}

/// Transform a world-space point into body-relative coordinates.
pub fn get_pos_rel_point(self: Self, p: [3]Real) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyGetPosRelPoint(self.id, p[0], p[1], p[2], &result);
    return .{ result[0], result[1], result[2] };
}

/// Rotate a direction vector from body-local to world coordinates (no translation).
pub fn vector_to_world(self: Self, v: [3]Real) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyVectorToWorld(self.id, v[0], v[1], v[2], &result);
    return .{ result[0], result[1], result[2] };
}

/// Rotate a direction vector from world to body-local coordinates (no translation).
pub fn vector_from_world(self: Self, v: [3]Real) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyVectorFromWorld(self.id, v[0], v[1], v[2], &result);
    return .{ result[0], result[1], result[2] };
}

// --- Finite rotation ---

/// Control how rotation is integrated. 0 = infinitesimal (fast, default),
/// 1 = finite (more accurate for fast-spinning bodies, slower).
pub fn set_finite_rotation_mode(self: Self, mode: c_int) void {
    c.dBodySetFiniteRotationMode(self.id, mode);
}

pub fn get_finite_rotation_mode(self: Self) c_int {
    return c.dBodyGetFiniteRotationMode(self.id);
}

/// When using finite rotation mode, set the axis around which the body is primarily
/// spinning. This improves stability for gyroscope-like objects.
pub fn set_finite_rotation_axis(self: Self, axis: [3]Real) void {
    c.dBodySetFiniteRotationAxis(self.id, axis[0], axis[1], axis[2]);
}

pub fn get_finite_rotation_axis(self: Self) [3]Real {
    var result: c.dVector3 = undefined;
    c.dBodyGetFiniteRotationAxis(self.id, &result);
    return .{ result[0], result[1], result[2] };
}

// --- Joints & geoms ---

/// Number of joints attached to this body.
pub fn get_num_joints(self: Self) c_int {
    return c.dBodyGetNumJoints(self.id);
}

/// Get the i-th joint attached to this body (0-based).
pub fn get_joint(self: Self, index: c_int) Joint.Generic {
    return .{ .id = c.dBodyGetJoint(self.id, index) };
}

/// Get the first collision geom attached to this body, or null if none.
/// To iterate all geoms, use `Geom.get_body_next` on the returned geom.
pub fn get_first_geom(self: Self) ?Geom.Generic {
    const g = c.dBodyGetFirstGeom(self.id);
    return if (g != null) .{ .id = g } else null;
}

// --- Enable/disable ---

/// Wake up a disabled body so it participates in simulation again.
pub fn enable(self: Self) void {
    c.dBodyEnable(self.id);
}

/// Disable this body (freeze it). Disabled bodies are not simulated, saving CPU.
/// They are automatically re-enabled if an enabled body touches them.
pub fn disable(self: Self) void {
    c.dBodyDisable(self.id);
}

pub fn is_enabled(self: Self) bool {
    return c.dBodyIsEnabled(self.id) != 0;
}

// --- Dynamic/kinematic ---

/// Set this body to dynamic mode (default). Dynamic bodies are affected by forces
/// and constraints.
pub fn set_dynamic(self: Self) void {
    c.dBodySetDynamic(self.id);
}

/// Set this body to kinematic mode. Kinematic bodies have infinite mass — they are
/// not affected by forces or collisions but can push dynamic bodies around.
/// Useful for moving platforms, animated characters, etc.
pub fn set_kinematic(self: Self) void {
    c.dBodySetKinematic(self.id);
}

pub fn is_kinematic(self: Self) bool {
    return c.dBodyIsKinematic(self.id) != 0;
}

// --- Gravity mode ---

/// Enable or disable gravity for this specific body. Useful for objects that should
/// float (e.g. balloons, spacecraft) while other bodies still fall normally.
pub fn set_gravity_mode(self: Self, mode: bool) void {
    c.dBodySetGravityMode(self.id, @intFromBool(mode));
}

pub fn get_gravity_mode(self: Self) bool {
    return c.dBodyGetGravityMode(self.id) != 0;
}

// --- Moved callback ---

/// Register a callback invoked whenever this body's position/rotation changes during a step.
/// Useful for synchronizing graphics transforms.
pub fn set_moved_callback(self: Self, callback: ?*const fn (c.dBodyID) callconv(.c) void) void {
    c.dBodySetMovedCallback(self.id, callback);
}

// --- Damping ---

/// Reset linear and angular damping to the world's default values.
pub fn set_damping_defaults(self: Self) void {
    c.dBodySetDampingDefaults(self.id);
}

/// Per-body linear damping scale (overrides world default). 0 = no damping, 1 = full stop.
pub fn set_linear_damping(self: Self, scale: Real) void {
    c.dBodySetLinearDamping(self.id, scale);
}

pub fn get_linear_damping(self: Self) Real {
    return c.dBodyGetLinearDamping(self.id);
}

/// Per-body angular damping scale.
pub fn set_angular_damping(self: Self, scale: Real) void {
    c.dBodySetAngularDamping(self.id, scale);
}

pub fn get_angular_damping(self: Self) Real {
    return c.dBodyGetAngularDamping(self.id);
}

/// Set both linear and angular damping at once.
pub fn set_damping(self: Self, linear_scale: Real, angular_scale: Real) void {
    c.dBodySetDamping(self.id, linear_scale, angular_scale);
}

/// Linear velocity below this threshold won't be damped.
pub fn set_linear_damping_threshold(self: Self, threshold: Real) void {
    c.dBodySetLinearDampingThreshold(self.id, threshold);
}

pub fn get_linear_damping_threshold(self: Self) Real {
    return c.dBodyGetLinearDampingThreshold(self.id);
}

/// Angular velocity below this threshold won't be damped.
pub fn set_angular_damping_threshold(self: Self, threshold: Real) void {
    c.dBodySetAngularDampingThreshold(self.id, threshold);
}

pub fn get_angular_damping_threshold(self: Self) Real {
    return c.dBodyGetAngularDampingThreshold(self.id);
}

/// Clamp this body's angular speed to a maximum value.
pub fn set_max_angular_speed(self: Self, max_speed: Real) void {
    c.dBodySetMaxAngularSpeed(self.id, max_speed);
}

pub fn get_max_angular_speed(self: Self) Real {
    return c.dBodyGetMaxAngularSpeed(self.id);
}

/// Enable or disable gyroscopic torque computation. Disabling can improve stability
/// for objects that don't need accurate gyroscopic effects (most game objects).
pub fn set_gyroscopic_mode(self: Self, enabled: bool) void {
    c.dBodySetGyroscopicMode(self.id, @intFromBool(enabled));
}

pub fn get_gyroscopic_mode(self: Self) bool {
    return c.dBodyGetGyroscopicMode(self.id) != 0;
}

// --- Auto-disable (per-body overrides) ---

/// Override the world's auto-disable setting for this body.
pub fn set_auto_disable_flag(self: Self, do_auto_disable: bool) void {
    c.dBodySetAutoDisableFlag(self.id, @intFromBool(do_auto_disable));
}

pub fn get_auto_disable_flag(self: Self) bool {
    return c.dBodyGetAutoDisableFlag(self.id) != 0;
}

/// Per-body linear velocity threshold for auto-disable.
pub fn set_auto_disable_linear_threshold(self: Self, threshold: Real) void {
    c.dBodySetAutoDisableLinearThreshold(self.id, threshold);
}

pub fn get_auto_disable_linear_threshold(self: Self) Real {
    return c.dBodyGetAutoDisableLinearThreshold(self.id);
}

/// Per-body angular velocity threshold for auto-disable.
pub fn set_auto_disable_angular_threshold(self: Self, threshold: Real) void {
    c.dBodySetAutoDisableAngularThreshold(self.id, threshold);
}

pub fn get_auto_disable_angular_threshold(self: Self) Real {
    return c.dBodyGetAutoDisableAngularThreshold(self.id);
}

/// Per-body step count threshold for auto-disable.
pub fn set_auto_disable_steps(self: Self, steps: c_int) void {
    c.dBodySetAutoDisableSteps(self.id, steps);
}

pub fn get_auto_disable_steps(self: Self) c_int {
    return c.dBodyGetAutoDisableSteps(self.id);
}

/// Per-body time threshold for auto-disable.
pub fn set_auto_disable_time(self: Self, time: Real) void {
    c.dBodySetAutoDisableTime(self.id, time);
}

pub fn get_auto_disable_time(self: Self) Real {
    return c.dBodyGetAutoDisableTime(self.id);
}

/// Per-body averaging sample count for auto-disable.
pub fn set_auto_disable_average_samples_count(self: Self, count: c_uint) void {
    c.dBodySetAutoDisableAverageSamplesCount(self.id, count);
}

pub fn get_auto_disable_average_samples_count(self: Self) c_int {
    return c.dBodyGetAutoDisableAverageSamplesCount(self.id);
}

/// Reset all auto-disable parameters on this body to the world defaults.
pub fn set_auto_disable_defaults(self: Self) void {
    c.dBodySetAutoDisableDefaults(self.id);
}
