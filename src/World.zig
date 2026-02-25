//! The simulation world: a container for bodies and joints with global parameters
//! like gravity, ERP/CFM, and stepping methods. Create one world, add bodies to it,
//! connect them with joints, and call `step` or `quick_step` each frame.

const c = @import("c.zig").c;
const Real = c.dReal;
const Body = @import("Body.zig");

id: c.dWorldID,

const Self = @This();

/// Allocate a new empty world with default parameters.
pub fn create() Self {
    return .{ .id = c.dWorldCreate() };
}

/// Destroy the world and all bodies/joints it contains.
pub fn destroy(self: Self) void {
    c.dWorldDestroy(self.id);
}

/// Attach an arbitrary user pointer to this world (e.g. for your game-state back-reference).
pub fn set_data(self: Self, data: ?*anyopaque) void {
    c.dWorldSetData(self.id, data);
}

pub fn get_data(self: Self) ?*anyopaque {
    return c.dWorldGetData(self.id);
}

/// Set the global gravity vector applied to all bodies each step.
pub fn set_gravity(self: Self, g: [3]Real) void {
    c.dWorldSetGravity(self.id, g[0], g[1], g[2]);
}

pub fn get_gravity(self: Self) [3]Real {
    var v: c.dVector3 = undefined;
    c.dWorldGetGravity(self.id, &v);
    return .{ v[0], v[1], v[2] };
}

/// Set the global Error Reduction Parameter (0..1). Controls how aggressively joint
/// errors are corrected each step. Higher values fix errors faster but can cause instability.
pub fn set_erp(self: Self, erp: Real) void {
    c.dWorldSetERP(self.id, erp);
}

pub fn get_erp(self: Self) Real {
    return c.dWorldGetERP(self.id);
}

/// Set the global Constraint Force Mixing value. Adds softness to constraints —
/// higher values make joints springy, lower values make them rigid. Must be >= 0.
pub fn set_cfm(self: Self, cfm: Real) void {
    c.dWorldSetCFM(self.id, cfm);
}

pub fn get_cfm(self: Self) Real {
    return c.dWorldGetCFM(self.id);
}

/// Advance the simulation by `stepsize` seconds using a direct (accurate but slow O(n^3)) solver.
/// Returns false on memory allocation failure.
pub fn step(self: Self, stepsize: Real) bool {
    return c.dWorldStep(self.id, stepsize) != 0;
}

/// Advance the simulation using the iterative QuickStep solver. Much faster than `step`
/// for large worlds, but less accurate. Accuracy improves with more iterations.
pub fn quick_step(self: Self, stepsize: Real) bool {
    return c.dWorldQuickStep(self.id, stepsize) != 0;
}

/// Set the number of SOR (Successive Over-Relaxation) iterations for QuickStep.
/// Default is 20. More iterations = more accurate but slower.
pub fn set_quick_step_num_iterations(self: Self, num: c_int) void {
    c.dWorldSetQuickStepNumIterations(self.id, num);
}

pub fn get_quick_step_num_iterations(self: Self) c_int {
    return c.dWorldGetQuickStepNumIterations(self.id);
}

/// Set the SOR over-relaxation parameter for QuickStep (default 1.3). Values in [1.0, 2.0)
/// can improve convergence; values outside that range may cause divergence.
pub fn set_quick_step_w(self: Self, over_relaxation: Real) void {
    c.dWorldSetQuickStepW(self.id, over_relaxation);
}

pub fn get_quick_step_w(self: Self) Real {
    return c.dWorldGetQuickStepW(self.id);
}

/// Convert an impulse (force * time) to a force suitable for the given step size.
/// Useful for applying impulses through the force-based API.
pub fn impulse_to_force(self: Self, stepsize: Real, impulse: [3]Real) [3]Real {
    var force: c.dVector3 = undefined;
    c.dWorldImpulseToForce(self.id, stepsize, impulse[0], impulse[1], impulse[2], &force);
    return .{ force[0], force[1], force[2] };
}

/// Limit the velocity used to correct interpenetration. Prevents large pop-out forces
/// when objects start deeply overlapping. 0 = no limit (infinity).
pub fn set_contact_max_correcting_vel(self: Self, vel: Real) void {
    c.dWorldSetContactMaxCorrectingVel(self.id, vel);
}

pub fn get_contact_max_correcting_vel(self: Self) Real {
    return c.dWorldGetContactMaxCorrectingVel(self.id);
}

/// Set the depth of the surface layer around all geometry. Contacts within this depth
/// are not corrected, which helps prevent jittering for resting objects.
pub fn set_contact_surface_layer(self: Self, depth: Real) void {
    c.dWorldSetContactSurfaceLayer(self.id, depth);
}

pub fn get_contact_surface_layer(self: Self) Real {
    return c.dWorldGetContactSurfaceLayer(self.id);
}

/// Enable/disable automatic disabling of idle bodies for this world.
/// When enabled, bodies that stop moving are deactivated to save CPU.
pub fn set_auto_disable_flag(self: Self, do_auto_disable: bool) void {
    c.dWorldSetAutoDisableFlag(self.id, @intFromBool(do_auto_disable));
}

pub fn get_auto_disable_flag(self: Self) bool {
    return c.dWorldGetAutoDisableFlag(self.id) != 0;
}

/// Bodies with linear velocity below this threshold (for the required number of steps)
/// are candidates for auto-disable.
pub fn set_auto_disable_linear_threshold(self: Self, threshold: Real) void {
    c.dWorldSetAutoDisableLinearThreshold(self.id, threshold);
}

pub fn get_auto_disable_linear_threshold(self: Self) Real {
    return c.dWorldGetAutoDisableLinearThreshold(self.id);
}

/// Bodies with angular velocity below this threshold are candidates for auto-disable.
pub fn set_auto_disable_angular_threshold(self: Self, threshold: Real) void {
    c.dWorldSetAutoDisableAngularThreshold(self.id, threshold);
}

pub fn get_auto_disable_angular_threshold(self: Self) Real {
    return c.dWorldGetAutoDisableAngularThreshold(self.id);
}

/// Number of consecutive steps a body must be below velocity thresholds before it is disabled.
pub fn set_auto_disable_steps(self: Self, steps: c_int) void {
    c.dWorldSetAutoDisableSteps(self.id, steps);
}

pub fn get_auto_disable_steps(self: Self) c_int {
    return c.dWorldGetAutoDisableSteps(self.id);
}

/// Minimum elapsed simulation time a body must be idle before it is disabled.
pub fn set_auto_disable_time(self: Self, time: Real) void {
    c.dWorldSetAutoDisableTime(self.id, time);
}

pub fn get_auto_disable_time(self: Self) Real {
    return c.dWorldGetAutoDisableTime(self.id);
}

/// Number of recent velocity samples to average when determining if a body should be disabled.
/// Higher values smooth out transients but delay disabling.
pub fn set_auto_disable_average_samples_count(self: Self, count: c_uint) void {
    c.dWorldSetAutoDisableAverageSamplesCount(self.id, count);
}

pub fn get_auto_disable_average_samples_count(self: Self) c_int {
    return c.dWorldGetAutoDisableAverageSamplesCount(self.id);
}

/// Velocity damping factor applied to all bodies' linear velocity each step.
/// 0.0 = no damping, 1.0 = full damping (body stops instantly). Small values like 0.01 are typical.
pub fn set_linear_damping(self: Self, scale: Real) void {
    c.dWorldSetLinearDamping(self.id, scale);
}

pub fn get_linear_damping(self: Self) Real {
    return c.dWorldGetLinearDamping(self.id);
}

/// Velocity damping factor applied to all bodies' angular velocity each step.
pub fn set_angular_damping(self: Self, scale: Real) void {
    c.dWorldSetAngularDamping(self.id, scale);
}

pub fn get_angular_damping(self: Self) Real {
    return c.dWorldGetAngularDamping(self.id);
}

/// Set both linear and angular damping in one call.
pub fn set_damping(self: Self, linear_scale: Real, angular_scale: Real) void {
    c.dWorldSetDamping(self.id, linear_scale, angular_scale);
}

/// Linear velocity magnitude below which damping is not applied.
/// Prevents damping from affecting very slow (nearly resting) motion.
pub fn set_linear_damping_threshold(self: Self, threshold: Real) void {
    c.dWorldSetLinearDampingThreshold(self.id, threshold);
}

pub fn get_linear_damping_threshold(self: Self) Real {
    return c.dWorldGetLinearDampingThreshold(self.id);
}

/// Angular velocity magnitude below which damping is not applied.
pub fn set_angular_damping_threshold(self: Self, threshold: Real) void {
    c.dWorldSetAngularDampingThreshold(self.id, threshold);
}

pub fn get_angular_damping_threshold(self: Self) Real {
    return c.dWorldGetAngularDampingThreshold(self.id);
}

/// Clamp all bodies' angular speed to this maximum. Prevents numerical instability
/// from extremely fast rotations. 0 = no limit.
pub fn set_max_angular_speed(self: Self, max_speed: Real) void {
    c.dWorldSetMaxAngularSpeed(self.id, max_speed);
}

pub fn get_max_angular_speed(self: Self) Real {
    return c.dWorldGetMaxAngularSpeed(self.id);
}

/// Limit how many threads can process constraint islands in parallel during a step.
pub fn set_step_islands_processing_max_thread_count(self: Self, count: c_uint) void {
    c.dWorldSetStepIslandsProcessingMaxThreadCount(self.id, count);
}

pub fn get_step_islands_processing_max_thread_count(self: Self) c_uint {
    return c.dWorldGetStepIslandsProcessingMaxThreadCount(self.id);
}

/// Share internal working memory with another world to reduce allocations when simulating
/// multiple worlds sequentially. Pass null to stop sharing.
pub fn use_shared_working_memory(self: Self, from_world: ?Self) bool {
    return c.dWorldUseSharedWorkingMemory(self.id, if (from_world) |w| w.id else null) != 0;
}

/// Free internal working memory. It will be reallocated automatically on the next step.
pub fn cleanup_working_memory(self: Self) void {
    c.dWorldCleanupWorkingMemory(self.id);
}

/// Assign a custom threading implementation for parallel constraint solving.
pub fn set_step_threading_implementation(self: Self, functions_info: ?*const c.dThreadingFunctionsInfo, threading_impl: c.dThreadingImplementationID) void {
    c.dWorldSetStepThreadingImplementation(self.id, functions_info, threading_impl);
}

/// Create a new rigid body in this world, initially at the origin with zero velocity.
pub fn create_body(self: Self) Body {
    return .{ .id = c.dBodyCreate(self.id) };
}
