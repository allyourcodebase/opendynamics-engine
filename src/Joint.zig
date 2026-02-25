//! ODE joint types and their Zig bindings.
//!
//! Joints constrain the relative motion of two rigid bodies (or one body
//! relative to the static environment). Each concrete joint type removes
//! specific degrees of freedom -- for example a `Hinge` allows only
//! single-axis rotation, while a `Ball` permits free rotation about a point.
//!
//! Temporary per-step joints (typically `Contact`) should be allocated in a
//! `Group` so they can be destroyed efficiently with a single `empty` call.

const c = @import("c.zig").c;
const Real = c.dReal;
const Body = @import("Body.zig");
const World = @import("World.zig");
const collision = @import("collision.zig");

/// Per-joint motor, limit, and softness parameters.
///
/// Unsuffixed values apply to the joint's first axis. The `2` and `3`
/// suffixes address the second and third axes of multi-axis joints
/// (e.g. Universal, AMotor).
///
/// Key groups:
///  - `lo_stop` / `hi_stop` -- angular or linear travel limits.
///  - `vel` / `f_max` -- motor target velocity and maximum force.
///  - `bounce` -- restitution coefficient when a limit is hit.
///  - `cfm` / `erp` -- per-joint constraint-force mixing and error-reduction,
///    controlling softness and error correction strength.
///  - `stop_erp` / `stop_cfm` -- softness at the limit stops specifically.
///  - `suspension_erp` / `suspension_cfm` -- used by Hinge2 for vehicle
///    suspension spring/damper behavior.
pub const Param = enum(c_int) {
    lo_stop = c.dParamLoStop,
    hi_stop = c.dParamHiStop,
    vel = c.dParamVel,
    lo_vel = c.dParamLoVel,
    hi_vel = c.dParamHiVel,
    f_max = c.dParamFMax,
    fudge_factor = c.dParamFudgeFactor,
    bounce = c.dParamBounce,
    cfm = c.dParamCFM,
    stop_erp = c.dParamStopERP,
    stop_cfm = c.dParamStopCFM,
    suspension_erp = c.dParamSuspensionERP,
    suspension_cfm = c.dParamSuspensionCFM,
    erp = c.dParamERP,
    lo_stop2 = c.dParamLoStop2,
    hi_stop2 = c.dParamHiStop2,
    vel2 = c.dParamVel2,
    lo_vel2 = c.dParamLoVel2,
    hi_vel2 = c.dParamHiVel2,
    f_max2 = c.dParamFMax2,
    fudge_factor2 = c.dParamFudgeFactor2,
    bounce2 = c.dParamBounce2,
    cfm2 = c.dParamCFM2,
    stop_erp2 = c.dParamStopERP2,
    stop_cfm2 = c.dParamStopCFM2,
    suspension_erp2 = c.dParamSuspensionERP2,
    suspension_cfm2 = c.dParamSuspensionCFM2,
    erp2 = c.dParamERP2,
    lo_stop3 = c.dParamLoStop3,
    hi_stop3 = c.dParamHiStop3,
    vel3 = c.dParamVel3,
    lo_vel3 = c.dParamLoVel3,
    hi_vel3 = c.dParamHiVel3,
    f_max3 = c.dParamFMax3,
    fudge_factor3 = c.dParamFudgeFactor3,
    bounce3 = c.dParamBounce3,
    cfm3 = c.dParamCFM3,
    stop_erp3 = c.dParamStopERP3,
    stop_cfm3 = c.dParamStopCFM3,
    suspension_erp3 = c.dParamSuspensionERP3,
    suspension_cfm3 = c.dParamSuspensionCFM3,
    erp3 = c.dParamERP3,
};

/// Discriminant for the concrete ODE joint type behind a joint handle.
pub const JointType = enum(c_int) {
    /// No joint / invalid.
    none = c.dJointTypeNone,
    /// Ball-and-socket: 3 rotational degrees of freedom.
    ball = c.dJointTypeBall,
    /// Hinge: single-axis rotation.
    hinge = c.dJointTypeHinge,
    /// Slider: single-axis translation (prismatic).
    slider = c.dJointTypeSlider,
    /// Contact: temporary collision-response joint, usually in a Group.
    contact = c.dJointTypeContact,
    /// Universal (Cardan): 2 perpendicular rotation axes.
    universal = c.dJointTypeUniversal,
    /// Hinge2: suspension joint with 2 hinge axes (e.g. car wheel).
    hinge2 = c.dJointTypeHinge2,
    /// Fixed: rigidly locks two bodies together (mainly for debugging).
    fixed = c.dJointTypeFixed,
    /// Null: placeholder joint with no effect on the simulation.
    @"null" = c.dJointTypeNull,
    /// Angular motor: drives or limits rotation on up to 3 axes.
    a_motor = c.dJointTypeAMotor,
    /// Linear motor: drives or limits translation on up to 3 axes.
    l_motor = c.dJointTypeLMotor,
    /// Plane2D: constrains a body to the XY plane.
    plane2d = c.dJointTypePlane2D,
    /// Prismatic-rotoide: combined slider + hinge.
    pr = c.dJointTypePR,
    /// Prismatic-universal: combined slider + universal.
    pu = c.dJointTypePU,
    /// Piston: slider that also allows rotation around the slide axis.
    piston = c.dJointTypePiston,
    /// Distance-preserving ball: spring-like, maintains distance between anchors.
    d_ball = c.dJointTypeDBall,
    /// Distance-preserving hinge: spring-like hinge that preserves anchor distance.
    d_hinge = c.dJointTypeDHinge,
    /// Transmission: gear, belt, or chain coupling two rotating bodies.
    transmission = c.dJointTypeTransmission,
};

/// Returns a struct of common joint methods parameterized for the given type.
/// Every joint struct (Generic, Ball, Hinge, etc.) re-exports these as its own methods.
fn JointMethods(comptime Self: type) type {
    return struct {
        /// Remove this joint from the simulation and free its resources.
        pub fn destroy(self: Self) void { c.dJointDestroy(self.id); }
        /// Connect this joint to two bodies. Pass `null` for either body to
        /// attach that side to the static environment.
        pub fn attach(self: Self, body1: ?Body, body2: ?Body) void { c.dJointAttach(self.id, if (body1) |b| b.id else null, if (body2) |b| b.id else null); }
        /// Allow this joint's constraint to take effect in the simulation.
        pub fn enable(self: Self) void { c.dJointEnable(self.id); }
        /// Temporarily suspend this joint's constraint without destroying it.
        pub fn disable(self: Self) void { c.dJointDisable(self.id); }
        /// Returns true if this joint is currently active in the simulation.
        pub fn is_enabled(self: Self) bool { return c.dJointIsEnabled(self.id) != 0; }
        /// Returns the number of bodies attached to this joint (0, 1, or 2).
        pub fn get_num_bodies(self: Self) c_int { return c.dJointGetNumBodies(self.id); }
        /// Returns the body attached at the given index (0 or 1), or null if
        /// that side is attached to the static environment.
        pub fn get_body(self: Self, index: c_int) ?Body { const b = c.dJointGetBody(self.id, index); return if (b != null) .{ .id = b } else null; }
        /// Store an arbitrary user pointer on this joint.
        pub fn set_data(self: Self, data: ?*anyopaque) void { c.dJointSetData(self.id, data); }
        /// Retrieve the user pointer previously stored with `set_data`.
        pub fn get_data(self: Self) ?*anyopaque { return c.dJointGetData(self.id); }
        /// Returns the concrete `JointType` discriminant for this joint.
        pub fn get_type(self: Self) JointType { return @enumFromInt(c.dJointGetType(self.id)); }
        /// Attach a feedback struct that ODE will fill each step with the
        /// constraint forces and torques applied by this joint.
        pub fn set_feedback(self: Self, feedback: ?*c.dJointFeedback) void { c.dJointSetFeedback(self.id, feedback); }
        /// Returns the feedback struct, or null if none was set.
        pub fn get_feedback(self: Self) ?*c.dJointFeedback { return c.dJointGetFeedback(self.id); }
        /// Erase the concrete type and return a type-erased `Generic` handle.
        pub fn to_generic(self: Self) Generic { return .{ .id = self.id }; }
    };
}

/// Type-erased joint handle. Provides only the operations common to all
/// joint types. Useful when storing heterogeneous joints in a single collection.
pub const Generic = struct {
    id: c.dJointID,

    const methods = JointMethods(Generic);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
};

/// Container for temporary joints (typically contact joints created during
/// collision handling). Call `empty` once per simulation step to destroy all
/// contained joints efficiently in bulk.
pub const Group = struct {
    id: c.dJointGroupID,

    /// Allocate a new, empty joint group.
    pub fn create() Group {
        return .{ .id = c.dJointGroupCreate(0) };
    }

    /// Destroy the group and all joints it contains.
    pub fn destroy(self: Group) void {
        c.dJointGroupDestroy(self.id);
    }

    /// Destroy all joints in the group without destroying the group itself.
    /// Call this each step before creating new contact joints.
    pub fn empty(self: Group) void {
        c.dJointGroupEmpty(self.id);
    }
};

/// Ball-and-socket joint -- allows 3 rotational degrees of freedom around a
/// single anchor point, like a shoulder joint.
pub const Ball = struct {
    id: c.dJointID,

    const methods = JointMethods(Ball);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a ball joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Ball {
        return .{ .id = c.dJointCreateBall(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point in world coordinates where the two bodies connect.
    pub fn set_anchor(self: Ball, anchor: [3]Real) void {
        c.dJointSetBallAnchor(self.id, anchor[0], anchor[1], anchor[2]);
    }

    /// Read back the anchor point on body 1 in world coordinates.
    pub fn get_anchor(self: Ball) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetBallAnchor(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Read back the anchor point on body 2 in world coordinates. Drift
    /// between `get_anchor` and `get_anchor2` indicates joint error.
    pub fn get_anchor2(self: Ball) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetBallAnchor2(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Set a joint parameter (motor/limit value) for this ball joint.
    pub fn set_param(self: Ball, parameter: Param, value: Real) void {
        c.dJointSetBallParam(self.id, @intFromEnum(parameter), value);
    }

    /// Get a joint parameter (motor/limit value) for this ball joint.
    pub fn get_param(self: Ball, parameter: Param) Real {
        return c.dJointGetBallParam(self.id, @intFromEnum(parameter));
    }
};

/// Hinge joint -- single-axis rotation, like a door hinge. The joint
/// constrains the two bodies to share an anchor point and rotate only
/// around the specified axis.
pub const Hinge = struct {
    id: c.dJointID,

    const methods = JointMethods(Hinge);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a hinge joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Hinge {
        return .{ .id = c.dJointCreateHinge(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point in world coordinates.
    pub fn set_anchor(self: Hinge, anchor: [3]Real) void {
        c.dJointSetHingeAnchor(self.id, anchor[0], anchor[1], anchor[2]);
    }

    /// Read back the anchor point on body 1 in world coordinates.
    pub fn get_anchor(self: Hinge) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetHingeAnchor(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Read back the anchor point on body 2. Drift from `get_anchor`
    /// indicates accumulated joint error.
    pub fn get_anchor2(self: Hinge) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetHingeAnchor2(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Set the hinge rotation axis in world coordinates.
    pub fn set_axis(self: Hinge, axis: [3]Real) void {
        c.dJointSetHingeAxis(self.id, axis[0], axis[1], axis[2]);
    }

    /// Get the hinge rotation axis in world coordinates.
    pub fn get_axis(self: Hinge) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetHingeAxis(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Current rotation angle (radians) relative to the initial configuration.
    pub fn get_angle(self: Hinge) Real {
        return c.dJointGetHingeAngle(self.id);
    }

    /// Time derivative of the hinge angle (radians/second).
    pub fn get_angle_rate(self: Hinge) Real {
        return c.dJointGetHingeAngleRate(self.id);
    }

    /// Set a joint parameter (motor/limit value) for this hinge.
    pub fn set_param(self: Hinge, parameter: Param, value: Real) void {
        c.dJointSetHingeParam(self.id, @intFromEnum(parameter), value);
    }

    /// Get a joint parameter (motor/limit value) for this hinge.
    pub fn get_param(self: Hinge, parameter: Param) Real {
        return c.dJointGetHingeParam(self.id, @intFromEnum(parameter));
    }

    /// Apply a torque about the hinge axis to both attached bodies.
    pub fn add_torque(self: Hinge, torque: Real) void {
        c.dJointAddHingeTorque(self.id, torque);
    }
};

/// Slider (prismatic) joint -- allows translation along a single axis with
/// no rotation. Think of a piston rod constrained to slide without twisting.
pub const Slider = struct {
    id: c.dJointID,

    const methods = JointMethods(Slider);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a slider joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Slider {
        return .{ .id = c.dJointCreateSlider(world.id, if (group) |g| g.id else null) };
    }

    /// Set the sliding axis direction in world coordinates.
    pub fn set_axis(self: Slider, axis: [3]Real) void {
        c.dJointSetSliderAxis(self.id, axis[0], axis[1], axis[2]);
    }

    /// Get the sliding axis direction in world coordinates.
    pub fn get_axis(self: Slider) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetSliderAxis(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Linear displacement along the slider axis relative to the initial position.
    pub fn get_position(self: Slider) Real {
        return c.dJointGetSliderPosition(self.id);
    }

    /// Time derivative of the slider position (linear velocity along the axis).
    pub fn get_position_rate(self: Slider) Real {
        return c.dJointGetSliderPositionRate(self.id);
    }

    /// Set a joint parameter (motor/limit value) for this slider.
    pub fn set_param(self: Slider, parameter: Param, value: Real) void {
        c.dJointSetSliderParam(self.id, @intFromEnum(parameter), value);
    }

    /// Get a joint parameter (motor/limit value) for this slider.
    pub fn get_param(self: Slider, parameter: Param) Real {
        return c.dJointGetSliderParam(self.id, @intFromEnum(parameter));
    }

    /// Apply a force along the slider axis to both attached bodies.
    pub fn add_force(self: Slider, force: Real) void {
        c.dJointAddSliderForce(self.id, force);
    }
};

/// Contact joint -- a temporary constraint created from a collision contact
/// point. These are typically created each simulation step inside a `Group`
/// and destroyed in bulk via `Group.empty`.
pub const Contact = struct {
    id: c.dJointID,

    const methods = JointMethods(Contact);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a contact joint from a collision `Contact` struct. The joint
    /// should be added to a group so it can be bulk-destroyed each step.
    pub fn create(world: World, group: ?Group, contact: *const collision.Contact) Contact {
        return .{ .id = c.dJointCreateContact(world.id, if (group) |g| g.id else null, contact) };
    }
};

/// Universal (Cardan) joint -- two perpendicular rotation axes, like a
/// universal joint in a drive shaft. Axis 1 is attached to body 1, axis 2
/// to body 2, and they are kept perpendicular by the constraint.
pub const Universal = struct {
    id: c.dJointID,

    const methods = JointMethods(Universal);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a universal joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Universal {
        return .{ .id = c.dJointCreateUniversal(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point in world coordinates.
    pub fn set_anchor(self: Universal, anchor: [3]Real) void {
        c.dJointSetUniversalAnchor(self.id, anchor[0], anchor[1], anchor[2]);
    }

    /// Read back the anchor point on body 1 in world coordinates.
    pub fn get_anchor(self: Universal) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetUniversalAnchor(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Read back the anchor point on body 2. Drift from `get_anchor`
    /// indicates accumulated joint error.
    pub fn get_anchor2(self: Universal) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetUniversalAnchor2(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Set rotation axis 1 (attached to body 1) in world coordinates.
    pub fn set_axis1(self: Universal, axis: [3]Real) void {
        c.dJointSetUniversalAxis1(self.id, axis[0], axis[1], axis[2]);
    }

    /// Get rotation axis 1 in world coordinates.
    pub fn get_axis1(self: Universal) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetUniversalAxis1(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Set rotation axis 2 (attached to body 2) in world coordinates.
    pub fn set_axis2(self: Universal, axis: [3]Real) void {
        c.dJointSetUniversalAxis2(self.id, axis[0], axis[1], axis[2]);
    }

    /// Get rotation axis 2 in world coordinates.
    pub fn get_axis2(self: Universal) [3]Real {
        var result: c.dVector3 = undefined;
        c.dJointGetUniversalAxis2(self.id, &result);
        return .{ result[0], result[1], result[2] };
    }

    /// Current rotation angle (radians) around axis 1.
    pub fn get_angle1(self: Universal) Real { return c.dJointGetUniversalAngle1(self.id); }
    /// Current rotation angle (radians) around axis 2.
    pub fn get_angle2(self: Universal) Real { return c.dJointGetUniversalAngle2(self.id); }
    /// Angular velocity (radians/second) around axis 1.
    pub fn get_angle1_rate(self: Universal) Real { return c.dJointGetUniversalAngle1Rate(self.id); }
    /// Angular velocity (radians/second) around axis 2.
    pub fn get_angle2_rate(self: Universal) Real { return c.dJointGetUniversalAngle2Rate(self.id); }

    /// Set a joint parameter. Use suffixed params (e.g. `vel2`) for axis 2.
    pub fn set_param(self: Universal, parameter: Param, value: Real) void {
        c.dJointSetUniversalParam(self.id, @intFromEnum(parameter), value);
    }

    /// Get a joint parameter. Use suffixed params (e.g. `vel2`) for axis 2.
    pub fn get_param(self: Universal, parameter: Param) Real {
        return c.dJointGetUniversalParam(self.id, @intFromEnum(parameter));
    }

    /// Apply torques about axis 1 and axis 2 to both attached bodies.
    pub fn add_torques(self: Universal, torque1: Real, torque2: Real) void {
        c.dJointAddUniversalTorques(self.id, torque1, torque2);
    }
};

/// Hinge2 (suspension) joint -- two hinge axes where axis 1 is the steering
/// axis and axis 2 is the wheel spin axis. Commonly used for vehicle wheel
/// suspension; the `suspension_erp`/`suspension_cfm` parameters control the
/// spring/damper behavior.
pub const Hinge2 = struct {
    id: c.dJointID,

    const methods = JointMethods(Hinge2);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a hinge2 joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Hinge2 {
        return .{ .id = c.dJointCreateHinge2(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point (typically the wheel hub) in world coordinates.
    pub fn set_anchor(self: Hinge2, anchor: [3]Real) void { c.dJointSetHinge2Anchor(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Read back the anchor on body 1 in world coordinates.
    pub fn get_anchor(self: Hinge2) [3]Real { var r: c.dVector3 = undefined; c.dJointGetHinge2Anchor(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Read back the anchor on body 2. Drift from `get_anchor` shows joint error.
    pub fn get_anchor2(self: Hinge2) [3]Real { var r: c.dVector3 = undefined; c.dJointGetHinge2Anchor2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set axis 1 (steering axis, attached to body 1) in world coordinates.
    pub fn set_axis1(self: Hinge2, axis: [3]Real) void { c.dJointSetHinge2Axis1(self.id, axis[0], axis[1], axis[2]); }
    /// Get axis 1 (steering axis) in world coordinates.
    pub fn get_axis1(self: Hinge2) [3]Real { var r: c.dVector3 = undefined; c.dJointGetHinge2Axis1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set axis 2 (wheel spin axis, attached to body 2) in world coordinates.
    pub fn set_axis2(self: Hinge2, axis: [3]Real) void { c.dJointSetHinge2Axis2(self.id, axis[0], axis[1], axis[2]); }
    /// Get axis 2 (wheel spin axis) in world coordinates.
    pub fn get_axis2(self: Hinge2) [3]Real { var r: c.dVector3 = undefined; c.dJointGetHinge2Axis2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Current rotation angle (radians) of axis 1 (steering angle).
    pub fn get_angle1(self: Hinge2) Real { return c.dJointGetHinge2Angle1(self.id); }
    /// Angular velocity (radians/second) around axis 1.
    pub fn get_angle1_rate(self: Hinge2) Real { return c.dJointGetHinge2Angle1Rate(self.id); }
    /// Angular velocity (radians/second) around axis 2 (wheel spin rate).
    pub fn get_angle2_rate(self: Hinge2) Real { return c.dJointGetHinge2Angle2Rate(self.id); }
    /// Set a joint parameter. Use suffixed params (e.g. `vel2`) for axis 2.
    pub fn set_param(self: Hinge2, parameter: Param, value: Real) void { c.dJointSetHinge2Param(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter. Use suffixed params (e.g. `vel2`) for axis 2.
    pub fn get_param(self: Hinge2, parameter: Param) Real { return c.dJointGetHinge2Param(self.id, @intFromEnum(parameter)); }
    /// Apply torques about axis 1 and axis 2 to both attached bodies.
    pub fn add_torques(self: Hinge2, torque1: Real, torque2: Real) void { c.dJointAddHinge2Torques(self.id, torque1, torque2); }
};

/// Fixed joint -- rigidly locks two bodies (or one body and the world) in
/// their current relative position and orientation. Mainly useful for
/// debugging; in production prefer compound collision geometries instead.
pub const Fixed = struct {
    id: c.dJointID,

    const methods = JointMethods(Fixed);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a fixed joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Fixed {
        return .{ .id = c.dJointCreateFixed(world.id, if (group) |g| g.id else null) };
    }

    /// Lock the attached bodies at their current relative pose. Must be called
    /// after `attach` and after positioning the bodies.
    pub fn set(self: Fixed) void { c.dJointSetFixed(self.id); }
    /// Set a joint parameter for this fixed joint.
    pub fn set_param(self: Fixed, parameter: Param, value: Real) void { c.dJointSetFixedParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter for this fixed joint.
    pub fn get_param(self: Fixed, parameter: Param) Real { return c.dJointGetFixedParam(self.id, @intFromEnum(parameter)); }
};

/// Null joint -- a placeholder that has no effect on the simulation.
/// Can be used as a sentinel or for bookkeeping purposes.
pub const Null = struct {
    id: c.dJointID,

    const methods = JointMethods(Null);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a null joint in the given world, optionally adding it to a group.
    pub fn create(world: World, group: ?Group) Null {
        return .{ .id = c.dJointCreateNull(world.id, if (group) |g| g.id else null) };
    }
};

/// Angular motor -- drives or limits rotation on up to 3 axes independently.
/// Supports two modes: user-specified axes or Euler angles. Use `set_param`
/// with suffixed parameters (`vel2`, `f_max3`, etc.) to control each axis.
pub const AMotor = struct {
    id: c.dJointID,

    const methods = JointMethods(AMotor);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create an angular motor joint in the given world.
    pub fn create(world: World, group: ?Group) AMotor {
        return .{ .id = c.dJointCreateAMotor(world.id, if (group) |g| g.id else null) };
    }

    /// Set the motor mode: 0 = user-specified axes, 1 = Euler angles.
    pub fn set_mode(self: AMotor, mode: c_int) void { c.dJointSetAMotorMode(self.id, mode); }
    /// Get the current motor mode.
    pub fn get_mode(self: AMotor) c_int { return c.dJointGetAMotorMode(self.id); }
    /// Set how many axes (0-3) this motor controls.
    pub fn set_num_axes(self: AMotor, num: c_int) void { c.dJointSetAMotorNumAxes(self.id, num); }
    /// Get the number of active axes.
    pub fn get_num_axes(self: AMotor) c_int { return c.dJointGetAMotorNumAxes(self.id); }
    /// Set the direction and reference frame for axis `anum` (0-2).
    /// `rel`: 0 = global, 1 = relative to body 1, 2 = relative to body 2.
    pub fn set_axis(self: AMotor, anum: c_int, rel: c_int, axis: [3]Real) void { c.dJointSetAMotorAxis(self.id, anum, rel, axis[0], axis[1], axis[2]); }
    /// Get the direction of axis `anum` in world coordinates.
    pub fn get_axis(self: AMotor, anum: c_int) [3]Real { var r: c.dVector3 = undefined; c.dJointGetAMotorAxis(self.id, anum, &r); return .{ r[0], r[1], r[2] }; }
    /// Get the reference frame (0=global, 1=body1, 2=body2) of axis `anum`.
    pub fn get_axis_rel(self: AMotor, anum: c_int) c_int { return c.dJointGetAMotorAxisRel(self.id, anum); }
    /// Set the current angle (radians) for axis `anum`. Only needed in user mode;
    /// Euler mode computes angles automatically.
    pub fn set_angle(self: AMotor, anum: c_int, angle: Real) void { c.dJointSetAMotorAngle(self.id, anum, angle); }
    /// Get the current angle (radians) of axis `anum`.
    pub fn get_angle(self: AMotor, anum: c_int) Real { return c.dJointGetAMotorAngle(self.id, anum); }
    /// Get the angular velocity (radians/second) of axis `anum`.
    pub fn get_angle_rate(self: AMotor, anum: c_int) Real { return c.dJointGetAMotorAngleRate(self.id, anum); }
    /// Set a joint parameter. Use suffixed params for axes 2 and 3.
    pub fn set_param(self: AMotor, parameter: Param, value: Real) void { c.dJointSetAMotorParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter. Use suffixed params for axes 2 and 3.
    pub fn get_param(self: AMotor, parameter: Param) Real { return c.dJointGetAMotorParam(self.id, @intFromEnum(parameter)); }
    /// Apply torques about all three motor axes simultaneously.
    pub fn add_torques(self: AMotor, torque0: Real, torque1: Real, torque2: Real) void { c.dJointAddAMotorTorques(self.id, torque0, torque1, torque2); }
};

/// Linear motor -- drives or limits translation on up to 3 axes independently.
/// Use `set_param` with suffixed parameters (`vel2`, `f_max3`, etc.) to
/// control each axis.
pub const LMotor = struct {
    id: c.dJointID,

    const methods = JointMethods(LMotor);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a linear motor joint in the given world.
    pub fn create(world: World, group: ?Group) LMotor {
        return .{ .id = c.dJointCreateLMotor(world.id, if (group) |g| g.id else null) };
    }

    /// Set how many axes (0-3) this motor controls.
    pub fn set_num_axes(self: LMotor, num: c_int) void { c.dJointSetLMotorNumAxes(self.id, num); }
    /// Get the number of active axes.
    pub fn get_num_axes(self: LMotor) c_int { return c.dJointGetLMotorNumAxes(self.id); }
    /// Set the direction and reference frame for axis `anum` (0-2).
    /// `rel`: 0 = global, 1 = relative to body 1, 2 = relative to body 2.
    pub fn set_axis(self: LMotor, anum: c_int, rel: c_int, axis: [3]Real) void { c.dJointSetLMotorAxis(self.id, anum, rel, axis[0], axis[1], axis[2]); }
    /// Get the direction of axis `anum` in world coordinates.
    pub fn get_axis(self: LMotor, anum: c_int) [3]Real { var r: c.dVector3 = undefined; c.dJointGetLMotorAxis(self.id, anum, &r); return .{ r[0], r[1], r[2] }; }
    /// Set a joint parameter. Use suffixed params for axes 2 and 3.
    pub fn set_param(self: LMotor, parameter: Param, value: Real) void { c.dJointSetLMotorParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter. Use suffixed params for axes 2 and 3.
    pub fn get_param(self: LMotor, parameter: Param) Real { return c.dJointGetLMotorParam(self.id, @intFromEnum(parameter)); }
};

/// Plane2D joint -- constrains a body to move only in the XY plane (z=0)
/// with rotation only around the Z axis. Useful for 2D physics simulations.
pub const Plane2D = struct {
    id: c.dJointID,

    const methods = JointMethods(Plane2D);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a Plane2D joint in the given world.
    pub fn create(world: World, group: ?Group) Plane2D {
        return .{ .id = c.dJointCreatePlane2D(world.id, if (group) |g| g.id else null) };
    }

    /// Set a motor/limit parameter for the X translational axis.
    pub fn set_x_param(self: Plane2D, parameter: Param, value: Real) void { c.dJointSetPlane2DXParam(self.id, @intFromEnum(parameter), value); }
    /// Set a motor/limit parameter for the Y translational axis.
    pub fn set_y_param(self: Plane2D, parameter: Param, value: Real) void { c.dJointSetPlane2DYParam(self.id, @intFromEnum(parameter), value); }
    /// Set a motor/limit parameter for the Z rotational axis.
    pub fn set_angle_param(self: Plane2D, parameter: Param, value: Real) void { c.dJointSetPlane2DAngleParam(self.id, @intFromEnum(parameter), value); }
};

/// Prismatic-rotoide (PR) joint -- combines a slider (prismatic, axis 1)
/// with a hinge (rotoide, axis 2). The body can translate along axis 1 and
/// rotate around axis 2.
pub const PR = struct {
    id: c.dJointID,

    const methods = JointMethods(PR);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a PR joint in the given world.
    pub fn create(world: World, group: ?Group) PR {
        return .{ .id = c.dJointCreatePR(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point in world coordinates.
    pub fn set_anchor(self: PR, anchor: [3]Real) void { c.dJointSetPRAnchor(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Get the anchor point in world coordinates.
    pub fn get_anchor(self: PR) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPRAnchor(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the prismatic (sliding) axis direction.
    pub fn set_axis1(self: PR, axis: [3]Real) void { c.dJointSetPRAxis1(self.id, axis[0], axis[1], axis[2]); }
    /// Get the prismatic (sliding) axis direction.
    pub fn get_axis1(self: PR) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPRAxis1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the rotoide (hinge) axis direction.
    pub fn set_axis2(self: PR, axis: [3]Real) void { c.dJointSetPRAxis2(self.id, axis[0], axis[1], axis[2]); }
    /// Get the rotoide (hinge) axis direction.
    pub fn get_axis2(self: PR) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPRAxis2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Linear displacement along the prismatic axis.
    pub fn get_position(self: PR) Real { return c.dJointGetPRPosition(self.id); }
    /// Rotation angle (radians) around the rotoide axis.
    pub fn get_angle(self: PR) Real { return c.dJointGetPRAngle(self.id); }
    /// Set a joint parameter. Use suffixed params (e.g. `vel2`) for axis 2.
    pub fn set_param(self: PR, parameter: Param, value: Real) void { c.dJointSetPRParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter. Use suffixed params (e.g. `vel2`) for axis 2.
    pub fn get_param(self: PR, parameter: Param) Real { return c.dJointGetPRParam(self.id, @intFromEnum(parameter)); }
    /// Apply a torque about the rotoide axis.
    pub fn add_torque(self: PR, torque: Real) void { c.dJointAddPRTorque(self.id, torque); }
};

/// Prismatic-universal (PU) joint -- combines a slider (prismatic) with a
/// universal joint. Allows translation along one axis plus rotation around
/// two perpendicular axes.
pub const PU = struct {
    id: c.dJointID,

    const methods = JointMethods(PU);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a PU joint in the given world.
    pub fn create(world: World, group: ?Group) PU {
        return .{ .id = c.dJointCreatePU(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point in world coordinates.
    pub fn set_anchor(self: PU, anchor: [3]Real) void { c.dJointSetPUAnchor(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Get the anchor point in world coordinates.
    pub fn get_anchor(self: PU) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPUAnchor(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set universal rotation axis 1.
    pub fn set_axis1(self: PU, axis: [3]Real) void { c.dJointSetPUAxis1(self.id, axis[0], axis[1], axis[2]); }
    /// Get universal rotation axis 1.
    pub fn get_axis1(self: PU) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPUAxis1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set universal rotation axis 2.
    pub fn set_axis2(self: PU, axis: [3]Real) void { c.dJointSetPUAxis2(self.id, axis[0], axis[1], axis[2]); }
    /// Get universal rotation axis 2.
    pub fn get_axis2(self: PU) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPUAxis2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set axis 3 (alias for the prismatic axis).
    pub fn set_axis3(self: PU, axis: [3]Real) void { c.dJointSetPUAxis3(self.id, axis[0], axis[1], axis[2]); }
    /// Set the prismatic (sliding) axis direction.
    pub fn set_axis_p(self: PU, axis: [3]Real) void { c.dJointSetPUAxisP(self.id, axis[0], axis[1], axis[2]); }
    /// Get the prismatic (sliding) axis direction.
    pub fn get_axis_p(self: PU) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPUAxisP(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Linear displacement along the prismatic axis.
    pub fn get_position(self: PU) Real { return c.dJointGetPUPosition(self.id); }
    /// Time derivative of the prismatic position.
    pub fn get_position_rate(self: PU) Real { return c.dJointGetPUPositionRate(self.id); }
    /// Current rotation angle (radians) around universal axis 1.
    pub fn get_angle1(self: PU) Real { return c.dJointGetPUAngle1(self.id); }
    /// Current rotation angle (radians) around universal axis 2.
    pub fn get_angle2(self: PU) Real { return c.dJointGetPUAngle2(self.id); }
    /// Angular velocity (radians/second) around universal axis 1.
    pub fn get_angle1_rate(self: PU) Real { return c.dJointGetPUAngle1Rate(self.id); }
    /// Angular velocity (radians/second) around universal axis 2.
    pub fn get_angle2_rate(self: PU) Real { return c.dJointGetPUAngle2Rate(self.id); }
    /// Set a joint parameter. Use suffixed params for additional axes.
    pub fn set_param(self: PU, parameter: Param, value: Real) void { c.dJointSetPUParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter. Use suffixed params for additional axes.
    pub fn get_param(self: PU, parameter: Param) Real { return c.dJointGetPUParam(self.id, @intFromEnum(parameter)); }
};

/// Piston joint -- allows both translation along and rotation around a single
/// axis, like a real piston that can also spin. Combines the freedoms of a
/// slider and a hinge sharing the same axis.
pub const Piston = struct {
    id: c.dJointID,

    const methods = JointMethods(Piston);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a piston joint in the given world.
    pub fn create(world: World, group: ?Group) Piston {
        return .{ .id = c.dJointCreatePiston(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point in world coordinates.
    pub fn set_anchor(self: Piston, anchor: [3]Real) void { c.dJointSetPistonAnchor(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Read back the anchor on body 1 in world coordinates.
    pub fn get_anchor(self: Piston) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPistonAnchor(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Read back the anchor on body 2. Drift from `get_anchor` shows joint error.
    pub fn get_anchor2(self: Piston) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPistonAnchor2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the piston axis (shared slide + rotation axis) in world coordinates.
    pub fn set_axis(self: Piston, axis: [3]Real) void { c.dJointSetPistonAxis(self.id, axis[0], axis[1], axis[2]); }
    /// Get the piston axis in world coordinates.
    pub fn get_axis(self: Piston) [3]Real { var r: c.dVector3 = undefined; c.dJointGetPistonAxis(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Linear displacement along the piston axis.
    pub fn get_position(self: Piston) Real { return c.dJointGetPistonPosition(self.id); }
    /// Linear velocity along the piston axis.
    pub fn get_position_rate(self: Piston) Real { return c.dJointGetPistonPositionRate(self.id); }
    /// Rotation angle (radians) around the piston axis.
    pub fn get_angle(self: Piston) Real { return c.dJointGetPistonAngle(self.id); }
    /// Angular velocity (radians/second) around the piston axis.
    pub fn get_angle_rate(self: Piston) Real { return c.dJointGetPistonAngleRate(self.id); }
    /// Set a joint parameter. Use suffixed params (e.g. `vel2`) for the rotational axis.
    pub fn set_param(self: Piston, parameter: Param, value: Real) void { c.dJointSetPistonParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter.
    pub fn get_param(self: Piston, parameter: Param) Real { return c.dJointGetPistonParam(self.id, @intFromEnum(parameter)); }
    /// Apply a force along the piston axis to both attached bodies.
    pub fn add_force(self: Piston, force: Real) void { c.dJointAddPistonForce(self.id, force); }
};

/// Distance-preserving ball joint -- a spring-like ball-and-socket that
/// maintains a target distance between the two anchor points rather than
/// requiring them to coincide. Useful for soft constraints and ragdolls.
pub const DBall = struct {
    id: c.dJointID,

    const methods = JointMethods(DBall);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a distance ball joint in the given world.
    pub fn create(world: World, group: ?Group) DBall {
        return .{ .id = c.dJointCreateDBall(world.id, if (group) |g| g.id else null) };
    }

    /// Set the anchor point on body 1 in world coordinates.
    pub fn set_anchor1(self: DBall, anchor: [3]Real) void { c.dJointSetDBallAnchor1(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Set the anchor point on body 2 in world coordinates.
    pub fn set_anchor2(self: DBall, anchor: [3]Real) void { c.dJointSetDBallAnchor2(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Get the anchor point on body 1 in world coordinates.
    pub fn get_anchor1(self: DBall) [3]Real { var r: c.dVector3 = undefined; c.dJointGetDBallAnchor1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Get the anchor point on body 2 in world coordinates.
    pub fn get_anchor2(self: DBall) [3]Real { var r: c.dVector3 = undefined; c.dJointGetDBallAnchor2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Explicitly set the target distance maintained between the two anchors.
    pub fn set_distance(self: DBall, dist: Real) void { c.dJointSetDBallDistance(self.id, dist); }
    /// Get the target distance between the two anchors.
    pub fn get_distance(self: DBall) Real { return c.dJointGetDBallDistance(self.id); }
    /// Set a joint parameter (e.g. ERP/CFM for spring/damper tuning).
    pub fn set_param(self: DBall, parameter: Param, value: Real) void { c.dJointSetDBallParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter.
    pub fn get_param(self: DBall, parameter: Param) Real { return c.dJointGetDBallParam(self.id, @intFromEnum(parameter)); }
};

/// Distance-preserving hinge -- a spring-like hinge that maintains a target
/// distance between the two anchor points while constraining rotation to a
/// single axis. Combines distance preservation with hinge behavior.
pub const DHinge = struct {
    id: c.dJointID,

    const methods = JointMethods(DHinge);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a distance hinge joint in the given world.
    pub fn create(world: World, group: ?Group) DHinge {
        return .{ .id = c.dJointCreateDHinge(world.id, if (group) |g| g.id else null) };
    }

    /// Set the hinge rotation axis in world coordinates.
    pub fn set_axis(self: DHinge, axis: [3]Real) void { c.dJointSetDHingeAxis(self.id, axis[0], axis[1], axis[2]); }
    /// Get the hinge rotation axis in world coordinates.
    pub fn get_axis(self: DHinge) [3]Real { var r: c.dVector3 = undefined; c.dJointGetDHingeAxis(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the anchor point on body 1 in world coordinates.
    pub fn set_anchor1(self: DHinge, anchor: [3]Real) void { c.dJointSetDHingeAnchor1(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Set the anchor point on body 2 in world coordinates.
    pub fn set_anchor2(self: DHinge, anchor: [3]Real) void { c.dJointSetDHingeAnchor2(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Get the anchor point on body 1 in world coordinates.
    pub fn get_anchor1(self: DHinge) [3]Real { var r: c.dVector3 = undefined; c.dJointGetDHingeAnchor1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Get the anchor point on body 2 in world coordinates.
    pub fn get_anchor2(self: DHinge) [3]Real { var r: c.dVector3 = undefined; c.dJointGetDHingeAnchor2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Get the target distance maintained between the two anchors.
    pub fn get_distance(self: DHinge) Real { return c.dJointGetDHingeDistance(self.id); }
    /// Set a joint parameter (e.g. ERP/CFM for spring/damper tuning).
    pub fn set_param(self: DHinge, parameter: Param, value: Real) void { c.dJointSetDHingeParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter.
    pub fn get_param(self: DHinge, parameter: Param) Real { return c.dJointGetDHingeParam(self.id, @intFromEnum(parameter)); }
};

/// Transmission joint -- couples two rotating bodies via a gear, belt, or
/// chain mechanism. Supports different transmission modes and configurable
/// gear ratios.
pub const Transmission = struct {
    id: c.dJointID,

    const methods = JointMethods(Transmission);
    pub const destroy = methods.destroy;
    pub const attach = methods.attach;
    pub const enable = methods.enable;
    pub const disable = methods.disable;
    pub const is_enabled = methods.is_enabled;
    pub const get_num_bodies = methods.get_num_bodies;
    pub const get_body = methods.get_body;
    pub const set_data = methods.set_data;
    pub const get_data = methods.get_data;
    pub const get_type = methods.get_type;
    pub const set_feedback = methods.set_feedback;
    pub const get_feedback = methods.get_feedback;
    pub const to_generic = methods.to_generic;

    /// Create a transmission joint in the given world.
    pub fn create(world: World, group: ?Group) Transmission {
        return .{ .id = c.dJointCreateTransmission(world.id, if (group) |g| g.id else null) };
    }

    /// Set the transmission mode: 0 = parallel axes, 1 = intersecting axes,
    /// 2 = chain drive.
    pub fn set_mode(self: Transmission, mode: c_int) void { c.dJointSetTransmissionMode(self.id, mode); }
    /// Get the current transmission mode.
    pub fn get_mode(self: Transmission) c_int { return c.dJointGetTransmissionMode(self.id); }
    /// Set the gear ratio (body1 angular velocity / body2 angular velocity).
    pub fn set_ratio(self: Transmission, ratio: Real) void { c.dJointSetTransmissionRatio(self.id, ratio); }
    /// Get the current gear ratio.
    pub fn get_ratio(self: Transmission) Real { return c.dJointGetTransmissionRatio(self.id); }
    /// Set the contact/attachment point on body 1 in world coordinates.
    pub fn set_anchor1(self: Transmission, anchor: [3]Real) void { c.dJointSetTransmissionAnchor1(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Get the contact/attachment point on body 1 in world coordinates.
    pub fn get_anchor1(self: Transmission) [3]Real { var r: c.dVector3 = undefined; c.dJointGetTransmissionAnchor1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the contact/attachment point on body 2 in world coordinates.
    pub fn set_anchor2(self: Transmission, anchor: [3]Real) void { c.dJointSetTransmissionAnchor2(self.id, anchor[0], anchor[1], anchor[2]); }
    /// Get the contact/attachment point on body 2 in world coordinates.
    pub fn get_anchor2(self: Transmission) [3]Real { var r: c.dVector3 = undefined; c.dJointGetTransmissionAnchor2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the rotation axis for body 1.
    pub fn set_axis1(self: Transmission, axis: [3]Real) void { c.dJointSetTransmissionAxis1(self.id, axis[0], axis[1], axis[2]); }
    /// Get the rotation axis for body 1.
    pub fn get_axis1(self: Transmission) [3]Real { var r: c.dVector3 = undefined; c.dJointGetTransmissionAxis1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the rotation axis for body 2.
    pub fn set_axis2(self: Transmission, axis: [3]Real) void { c.dJointSetTransmissionAxis2(self.id, axis[0], axis[1], axis[2]); }
    /// Get the rotation axis for body 2.
    pub fn get_axis2(self: Transmission) [3]Real { var r: c.dVector3 = undefined; c.dJointGetTransmissionAxis2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set both rotation axes to the same direction (convenience for parallel axes mode).
    pub fn set_axis(self: Transmission, axis: [3]Real) void { c.dJointSetTransmissionAxis(self.id, axis[0], axis[1], axis[2]); }
    /// Get the point where the belt/chain contacts wheel 1 (chain mode).
    pub fn get_contact_point1(self: Transmission) [3]Real { var r: c.dVector3 = undefined; c.dJointGetTransmissionContactPoint1(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Get the point where the belt/chain contacts wheel 2 (chain mode).
    pub fn get_contact_point2(self: Transmission) [3]Real { var r: c.dVector3 = undefined; c.dJointGetTransmissionContactPoint2(self.id, &r); return .{ r[0], r[1], r[2] }; }
    /// Set the wheel radius for body 1 (used in chain/belt modes).
    pub fn set_radius1(self: Transmission, radius: Real) void { c.dJointSetTransmissionRadius1(self.id, radius); }
    /// Get the wheel radius for body 1.
    pub fn get_radius1(self: Transmission) Real { return c.dJointGetTransmissionRadius1(self.id); }
    /// Set the wheel radius for body 2 (used in chain/belt modes).
    pub fn set_radius2(self: Transmission, radius: Real) void { c.dJointSetTransmissionRadius2(self.id, radius); }
    /// Get the wheel radius for body 2.
    pub fn get_radius2(self: Transmission) Real { return c.dJointGetTransmissionRadius2(self.id); }
    /// Set the backlash (free play) in the transmission mechanism.
    pub fn set_backlash(self: Transmission, backlash: Real) void { c.dJointSetTransmissionBacklash(self.id, backlash); }
    /// Get the backlash (free play) in the transmission mechanism.
    pub fn get_backlash(self: Transmission) Real { return c.dJointGetTransmissionBacklash(self.id); }
    /// Set a joint parameter.
    pub fn set_param(self: Transmission, parameter: Param, value: Real) void { c.dJointSetTransmissionParam(self.id, @intFromEnum(parameter), value); }
    /// Get a joint parameter.
    pub fn get_param(self: Transmission, parameter: Param) Real { return c.dJointGetTransmissionParam(self.id, @intFromEnum(parameter)); }
    /// Get the current rotation angle (radians) of body 1's wheel.
    pub fn get_angle1(self: Transmission) Real { return c.dJointGetTransmissionAngle1(self.id); }
    /// Get the current rotation angle (radians) of body 2's wheel.
    pub fn get_angle2(self: Transmission) Real { return c.dJointGetTransmissionAngle2(self.id); }
};
