//! Idiomatic Zig bindings for the Open Dynamics Engine (ODE), a rigid body physics library.
//!
//! Call `init.init_ode2(.{})` before using any other functions, and `init.close_ode()` when done.
//! For multithreaded use, each thread must also call `init.allocate_data_for_thread(.{ .collision_data = true })`.

pub const c = @import("c.zig").c;

/// Floating-point type used throughout ODE. Either f32 or f64 depending on build-time precision setting.
pub const Real = c.dReal;
/// 3-component vector (x, y, z).
pub const Vector3 = [3]Real;
/// 4-component vector (x, y, z, w).
pub const Vector4 = [4]Real;
/// 3x4 row-major rotation matrix. Each of the 3 rows has a 4th padding element (12 floats total).
pub const Matrix3 = [12]Real;
/// 4x4 row-major matrix.
pub const Matrix4 = [16]Real;
/// Quaternion stored as (w, x, y, z).
pub const Quaternion = [4]Real;

/// Library initialization and shutdown.
pub const init = @import("init.zig");
/// Simulation world containing global parameters (gravity, ERP, CFM) and stepping functions.
pub const World = @import("World.zig");
/// Rigid body with position, velocity, mass, and force accumulators.
pub const Body = @import("Body.zig");
/// Mass distribution (total mass, center of gravity, inertia tensor) assignable to a Body.
pub const Mass = @import("Mass.zig");
/// Constraints between bodies: ball-and-socket, hinges, sliders, motors, and more.
pub const Joint = @import("Joint.zig");
/// Spatial indexing structures that accelerate broad-phase collision detection.
pub const Space = @import("Space.zig");
/// Collision geometry (shapes) that can be attached to bodies or placed as static environment.
pub const Geom = @import("Geom.zig");
/// Terrain geometry defined by a grid of height samples.
pub const Heightfield = @import("Heightfield.zig");
/// Utility functions for building rotation matrices and quaternions from axes, angles, and Euler angles.
pub const Rotation = @import("Rotation.zig");
/// Narrow-phase collision testing and contact data structures.
pub const collision = @import("collision.zig");

test {
    @import("std").testing.refAllDecls(@This());
}

test "smoke: init, world, body, step" {
    const std = @import("std");

    // Init ODE
    std.debug.assert(init.init_ode2(.{}));
    defer init.close_ode();
    std.debug.assert(init.allocate_data_for_thread(.{ .collision_data = true }));

    // Create world
    const world = World.create();
    defer world.destroy();
    world.set_gravity(.{ 0, -9.81, 0 });

    const gravity = world.get_gravity();
    try std.testing.expectApproxEqAbs(@as(Real, 0), gravity[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, -9.81), gravity[1], 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, 0), gravity[2], 1e-5);

    // Create body
    const body = world.create_body();
    defer body.destroy();

    var mass = Mass.sphere(1.0, 0.5);
    body.set_mass(&mass);
    body.set_position(.{ 0, 10, 0 });

    const pos_before = body.get_position();
    try std.testing.expectApproxEqAbs(@as(Real, 10), pos_before[1], 1e-5);

    // Step simulation
    _ = world.step(0.01);

    // Verify position changed (body should fall due to gravity)
    const pos_after = body.get_position();
    try std.testing.expect(pos_after[1] < pos_before[1]);
}
