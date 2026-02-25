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

test "collision: sphere-plane contacts" {
    const std = @import("std");
    std.debug.assert(init.init_ode2(.{}));
    defer init.close_ode();
    std.debug.assert(init.allocate_data_for_thread(.{ .collision_data = true }));

    // Ground plane at z=0 with normal pointing up
    const plane = Geom.Plane.create(null, 0, 0, 1, 0);
    defer plane.destroy();

    // Sphere touching the ground (center at z=0.4, radius 0.5 => penetration 0.1)
    const sphere = Geom.Sphere.create(null, 0.5);
    defer sphere.destroy();
    sphere.set_position(.{ 0, 0, 0.4 });

    var contact_geoms: [4]collision.ContactGeom = undefined;
    const contacts = collision.collide(sphere.to_generic(), plane.to_generic(), &contact_geoms);

    try std.testing.expect(contacts.len > 0);
    // Contact normal should point up (positive z)
    try std.testing.expect(contacts[0].normal[2] > 0.5);
}

test "joint: hinge constrains rotation" {
    const std = @import("std");
    std.debug.assert(init.init_ode2(.{}));
    defer init.close_ode();

    const world = World.create();
    defer world.destroy();
    world.set_gravity(.{ 0, 0, -9.81 });

    // Single body attached to the static environment via a hinge
    // so it swings like a pendulum under gravity.
    const body = world.create_body();
    defer body.destroy();
    body.set_position(.{ 1, 0, 0 });
    var m = Mass.box(1.0, 0.2, 0.2, 0.2);
    body.set_mass(&m);

    const hinge = Joint.Hinge.create(world, null);
    defer hinge.destroy();
    hinge.attach(body, null);
    hinge.set_anchor(.{ 0, 0, 0 });
    hinge.set_axis(.{ 0, 1, 0 });

    const angle_before = hinge.get_angle();

    // Step a few times
    for (0..10) |_| {
        _ = world.step(0.05);
    }

    const angle_after = hinge.get_angle();
    // Hinge angle should change as body swings down
    try std.testing.expect(@abs(angle_after - angle_before) > 0.01);
}

test "space: add/remove/query geoms" {
    const std = @import("std");
    std.debug.assert(init.init_ode2(.{}));
    defer init.close_ode();
    std.debug.assert(init.allocate_data_for_thread(.{ .collision_data = true }));

    const space = Space.Hash.create(null);
    defer space.destroy();

    const s1 = Geom.Sphere.create(null, 0.5);
    defer s1.destroy();
    const s2 = Geom.Sphere.create(null, 0.5);
    defer s2.destroy();

    const gs = space.to_generic();

    gs.add(s1.to_generic());
    gs.add(s2.to_generic());
    try std.testing.expectEqual(@as(c_int, 2), gs.get_num_geoms());
    try std.testing.expect(gs.query(s1.to_generic()));
    try std.testing.expect(gs.query(s2.to_generic()));

    gs.remove(s1.to_generic());
    try std.testing.expectEqual(@as(c_int, 1), gs.get_num_geoms());
    try std.testing.expect(!gs.query(s1.to_generic()));
    try std.testing.expect(gs.query(s2.to_generic()));

    gs.remove(s2.to_generic());
    try std.testing.expectEqual(@as(c_int, 0), gs.get_num_geoms());
}

test "mass: sphere properties" {
    const std = @import("std");
    std.debug.assert(init.init_ode2(.{}));
    defer init.close_ode();

    var m = Mass.sphere(1.0, 0.5);
    try std.testing.expect(m.check());
    const original_mass = m.get_mass();
    try std.testing.expect(original_mass > 0);

    m.adjust(5.0);
    try std.testing.expectApproxEqAbs(@as(Real, 5.0), m.get_mass(), 1e-5);
    try std.testing.expect(m.check());

    // Center of gravity should be at origin for a uniform sphere
    const center = m.get_center();
    try std.testing.expectApproxEqAbs(@as(Real, 0), center[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, 0), center[1], 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, 0), center[2], 1e-5);
}

test "body: force accumulation" {
    const std = @import("std");
    std.debug.assert(init.init_ode2(.{}));
    defer init.close_ode();

    const world = World.create();
    defer world.destroy();
    world.set_gravity(.{ 0, 0, 0 }); // no gravity for this test

    const body = world.create_body();
    defer body.destroy();
    body.set_position(.{ 0, 0, 0 });
    var m = Mass.sphere(1.0, 0.5);
    m.adjust(1.0);
    body.set_mass(&m);

    // Add force and check accumulator
    body.add_force(.{ 10, 0, 0 });
    const f = body.get_force();
    try std.testing.expectApproxEqAbs(@as(Real, 10), f[0], 1e-5);
    try std.testing.expectApproxEqAbs(@as(Real, 0), f[1], 1e-5);

    // Step and verify position changed due to force
    _ = world.step(0.1);
    const pos = body.get_position();
    try std.testing.expect(pos[0] > 0);

    // Force should be cleared after step
    const f_after = body.get_force();
    try std.testing.expectApproxEqAbs(@as(Real, 0), f_after[0], 1e-5);
}
