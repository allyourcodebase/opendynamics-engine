/// Headless port of ODE demo_hinge.cpp
///
/// Two boxes connected by a hinge joint. Oscillating torque is applied and
/// angular velocity damping keeps things stable. No collision.
/// Prints the hinge angle and angular rate each step.
const std = @import("std");
const ode = @import("ode");

const Real = ode.Real;

const STEP_SIZE = 0.05;
const NUM_STEPS = 200;

pub fn main() !void {
    var buf: [4096]u8 = undefined;
    var stdout = std.fs.File.stdout().writer(&buf);

    std.debug.assert(ode.init.init_ode2(.{}));
    defer ode.init.close_ode();

    const world = ode.World.create();
    defer world.destroy();

    world.set_gravity(.{ 0, 0, -9.81 });

    // Body 1: fixed to the world via the hinge
    const body1 = world.create_body();
    defer body1.destroy();
    body1.set_position(.{ 0, 0, 1.0 });
    var mass1 = ode.Mass.box(1.0, 0.4, 0.4, 0.4);
    mass1.adjust(1.0);
    body1.set_mass(&mass1);

    // Body 2: connected to body1 via hinge
    const body2 = world.create_body();
    defer body2.destroy();
    body2.set_position(.{ 0.6, 0, 1.0 });
    var mass2 = ode.Mass.box(1.0, 0.4, 0.4, 0.4);
    mass2.adjust(1.0);
    body2.set_mass(&mass2);

    // Create hinge joint between body1 and body2
    const hinge = ode.Joint.Hinge.create(world, null);
    defer hinge.destroy();
    hinge.attach(body1, body2);
    hinge.set_anchor(.{ 0.3, 0, 1.0 });
    hinge.set_axis(.{ 0, 1, 0 }); // rotate around Y

    // Pin body1 to the world with a fixed joint equivalent:
    // attach body1 to static environment with a hinge at its center
    const anchor_hinge = ode.Joint.Hinge.create(world, null);
    defer anchor_hinge.destroy();
    anchor_hinge.attach(body1, null);
    anchor_hinge.set_anchor(.{ 0, 0, 1.0 });
    anchor_hinge.set_axis(.{ 0, 1, 0 });

    try stdout.interface.print("step,angle,angle_rate\n", .{});

    for (0..NUM_STEPS) |step| {
        // Apply oscillating torque about the hinge axis
        const t: Real = @as(Real, @floatFromInt(step)) * STEP_SIZE;
        const torque = 0.5 * @sin(t * 3.0);
        hinge.add_torque(torque);

        // Simple angular velocity damping on both bodies
        const av1 = body1.get_angular_vel();
        body1.add_torque(.{ -0.1 * av1[0], -0.1 * av1[1], -0.1 * av1[2] });
        const av2 = body2.get_angular_vel();
        body2.add_torque(.{ -0.1 * av2[0], -0.1 * av2[1], -0.1 * av2[2] });

        _ = world.step(STEP_SIZE);

        const angle = hinge.get_angle();
        const rate = hinge.get_angle_rate();
        try stdout.interface.print("{d},{d:.6},{d:.6}\n", .{ step, angle, rate });
    }
    try stdout.interface.flush();
}
