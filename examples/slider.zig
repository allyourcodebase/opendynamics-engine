/// Headless port of ODE demo_slider.cpp
///
/// Two boxes on a slider (prismatic) joint with a manual spring force
/// holding them together and oscillating torque. No collision.
/// Prints the slider position and velocity each step.
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

    // Body 1
    const body1 = world.create_body();
    defer body1.destroy();
    body1.set_position(.{ 0, 0, 1.5 });
    var mass1 = ode.Mass.box(1.0, 0.3, 0.3, 0.3);
    mass1.adjust(1.0);
    body1.set_mass(&mass1);

    // Body 2
    const body2 = world.create_body();
    defer body2.destroy();
    body2.set_position(.{ 0.5, 0, 1.5 });
    var mass2 = ode.Mass.box(1.0, 0.3, 0.3, 0.3);
    mass2.adjust(1.0);
    body2.set_mass(&mass2);

    // Slider joint along X axis
    const slider = ode.Joint.Slider.create(world, null);
    defer slider.destroy();
    slider.attach(body1, body2);
    slider.set_axis(.{ 1, 0, 0 });

    // Pin body1 to world with a hinge so the whole assembly doesn't fall
    const anchor_hinge = ode.Joint.Hinge.create(world, null);
    defer anchor_hinge.destroy();
    anchor_hinge.attach(body1, null);
    anchor_hinge.set_anchor(.{ 0, 0, 1.5 });
    anchor_hinge.set_axis(.{ 0, 1, 0 });

    try stdout.interface.print("step,position,velocity\n", .{});

    for (0..NUM_STEPS) |step| {
        // Spring force pulling bodies together (Hooke's law)
        const pos = slider.get_position();
        const vel = slider.get_position_rate();
        const spring_k: Real = 5.0;
        const damping_c: Real = 0.5;
        const spring_force = -spring_k * pos - damping_c * vel;
        slider.add_force(spring_force);

        // Oscillating torque on the anchor hinge
        const t: Real = @as(Real, @floatFromInt(step)) * STEP_SIZE;
        const torque = 0.3 * @sin(t * 2.0);
        anchor_hinge.add_torque(torque);

        // Angular damping
        const av1 = body1.get_angular_vel();
        body1.add_torque(.{ -0.1 * av1[0], -0.1 * av1[1], -0.1 * av1[2] });
        const av2 = body2.get_angular_vel();
        body2.add_torque(.{ -0.1 * av2[0], -0.1 * av2[1], -0.1 * av2[2] });

        _ = world.step(STEP_SIZE);

        const final_pos = slider.get_position();
        const final_vel = slider.get_position_rate();
        try stdout.interface.print("{d},{d:.6},{d:.6}\n", .{ step, final_pos, final_vel });
    }
    try stdout.interface.flush();
}
