/// Headless port of ODE demo_chain1.c
///
/// 10 spheres linked by ball-and-socket joints, falling under gravity,
/// colliding with a ground plane. An oscillating force drives the chain end.
/// Prints the position of the last body each step.
const std = @import("std");
const ode = @import("ode");

const Real = ode.Real;
const Body = ode.Body;
const World = ode.World;
const Mass = ode.Mass;
const Joint = ode.Joint;
const Space = ode.Space;
const Geom = ode.Geom;
const collision = ode.collision;

const NUM = 10;
const SIDE = 0.2;
const RADIUS = 0.1732;
const STEP_SIZE = 0.05;
const NUM_STEPS = 200;
const MAX_CONTACTS = 4;

var world: World = undefined;
var contact_group: Joint.Group = undefined;

var bodies: [NUM]Body = undefined;
var joints: [NUM - 1]Joint.Ball = undefined;
var spheres: [NUM]Geom.Sphere = undefined;

fn near_callback(_: ?*anyopaque, o1: ode.c.dGeomID, o2: ode.c.dGeomID) callconv(.c) void {
    const g1 = Geom.Generic{ .id = o1 };
    const g2 = Geom.Generic{ .id = o2 };

    var contact_geoms: [MAX_CONTACTS]collision.ContactGeom = undefined;
    const contacts = collision.collide(g1, g2, &contact_geoms);
    if (contacts.len == 0) return;

    for (contacts) |*cg| {
        var contact: collision.Contact = std.mem.zeroes(collision.Contact);
        contact.surface.mode = collision.contact_bounce | collision.contact_soft_cfm;
        contact.surface.mu = @as(Real, std.math.inf(f64));
        contact.surface.bounce = 0.1;
        contact.surface.bounce_vel = 0.1;
        contact.surface.soft_cfm = 0.01;
        contact.geom = cg.*;

        const cj = Joint.Contact.create(world, contact_group, &contact);
        cj.attach(g1.get_body(), g2.get_body());
    }
}

pub fn main() !void {
    var buf: [4096]u8 = undefined;
    var stdout = std.fs.File.stdout().writer(&buf);

    std.debug.assert(ode.init.init_ode2(.{}));
    defer ode.init.close_ode();
    std.debug.assert(ode.init.allocate_data_for_thread(.{ .collision_data = true }));

    world = World.create();
    defer world.destroy();

    const space = Space.Hash.create(null);
    defer space.destroy();

    contact_group = Joint.Group.create();
    defer contact_group.destroy();

    world.set_gravity(.{ 0, 0, -0.5 });
    _ = Geom.Plane.create(space.to_generic(), 0, 0, 1, 0);

    // Create bodies, masses, and geoms
    for (0..NUM) |i| {
        bodies[i] = world.create_body();
        const k: Real = @floatFromInt(i);
        bodies[i].set_position(.{ k * SIDE, 0, k * SIDE + 0.4 });

        var mass = Mass.sphere(1.0, RADIUS);
        mass.adjust(1.0);
        bodies[i].set_mass(&mass);

        spheres[i] = Geom.Sphere.create(space.to_generic(), RADIUS);
        spheres[i].set_body(bodies[i]);
    }

    // Create ball joints between adjacent bodies
    for (0..NUM - 1) |i| {
        joints[i] = Joint.Ball.create(world, null);
        joints[i].attach(bodies[i], bodies[i + 1]);
        const k: Real = @floatFromInt(i);
        joints[i].set_anchor(.{ (k + 0.5) * SIDE, 0, (k + 0.5) * SIDE + 0.4 });
    }

    try stdout.interface.print("step,x,y,z\n", .{});

    for (0..NUM_STEPS) |step| {
        // Apply oscillating force to the last body
        const t: Real = @as(Real, @floatFromInt(step)) * STEP_SIZE;
        const fx = 0.3 * @sin(t * 4.0);
        const fz = 0.3 * @cos(t * 4.0);
        bodies[NUM - 1].add_force(.{ fx, 0, fz });

        // Collision detection
        space.collide(null, &near_callback);

        // Step the world
        _ = world.step(STEP_SIZE);

        // Remove contact joints
        contact_group.empty();

        // Print the last body's position
        const pos = bodies[NUM - 1].get_position();
        try stdout.interface.print("{d},{d:.6},{d:.6},{d:.6}\n", .{ step, pos[0], pos[1], pos[2] });
    }
    try stdout.interface.flush();

    // Cleanup bodies and geoms
    for (0..NUM) |i| {
        spheres[i].destroy();
        bodies[i].destroy();
    }
    for (0..NUM - 1) |i| {
        joints[i].destroy();
    }
}
