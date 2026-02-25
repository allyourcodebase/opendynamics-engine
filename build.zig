const std = @import("std");

pub const Precision = enum { single, double };
pub const TrimeshLibrary = enum { none, opcode, gimpact, opcode_old };
pub const IndexSize = enum { u16, u32 };

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const precision = b.option(Precision, "precision", "ODE floating-point precision") orelse .single;
    const trimesh = b.option(TrimeshLibrary, "trimesh", "Trimesh collision library") orelse .opcode;
    const index_size = b.option(IndexSize, "index_size", "Trimesh index size") orelse .u16;
    const no_builtin_threading_impl = b.option(bool, "no_builtin_threading_impl", "Disable built-in multithreaded threading implementation") orelse false;
    const no_threading_intf = b.option(bool, "no_threading_intf", "Disable threading interface support") orelse false;
    const ou = b.option(bool, "ou", "Use TLS for global caches (allows threaded collision checks for separated spaces)") orelse false;
    const libccd = b.option(bool, "libccd", "Enable libccd collision detection") orelse false;
    const libccd_system = b.option(bool, "libccd_system", "Link system libccd instead of compiling internal") orelse false;
    const libccd_box_cyl = b.option(bool, "libccd_box_cyl", "Enable libccd box-cylinder collisions") orelse true;
    const libccd_cap_cyl = b.option(bool, "libccd_cap_cyl", "Enable libccd capsule-cylinder collisions") orelse true;
    const libccd_cyl_cyl = b.option(bool, "libccd_cyl_cyl", "Enable libccd cylinder-cylinder collisions") orelse true;
    const libccd_convex_box = b.option(bool, "libccd_convex_box", "Enable libccd convex-box collisions") orelse true;
    const libccd_convex_cap = b.option(bool, "libccd_convex_cap", "Enable libccd convex-capsule collisions") orelse true;
    const libccd_convex_convex = b.option(bool, "libccd_convex_convex", "Enable libccd convex-convex collisions") orelse true;
    const libccd_convex_cyl = b.option(bool, "libccd_convex_cyl", "Enable libccd convex-cylinder collisions") orelse true;
    const libccd_convex_sphere = b.option(bool, "libccd_convex_sphere", "Enable libccd convex-sphere collisions") orelse true;

    const upstream = b.dependency("ode", .{});

    const lib = b.addLibrary(.{
        .name = "ode",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
            .link_libcpp = true,
        }),
    });

    const lib_mod = lib.root_module;

    lib_mod.addCMacro("DODE_LIB", "1");

    // Shared include paths and precision macros (used by both C lib and Zig module)
    configureOdeIncludes(lib_mod, b, upstream, precision, trimesh, index_size);

    // Additional internal include paths (C lib only)
    lib_mod.addIncludePath(upstream.path("ode/src"));
    lib_mod.addIncludePath(upstream.path("ode/src/joints"));
    lib_mod.addIncludePath(upstream.path("ou/include"));

    // Platform-specific configuration
    if (target.result.os.tag.isDarwin()) {
        lib_mod.addCMacro("MAC_OS_X_VERSION", "1050");
    }

    // OU configuration
    lib_mod.addCMacro("_OU_NAMESPACE", "odeou");
    lib_mod.addCMacro("_OU_FEATURE_SET", if (ou)
        "_OU_FEATURE_SET_TLS"
    else if (!no_threading_intf)
        "_OU_FEATURE_SET_ATOMICS"
    else
        "_OU_FEATURE_SET_BASICS");
    lib_mod.addCMacro("dOU_ENABLED", "1");

    if (ou) {
        lib_mod.addCMacro("dATOMICS_ENABLED", "1");
        lib_mod.addCMacro("dTLS_ENABLED", "1");
    } else if (!no_threading_intf) {
        lib_mod.addCMacro("dATOMICS_ENABLED", "1");
    }

    // Threading configuration
    if (!no_builtin_threading_impl) {
        lib_mod.addCMacro("dBUILTIN_THREADING_IMPL_ENABLED", "1");
    }
    if (no_threading_intf) {
        lib_mod.addCMacro("dTHREADING_INTF_DISABLED", "1");
    }
    if (no_builtin_threading_impl and no_threading_intf) {
        lib_mod.single_threaded = true;
    }

    const cpp_flags: []const []const u8 = &.{
        "-Wno-implicit-int-float-conversion",
        "-Wno-implicit-float-conversion",
        "-Wno-implicit-const-int-float-conversion",
        "-Wno-deprecated-declarations",
        "-Wno-null-dereference",
    };

    // Disable ODE internal assertions in non-Debug builds (matches CMake behavior)
    if (optimize != .Debug) {
        lib_mod.addCMacro("dNODEBUG", "1");
    }

    // Core ODE sources
    lib_mod.addCSourceFiles(.{
        .root = upstream.path(""),
        .files = ode_sources,
        .flags = cpp_flags,
    });

    // lcp.cpp has an upstream bug: an uninitialized bool array is swapped before
    // being written, which is harmless but triggers UBSan. Compile it separately
    // with bool sanitization disabled so dWorldStep works in Debug builds.
    lib_mod.addCSourceFiles(.{
        .root = upstream.path(""),
        .files = &.{"ode/src/lcp.cpp"},
        .flags = cpp_flags ++ &[_][]const u8{"-fno-sanitize=bool"},
    });

    // Libccd configuration
    if (libccd) {
        lib_mod.addCMacro("dLIBCCD_ENABLED", "1");

        if (libccd_system) {
            lib_mod.addCMacro("dLIBCCD_SYSTEM", "1");
            lib_mod.linkSystemLibrary("ccd", .{});
        } else {
            lib_mod.addIncludePath(upstream.path("libccd/src"));
            lib_mod.addCMacro("dLIBCCD_INTERNAL", "1");
            lib_mod.addCSourceFiles(.{
                .root = upstream.path(""),
                .files = libccd_sources,
                .flags = cpp_flags,
            });
        }

        lib_mod.addCSourceFiles(.{
            .root = upstream.path(""),
            .files = libccd_addon_sources,
            .flags = cpp_flags,
        });
        lib_mod.addIncludePath(upstream.path("libccd/src/custom"));

        if (libccd_box_cyl) lib_mod.addCMacro("dLIBCCD_BOX_CYL", "1");
        if (libccd_cap_cyl) lib_mod.addCMacro("dLIBCCD_CAP_CYL", "1");
        if (libccd_cyl_cyl) lib_mod.addCMacro("dLIBCCD_CYL_CYL", "1");
        if (libccd_convex_box) lib_mod.addCMacro("dLIBCCD_CONVEX_BOX", "1");
        if (libccd_convex_cap) lib_mod.addCMacro("dLIBCCD_CONVEX_CAP", "1");
        if (libccd_convex_convex) lib_mod.addCMacro("dLIBCCD_CONVEX_CONVEX", "1");
        if (libccd_convex_cyl) lib_mod.addCMacro("dLIBCCD_CONVEX_CYL", "1");
        if (libccd_convex_sphere) lib_mod.addCMacro("dLIBCCD_CONVEX_SPHERE", "1");
    }

    // Trimesh configuration
    switch (trimesh) {
        .none => {},
        .opcode, .opcode_old => {
            lib_mod.addCSourceFiles(.{
                .root = upstream.path(""),
                .files = opcode_sources,
                .flags = cpp_flags,
            });

            if (trimesh == .opcode_old) {
                lib_mod.addCMacro("dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER", "1");
            }

            lib_mod.addIncludePath(upstream.path("OPCODE"));
            lib_mod.addIncludePath(upstream.path("OPCODE/Ice"));
        },
        .gimpact => {
            lib_mod.addCSourceFiles(.{
                .root = upstream.path(""),
                .files = gimpact_sources,
                .flags = cpp_flags,
            });

            lib_mod.addIncludePath(upstream.path("GIMPACT/include"));
        },
    }

    b.installArtifact(lib);

    // Zig module
    const mod = b.addModule("ode", .{
        .root_source_file = b.path("src/ode.zig"),
    });
    configureOdeIncludes(mod, b, upstream, precision, trimesh, index_size);
    mod.linkLibrary(lib);

    // Tests
    const tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/ode.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    configureOdeIncludes(tests.root_module, b, upstream, precision, trimesh, index_size);
    tests.root_module.linkLibrary(lib);
    const run_tests = b.addRunArtifact(tests);
    b.step("test", "Run ODE binding tests").dependOn(&run_tests.step);

    // Examples
    const examples = [_]struct { name: []const u8, path: []const u8 }{
        .{ .name = "chain", .path = "examples/chain.zig" },
        .{ .name = "hinge", .path = "examples/hinge.zig" },
        .{ .name = "slider", .path = "examples/slider.zig" },
    };

    for (examples) |ex| {
        const exe = b.addExecutable(.{
            .name = ex.name,
            .root_module = b.createModule(.{
                .root_source_file = b.path(ex.path),
                .target = target,
                .optimize = optimize,
                .imports = &.{.{ .name = "ode", .module = mod }},
            }),
        });
        b.installArtifact(exe);
        const run = b.addRunArtifact(exe);
        b.step(b.fmt("example-{s}", .{ex.name}), b.fmt("Run {s} example", .{ex.name})).dependOn(&run.step);
    }
}

fn configureOdeIncludes(
    m: *std.Build.Module,
    b: *std.Build,
    upstream: *std.Build.Dependency,
    precision: Precision,
    trimesh: TrimeshLibrary,
    index_size: IndexSize,
) void {
    m.addIncludePath(b.path("include/common"));
    m.addIncludePath(upstream.path("include"));
    switch (precision) {
        .single => {
            m.addIncludePath(b.path("include/single"));
            m.addCMacro("dIDESINGLE", "1");
            m.addCMacro("CCD_IDESINGLE", "1");
        },
        .double => {
            m.addIncludePath(b.path("include/double"));
            m.addCMacro("dIDEDOUBLE", "1");
            m.addCMacro("CCD_IDEDOUBLE", "1");
        },
    }
    switch (trimesh) {
        .none => {},
        .opcode, .opcode_old => {
            m.addCMacro("dTRIMESH_ENABLED", "1");
            m.addCMacro("dTRIMESH_OPCODE", "1");
        },
        .gimpact => {
            m.addCMacro("dTRIMESH_ENABLED", "1");
            m.addCMacro("dTRIMESH_GIMPACT", "1");
        },
    }
    switch (index_size) {
        .u16 => m.addCMacro("dTRIMESH_16BIT_INDICES", "1"),
        .u32 => {},
    }
}

const ode_sources: []const []const u8 = &.{
    "ode/src/array.cpp",
    "ode/src/box.cpp",
    "ode/src/capsule.cpp",
    "ode/src/collision_cylinder_box.cpp",
    "ode/src/collision_cylinder_plane.cpp",
    "ode/src/collision_cylinder_sphere.cpp",
    "ode/src/collision_kernel.cpp",
    "ode/src/collision_quadtreespace.cpp",
    "ode/src/collision_sapspace.cpp",
    "ode/src/collision_space.cpp",
    "ode/src/collision_transform.cpp",
    "ode/src/collision_trimesh_disabled.cpp",
    "ode/src/collision_util.cpp",
    "ode/src/convex.cpp",
    "ode/src/cylinder.cpp",
    "ode/src/default_threading.cpp",
    "ode/src/error.cpp",
    "ode/src/export-dif.cpp",
    "ode/src/fastdot.cpp",
    "ode/src/fastldltfactor.cpp",
    "ode/src/fastldltsolve.cpp",
    "ode/src/fastlsolve.cpp",
    "ode/src/fastltsolve.cpp",
    "ode/src/fastvecscale.cpp",
    "ode/src/heightfield.cpp",
    // lcp.cpp compiled separately with -fno-sanitize=bool (see above)
    "ode/src/mass.cpp",
    "ode/src/mat.cpp",
    "ode/src/matrix.cpp",
    "ode/src/memory.cpp",
    "ode/src/misc.cpp",
    "ode/src/nextafterf.c",
    "ode/src/objects.cpp",
    "ode/src/obstack.cpp",
    "ode/src/ode.cpp",
    "ode/src/odeinit.cpp",
    "ode/src/odemath.cpp",
    "ode/src/plane.cpp",
    "ode/src/quickstep.cpp",
    "ode/src/ray.cpp",
    "ode/src/resource_control.cpp",
    "ode/src/rotation.cpp",
    "ode/src/simple_cooperative.cpp",
    "ode/src/sphere.cpp",
    "ode/src/step.cpp",
    "ode/src/threading_base.cpp",
    "ode/src/threading_impl.cpp",
    "ode/src/threading_pool_posix.cpp",
    "ode/src/threading_pool_win.cpp",
    "ode/src/timer.cpp",
    "ode/src/util.cpp",
    "ode/src/joints/amotor.cpp",
    "ode/src/joints/ball.cpp",
    "ode/src/joints/contact.cpp",
    "ode/src/joints/dball.cpp",
    "ode/src/joints/dhinge.cpp",
    "ode/src/joints/fixed.cpp",
    "ode/src/joints/hinge.cpp",
    "ode/src/joints/hinge2.cpp",
    "ode/src/joints/joint.cpp",
    "ode/src/joints/lmotor.cpp",
    "ode/src/joints/null.cpp",
    "ode/src/joints/piston.cpp",
    "ode/src/joints/plane2d.cpp",
    "ode/src/joints/pr.cpp",
    "ode/src/joints/pu.cpp",
    "ode/src/joints/slider.cpp",
    "ode/src/joints/transmission.cpp",
    "ode/src/joints/universal.cpp",
    // OU:
    "ode/src/odeou.cpp",
    "ode/src/odetls.cpp",
    "ou/src/ou/atomic.cpp",
    "ou/src/ou/customization.cpp",
    "ou/src/ou/malloc.cpp",
    "ou/src/ou/threadlocalstorage.cpp",
};

const opcode_sources: []const []const u8 = &.{
    "ode/src/collision_convex_trimesh.cpp",
    "ode/src/collision_cylinder_trimesh.cpp",
    "ode/src/collision_trimesh_box.cpp",
    "ode/src/collision_trimesh_ccylinder.cpp",
    "ode/src/collision_trimesh_internal.cpp",
    "ode/src/collision_trimesh_opcode.cpp",
    "ode/src/collision_trimesh_plane.cpp",
    "ode/src/collision_trimesh_ray.cpp",
    "ode/src/collision_trimesh_sphere.cpp",
    "ode/src/collision_trimesh_trimesh.cpp",
    "ode/src/collision_trimesh_trimesh_old.cpp",
    "OPCODE/OPC_AABBCollider.cpp",
    "OPCODE/OPC_AABBTree.cpp",
    "OPCODE/OPC_BaseModel.cpp",
    "OPCODE/OPC_Collider.cpp",
    "OPCODE/OPC_Common.cpp",
    "OPCODE/OPC_HybridModel.cpp",
    "OPCODE/OPC_LSSCollider.cpp",
    "OPCODE/OPC_MeshInterface.cpp",
    "OPCODE/OPC_Model.cpp",
    "OPCODE/OPC_OBBCollider.cpp",
    "OPCODE/OPC_OptimizedTree.cpp",
    "OPCODE/OPC_Picking.cpp",
    "OPCODE/OPC_PlanesCollider.cpp",
    "OPCODE/OPC_RayCollider.cpp",
    "OPCODE/OPC_SphereCollider.cpp",
    "OPCODE/OPC_TreeBuilders.cpp",
    "OPCODE/OPC_TreeCollider.cpp",
    "OPCODE/OPC_VolumeCollider.cpp",
    "OPCODE/Opcode.cpp",
    "OPCODE/Ice/IceAABB.cpp",
    "OPCODE/Ice/IceContainer.cpp",
    "OPCODE/Ice/IceHPoint.cpp",
    "OPCODE/Ice/IceIndexedTriangle.cpp",
    "OPCODE/Ice/IceMatrix3x3.cpp",
    "OPCODE/Ice/IceMatrix4x4.cpp",
    "OPCODE/Ice/IceOBB.cpp",
    "OPCODE/Ice/IcePlane.cpp",
    "OPCODE/Ice/IcePoint.cpp",
    "OPCODE/Ice/IceRandom.cpp",
    "OPCODE/Ice/IceRay.cpp",
    "OPCODE/Ice/IceRevisitedRadix.cpp",
    "OPCODE/Ice/IceSegment.cpp",
    "OPCODE/Ice/IceTriangle.cpp",
    "OPCODE/Ice/IceUtils.cpp",
};

const gimpact_sources: []const []const u8 = &.{
    "GIMPACT/src/gim_boxpruning.cpp",
    "GIMPACT/src/gim_contact.cpp",
    "GIMPACT/src/gim_math.cpp",
    "GIMPACT/src/gim_memory.cpp",
    "GIMPACT/src/gim_tri_tri_overlap.cpp",
    "GIMPACT/src/gim_trimesh.cpp",
    "GIMPACT/src/gim_trimesh_capsule_collision.cpp",
    "GIMPACT/src/gim_trimesh_ray_collision.cpp",
    "GIMPACT/src/gim_trimesh_sphere_collision.cpp",
    "GIMPACT/src/gim_trimesh_trimesh_collision.cpp",
    "GIMPACT/src/gimpact.cpp",
    "ode/src/collision_convex_trimesh.cpp",
    "ode/src/collision_cylinder_trimesh.cpp",
    "ode/src/collision_trimesh_box.cpp",
    "ode/src/collision_trimesh_ccylinder.cpp",
    "ode/src/collision_trimesh_gimpact.cpp",
    "ode/src/collision_trimesh_internal.cpp",
    "ode/src/collision_trimesh_plane.cpp",
    "ode/src/collision_trimesh_ray.cpp",
    "ode/src/collision_trimesh_sphere.cpp",
    "ode/src/collision_trimesh_trimesh.cpp",
    "ode/src/gimpact_contact_export_helper.cpp",
};

const libccd_sources: []const []const u8 = &.{
    "libccd/src/alloc.c",
    "libccd/src/ccd.c",
    "libccd/src/mpr.c",
    "libccd/src/polytope.c",
    "libccd/src/support.c",
    "libccd/src/vec3.c",
};

const libccd_addon_sources: []const []const u8 = &.{
    "ode/src/collision_libccd.cpp",
};
