//! ODE library initialization and shutdown. Must be called before and after all other ODE usage.

const c = @import("c.zig").c;

/// Flags for `init_ode2`.
pub const InitFlags = packed struct(c_uint) {
    /// When set, ODE will not automatically clean up thread-local data on thread exit.
    /// You must call `cleanup_all_data_for_thread` manually before each thread terminates.
    manual_thread_cleanup: bool = false,
    _padding: @Type(.{ .int = .{ .signedness = .unsigned, .bits = @bitSizeOf(c_uint) - 1 } }) = 0,
};

/// Flags for `allocate_data_for_thread`.
pub const AllocateDataFlags = packed struct(c_uint) {
    /// Allocate thread-local collision detection caches. Required for any thread that calls collision functions.
    collision_data: bool = false,
    _padding: @Type(.{ .int = .{ .signedness = .unsigned, .bits = @bitSizeOf(c_uint) - 1 } }) = 0,
};

/// Simple initialization with default settings. Prefer `init_ode2` for multithreaded use.
pub fn init_ode() void {
    c.dInitODE();
}

/// Initialize the library with explicit flags. Must be called before any other ODE function.
/// Returns false if initialization fails.
pub fn init_ode2(flags: InitFlags) bool {
    return c.dInitODE2(@bitCast(flags)) != 0;
}

/// Allocate thread-local data for the calling thread. Each thread that uses ODE
/// (especially collision) must call this after `init_ode2`. Returns false on failure.
pub fn allocate_data_for_thread(flags: AllocateDataFlags) bool {
    return c.dAllocateODEDataForThread(@bitCast(flags)) != 0;
}

/// Free thread-local data for the calling thread. Only needed when `InitFlags.manual_thread_cleanup` was set.
pub fn cleanup_all_data_for_thread() void {
    c.dCleanupODEAllDataForThread();
}

/// Shut down the library and release all global resources. No ODE calls are valid after this.
pub fn close_ode() void {
    c.dCloseODE();
}

/// Returns a string describing the build configuration (precision, trimesh backend, etc.).
pub fn get_configuration() [*:0]const u8 {
    return c.dGetConfiguration();
}

/// Check whether a specific feature token (e.g. "ODE_double_precision") is present in the build.
pub fn check_configuration(token: [*:0]const u8) bool {
    return c.dCheckConfiguration(token) != 0;
}
