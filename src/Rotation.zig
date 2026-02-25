//! Utility functions for constructing 3x3 rotation matrices and quaternions from
//! axes, angles, Euler angles, and conversions between representations.

const c = @import("c.zig").c;
const Real = c.dReal;

/// Returns a 3x3 identity rotation matrix (no rotation).
pub fn matrix_set_identity() [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dRSetIdentity(&r);
    return r;
}

/// Build a rotation matrix from an axis and angle (radians). The axis does not need to be normalized.
pub fn matrix_from_axis_and_angle(axis: [3]Real, angle: Real) [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dRFromAxisAndAngle(&r, axis[0], axis[1], axis[2], angle);
    return r;
}

/// Build a rotation matrix from Euler angles (phi, theta, psi) in radians.
/// Rotation order is Z-X-Z (aerospace convention).
pub fn matrix_from_euler_angles(phi: Real, theta: Real, psi: Real) [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dRFromEulerAngles(&r, phi, theta, psi);
    return r;
}

/// Build a rotation matrix from two axes. The first axis becomes the X direction,
/// the second is projected onto the YZ plane to determine Y and Z.
pub fn matrix_from_2_axes(a: [3]Real, b_vec: [3]Real) [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dRFrom2Axes(&r, a[0], a[1], a[2], b_vec[0], b_vec[1], b_vec[2]);
    return r;
}

/// Build a rotation matrix that aligns the local Z axis with the given world-space direction.
pub fn matrix_from_z_axis(axis: [3]Real) [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dRFromZAxis(&r, axis[0], axis[1], axis[2]);
    return r;
}

/// Returns the identity quaternion (no rotation): (w=1, x=0, y=0, z=0).
pub fn quaternion_set_identity() [4]Real {
    var q: c.dQuaternion = undefined;
    c.dQSetIdentity(&q);
    return q;
}

/// Build a quaternion from an axis and angle (radians). The axis does not need to be normalized.
pub fn quaternion_from_axis_and_angle(axis: [3]Real, angle: Real) [4]Real {
    var q: c.dQuaternion = undefined;
    c.dQFromAxisAndAngle(&q, axis[0], axis[1], axis[2], angle);
    return q;
}

/// Quaternion multiplication: result = b * q_c. Both operands are treated as non-inverted.
pub fn quaternion_multiply0(b: [4]Real, q_c: [4]Real) [4]Real {
    var qa: c.dQuaternion = undefined;
    c.dQMultiply0(&qa, &b, &q_c);
    return qa;
}

/// Quaternion multiplication: result = b_inverse * q_c (first operand is conjugated).
pub fn quaternion_multiply1(b: [4]Real, q_c: [4]Real) [4]Real {
    var qa: c.dQuaternion = undefined;
    c.dQMultiply1(&qa, &b, &q_c);
    return qa;
}

/// Quaternion multiplication: result = b * q_c_inverse (second operand is conjugated).
pub fn quaternion_multiply2(b: [4]Real, q_c: [4]Real) [4]Real {
    var qa: c.dQuaternion = undefined;
    c.dQMultiply2(&qa, &b, &q_c);
    return qa;
}

/// Quaternion multiplication: result = b_inverse * q_c_inverse (both operands are conjugated).
pub fn quaternion_multiply3(b: [4]Real, q_c: [4]Real) [4]Real {
    var qa: c.dQuaternion = undefined;
    c.dQMultiply3(&qa, &b, &q_c);
    return qa;
}

/// Convert a quaternion to a 3x3 rotation matrix.
pub fn matrix_from_quaternion(q: [4]Real) [12]Real {
    var r: c.dMatrix3 = undefined;
    c.dRfromQ(&r, &q);
    return r;
}

/// Convert a 3x3 rotation matrix to a quaternion.
pub fn quaternion_from_matrix(r: *const [12]Real) [4]Real {
    var q: c.dQuaternion = undefined;
    c.dQfromR(&q, r);
    return q;
}

/// Compute the quaternion time-derivative from an angular velocity vector and current orientation.
/// Useful for integrating angular velocity into quaternion form: q' = 0.5 * omega * q.
pub fn dq_from_w(w: [3]Real, q: [4]Real) [4]Real {
    var dq: [4]Real = undefined;
    c.dDQfromW(&dq, &.{ w[0], w[1], w[2], 0 }, &q);
    return dq;
}
