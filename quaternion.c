/*
 * quaternion.c
 * A complete implementation of quaternion math in C,
 * demonstrating: creation, conjugate, norm, inverse, multiplication, rotation
 * encoding, point rotation, and SLERP.
 * $ clang quaternion.c -o quaternion -lm
 */

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Quaternion and Vector Structures */
typedef struct {
  double w, x, y, z;
} Quaternion;

typedef struct {
  double x, y, z;
} Vec3;

/* Basic Constructors */
Quaternion new_quat(double w, double x, double y, double z) {
  return (Quaternion){w, x, y, z};
}

/* Encode a rotation: axis must be a unit vector, angle in radians */
Quaternion quat_from_axis_angle(Vec3 axis, double angle) {
  double half = angle / 2.0;
  double s = sin(half);
  return (Quaternion){
      .w = cos(half), .x = axis.x * s, .y = axis.y * s, .z = axis.z * s};
}

/* Identity quaternion — represents "no rotation" */
Quaternion quat_identity(void) { return (Quaternion){1.0, 0.0, 0.0, 0.0}; }

/* Core Operations */
double quat_norm(Quaternion q) {
  return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion quat_normalize(Quaternion q) {
  double n = quat_norm(q);
  return (Quaternion){q.w / n, q.x / n, q.y / n, q.z / n};
}

/* Conjugate: reverses the rotation */
Quaternion quat_conjugate(Quaternion q) {
  return (Quaternion){q.w, -q.x, -q.y, -q.z};
}

/* Inverse: q* / |q|^2  (for unit quaternions, inverse == conjugate) */
Quaternion quat_inverse(Quaternion q) {
  double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  Quaternion c = quat_conjugate(q);
  return (Quaternion){c.w / n2, c.x / n2, c.y / n2, c.z / n2};
}

/* Hamilton product: apply q1 THEN q2  →  q2 * q1 */
Quaternion quat_mul(Quaternion a, Quaternion b) {
  return (Quaternion){.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                      .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

/* Dot product (used by SLERP) */
double quat_dot(Quaternion a, Quaternion b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/* Sandwich product: p' = q * p * q^-1 */
Vec3 quat_rotate_vec3(Quaternion q, Vec3 v) {
  /* Treat v as a pure quaternion (w=0) */
  Quaternion p = {0.0, v.x, v.y, v.z};
  Quaternion qi = quat_inverse(q);
  Quaternion res = quat_mul(quat_mul(q, p), qi);
  return (Vec3){res.x, res.y, res.z};
}

/* SLERP
 * Spherical Linear Interpolation.
 * t=0 → returns q1, t=1 → returns q2.
 * Traces the shortest arc on the 4D unit sphere.
 */
Quaternion quat_slerp(Quaternion q1, Quaternion q2, double t) {
  double dot = quat_dot(q1, q2);

  /* Ensure shortest path: negate q2 if dot < 0 */
  if (dot < 0.0) {
    q2 = (Quaternion){-q2.w, -q2.x, -q2.y, -q2.z};
    dot = -dot;
  }

  /* If quaternions are very close, fall back to linear interpolation */
  if (dot > 0.9995) {
    Quaternion result = {q1.w + t * (q2.w - q1.w), q1.x + t * (q2.x - q1.x),
                         q1.y + t * (q2.y - q1.y), q1.z + t * (q2.z - q1.z)};
    return quat_normalize(result);
  }

  double theta_0 = acos(dot); /* angle between q1 and q2 */
  double theta = theta_0 * t; /* angle for this step      */
  double s1 = cos(theta) - dot * sin(theta) / sin(theta_0);
  double s2 = sin(theta) / sin(theta_0);

  return (Quaternion){s1 * q1.w + s2 * q2.w, s1 * q1.x + s2 * q2.x,
                      s1 * q1.y + s2 * q2.y, s1 * q1.z + s2 * q2.z};
}

/* Convert to Rotation Matrix (column-major, 3×3) */
void quat_to_matrix(Quaternion q, double m[3][3]) {
  double w = q.w, x = q.x, y = q.y, z = q.z;
  m[0][0] = 1 - 2 * (y * y + z * z);
  m[0][1] = 2 * (x * y - w * z);
  m[0][2] = 2 * (x * z + w * y);
  m[1][0] = 2 * (x * y + w * z);
  m[1][1] = 1 - 2 * (x * x + z * z);
  m[1][2] = 2 * (y * z - w * x);
  m[2][0] = 2 * (x * z - w * y);
  m[2][1] = 2 * (y * z + w * x);
  m[2][2] = 1 - 2 * (x * x + y * y);
}

/* Helpers */
void print_quat(const char *label, Quaternion q) {
  printf("%-22s  w=%7.4f  x=%7.4f  y=%7.4f  z=%7.4f\n", label, q.w, q.x, q.y,
         q.z);
}

void print_vec3(const char *label, Vec3 v) {
  printf("%-22s  x=%7.4f  y=%7.4f  z=%7.4f\n", label, v.x, v.y, v.z);
}

void separator(const char *title) {
  printf("\n━━━━━━━━━━━━━━━ %s ━━━━━━━━━━━━━━━\n\n", title);
}

void print_title(const char *title) {
  size_t len = strlen(title);
  size_t total = len + 4; // 2 spaces + 2 border chars padding

  char pad[total + 1];
  memset(pad, '=', total);
  pad[total] = '\0';

  printf("%s\n", pad);
  printf("== %s ==\n", title);
  printf("%s\n", pad);
}

int main(void) {
  print_title("QUATERNION DEMO  —  quaternion.c");

  /* 1. Basic construction */
  separator("1. CONSTRUCTION & NORM");

  Quaternion q = new_quat(1.0, 2.0, 3.0, 4.0);
  print_quat("Raw quaternion q:", q);
  printf("  norm(q)          = %.4f  (should be %.4f for unit)\n", quat_norm(q),
         1.0);

  Quaternion qu = quat_normalize(q);
  print_quat("Normalized q:", qu);
  printf("  norm(q_unit)     = %.4f\n", quat_norm(qu));

  /* 2. Conjugate & Inverse */
  separator("2. CONJUGATE & INVERSE");

  Quaternion conj = quat_conjugate(qu);
  Quaternion inv = quat_inverse(qu);
  print_quat("conjugate(q):", conj);
  print_quat("inverse(q)  :", inv);

  Quaternion should_be_identity = quat_mul(qu, inv);
  print_quat("q * q^-1    :", should_be_identity);
  printf("  → should be identity (1, 0, 0, 0)\n");

  /* 3. Encoding a rotation */
  separator("3. ROTATION ENCODING");

  /* 90° rotation around the Z axis */
  Vec3 z_axis = {0.0, 0.0, 1.0};
  double angle = M_PI / 2.0; /* 90 degrees */
  Quaternion rot90z = quat_from_axis_angle(z_axis, angle);
  print_quat("90° around Z axis:", rot90z);
  printf("  Expected: w=cos(45°)=%.4f, z=sin(45°)=%.4f\n", cos(angle / 2),
         sin(angle / 2));

  /* 4. Rotating a point */
  separator("4. ROTATING A 3D POINT");

  Vec3 point = {1.0, 0.0, 0.0}; /* unit X vector */
  Vec3 rotated = quat_rotate_vec3(rot90z, point);
  print_vec3("Original point (1,0,0):", point);
  print_vec3("After 90° rotation around Z:", rotated);
  printf("  Expected: (0, 1, 0)  — X axis maps to Y axis\n");

  /* 5. Composing rotations */
  separator("5. COMPOSING ROTATIONS");

  /* Two consecutive 90° rotations around Z = 180° total */
  Quaternion rot180z = quat_mul(rot90z, rot90z);
  Vec3 rotated2 = quat_rotate_vec3(rot180z, point);
  print_vec3("After 2×90° around Z:", rotated2);
  printf("  Expected: (-1, 0, 0) — 180° flips X to -X\n");

  /* 90° around X, then 90° around Y (order matters!) */
  Vec3 x_axis = {1.0, 0.0, 0.0};
  Vec3 y_axis = {0.0, 1.0, 0.0};
  Quaternion rot90x = quat_from_axis_angle(x_axis, M_PI / 2);
  Quaternion rot90y = quat_from_axis_angle(y_axis, M_PI / 2);

  /* q_xy means: first X rotation, then Y rotation → q_y * q_x */
  Quaternion rot_xy = quat_mul(rot90y, rot90x);
  Quaternion rot_yx = quat_mul(rot90x, rot90y);

  Vec3 p = {1.0, 0.0, 0.0};
  print_vec3("X then Y rotation of (1,0,0):", quat_rotate_vec3(rot_xy, p));
  print_vec3("Y then X rotation of (1,0,0):", quat_rotate_vec3(rot_yx, p));
  printf("  → Different results prove non-commutativity!\n");

  /* 6. SLERP */
  separator("6. SLERP — SMOOTH INTERPOLATION");

  Quaternion start = quat_identity();
  Quaternion end = quat_from_axis_angle(z_axis, M_PI); /* 180° around Z */

  printf("  Interpolating from identity to 180° around Z:\n");
  printf("  %-6s  %-10s  %-10s  %-10s  %-10s\n", "t", "w", "x", "y", "z");
  printf("  %-6s  %-10s  %-10s  %-10s  %-10s\n", "──────", "──────────",
         "──────────", "──────────", "──────────");
  for (int i = 0; i <= 5; i++) {
    double t = i / 5.0;
    Quaternion s = quat_slerp(start, end, t);
    printf("  %-6.2f  %-10.4f  %-10.4f  %-10.4f  %-10.4f\n", t, s.w, s.x, s.y,
           s.z);
  }
  printf("  → Smooth interpolation, constant angular velocity\n");

  /* 7. Rotation matrix */
  separator("7. CONVERTING TO ROTATION MATRIX");

  double mat[3][3];
  quat_to_matrix(rot90z, mat);
  printf("  Rotation matrix for 90° around Z:\n\n");
  for (int r = 0; r < 3; r++) {
    printf("    [ %7.4f  %7.4f  %7.4f ]\n", mat[r][0], mat[r][1], mat[r][2]);
  }
  printf("\n  Expected:\n");
  printf("    [  0.0000   -1.0000    0.0000 ]\n");
  printf("    [  1.0000    0.0000    0.0000 ]\n");
  printf("    [  0.0000    0.0000    1.0000 ]\n");

  return 0;
}
