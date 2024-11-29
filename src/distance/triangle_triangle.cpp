/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THICoalScalar SOFTWARE ICoalScalar PROVIDED BY THE COPYRIGHT
 * HOLDERCoalScalar AND CONTRIBUTORS "ACoalScalar IS" AND ANY EXPRESCoalScalar
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIECoalScalar OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORCoalScalar BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGECoalScalar (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODCoalScalar OR SERVICES;
 *  LOSCoalScalar OF USE, DATA, OR PROFITS; OR BUSINESCoalScalar INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THICoalScalar SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Louis Montaut */

#include <cstdint>
#include "coal/data_types.h"
#include "coal/shape/geometric_shapes.h"

#include "coal/internal/shape_shape_func.h"
#include "../narrowphase/details.h"

namespace coal {

namespace internal {
void segPoints(const Vec3s& P, const Vec3s& A, const Vec3s& Q, const Vec3s& B,
               Vec3s& VEC, Vec3s& X, Vec3s& Y) {
  Vec3s T;
  CoalScalar A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
  Vec3s TMP;

  T = Q - P;
  A_dot_A = A.dot(A);
  B_dot_B = B.dot(B);
  A_dot_B = A.dot(B);
  A_dot_T = A.dot(T);
  B_dot_T = B.dot(T);

  // t parameterizes ray P,A
  // u parameterizes ray Q,B

  CoalScalar t, u;

  // compute t for the closest point on ray P,A to
  // ray Q,B

  CoalScalar denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;

  t = (A_dot_T * B_dot_B - B_dot_T * A_dot_B) / denom;

  // clamp result so t is on the segment P,A

  if ((t < 0) || std::isnan(t))
    t = 0;
  else if (t > 1)
    t = 1;

  // find u for point on ray Q,B closest to point at t

  u = (t * A_dot_B - B_dot_T) / B_dot_B;

  // if u is on segment Q,B, t and u correspond to
  // closest points, otherwise, clamp u, recompute and
  // clamp t

  if ((u <= 0) || std::isnan(u)) {
    Y = Q;

    t = A_dot_T / A_dot_A;

    if ((t <= 0) || std::isnan(t)) {
      X = P;
      VEC = Q - P;
    } else if (t >= 1) {
      X = P + A;
      VEC = Q - X;
    } else {
      X = P + A * t;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  } else if (u >= 1) {
    Y = Q + B;

    t = (A_dot_B + A_dot_T) / A_dot_A;

    if ((t <= 0) || std::isnan(t)) {
      X = P;
      VEC = Y - P;
    } else if (t >= 1) {
      X = P + A;
      VEC = Y - X;
    } else {
      X = P + A * t;
      T = Y - P;
      TMP = T.cross(A);
      VEC = A.cross(TMP);
    }
  } else {
    Y = Q + B * u;

    if ((t <= 0) || std::isnan(t)) {
      X = P;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    } else if (t >= 1) {
      X = P + A;
      T = Q - X;
      TMP = T.cross(B);
      VEC = B.cross(TMP);
    } else {
      X = P + A * t;
      VEC = A.cross(B);
      if (VEC.dot(T) < 0) {
        VEC = VEC * (-1);
      }
    }
  }
}

CoalScalar triDistance(const Vec3s T1[3], const Vec3s T2[3], Vec3s& P,
                       Vec3s& Q) {
  // Compute vectors along the 6 sides

  Vec3s Sv[3];
  Vec3s Tv[3];
  Vec3s VEC;

  Sv[0] = T1[1] - T1[0];
  Sv[1] = T1[2] - T1[1];
  Sv[2] = T1[0] - T1[2];

  Tv[0] = T2[1] - T2[0];
  Tv[1] = T2[2] - T2[1];
  Tv[2] = T2[0] - T2[2];

  // For each edge pair, the vector connecting the closest points
  // of the edges defines a slab (parallel planes at head and tail
  // enclose the slab). If we can show that the off-edge vertex of
  // each triangle is outside of the slab, then the closest points
  // of the edges are the closest points for the triangles.
  // Even if these tests fail, it may be helpful to know the closest
  // points found, and whether the triangles were shown disjoint

  Vec3s V;
  Vec3s Z;
  Vec3s minP = Vec3s::Zero();
  Vec3s minQ = Vec3s::Zero();
  CoalScalar mindd;
  int shown_disjoint = 0;

  mindd = (T1[0] - T2[0]).squaredNorm() + 1;  // Set first minimum safely high

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      // Find closest points on edges i & j, plus the
      // vector (and distance squared) between these points
      segPoints(T1[i], Sv[i], T2[j], Tv[j], VEC, P, Q);

      V = Q - P;
      CoalScalar dd = V.dot(V);

      // Verify this closest point pair only if the distance
      // squared is less than the minimum found thus far.

      if (dd <= mindd) {
        minP = P;
        minQ = Q;
        mindd = dd;

        Z = T1[(i + 2) % 3] - P;
        CoalScalar a = Z.dot(VEC);
        Z = T2[(j + 2) % 3] - Q;
        CoalScalar b = Z.dot(VEC);

        if ((a <= 0) && (b >= 0)) return sqrt(dd);

        CoalScalar p = V.dot(VEC);

        if (a < 0) a = 0;
        if (b > 0) b = 0;
        if ((p - a + b) > 0) shown_disjoint = 1;
      }
    }
  }

  // No edge pairs contained the closest points.
  // either:
  // 1. one of the closest points is a vertex, and the
  //    other point is interior to a face.
  // 2. the triangles are overlapping.
  // 3. an edge of one triangle is parallel to the other's face. If
  //    cases 1 and 2 are not true, then the closest points from the 9
  //    edge pairs checks above can be taken as closest points for the
  //    triangles.
  // 4. possibly, the triangles were degenerate.  When the
  //    triangle points are nearly colinear or coincident, one
  //    of above tests might fail even though the edges tested
  //    contain the closest points.

  // First check for case 1

  Vec3s Sn;
  CoalScalar Snl;

  Sn = Sv[0].cross(Sv[1]);  // Compute normal to T1 triangle
  Snl = Sn.dot(Sn);         // Compute square of length of normal

  // If cross product is long enough,

  if (Snl > 1e-15) {
    // Get projection lengths of T2 points

    Vec3s Tp;

    V = T1[0] - T2[0];
    Tp[0] = V.dot(Sn);

    V = T1[0] - T2[1];
    Tp[1] = V.dot(Sn);

    V = T1[0] - T2[2];
    Tp[2] = V.dot(Sn);

    // If Sn is a separating direction,
    // find point with smallest projection

    int point = -1;
    if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0)) {
      if (Tp[0] < Tp[1])
        point = 0;
      else
        point = 1;
      if (Tp[2] < Tp[point]) point = 2;
    } else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0)) {
      if (Tp[0] > Tp[1])
        point = 0;
      else
        point = 1;
      if (Tp[2] > Tp[point]) point = 2;
    }

    // If Sn is a separating direction,

    if (point >= 0) {
      shown_disjoint = 1;

      // Test whether the point found, when projected onto the
      // other triangle, lies within the face.

      V = T2[point] - T1[0];
      Z = Sn.cross(Sv[0]);
      if (V.dot(Z) > 0) {
        V = T2[point] - T1[1];
        Z = Sn.cross(Sv[1]);
        if (V.dot(Z) > 0) {
          V = T2[point] - T1[2];
          Z = Sn.cross(Sv[2]);
          if (V.dot(Z) > 0) {
            // T[point] passed the test - it's a closest point for
            // the T2 triangle; the other point is on the face of T1
            P = T2[point] + Sn * (Tp[point] / Snl);
            Q = T2[point];
            return (P - Q).norm();
          }
        }
      }
    }
  }

  Vec3s Tn;
  CoalScalar Tnl;

  Tn = Tv[0].cross(Tv[1]);
  Tnl = Tn.dot(Tn);

  if (Tnl > 1e-15) {
    Vec3s Sp;

    V = T2[0] - T1[0];
    Sp[0] = V.dot(Tn);

    V = T2[0] - T1[1];
    Sp[1] = V.dot(Tn);

    V = T2[0] - T1[2];
    Sp[2] = V.dot(Tn);

    int point = -1;
    if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0)) {
      if (Sp[0] < Sp[1])
        point = 0;
      else
        point = 1;
      if (Sp[2] < Sp[point]) point = 2;
    } else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0)) {
      if (Sp[0] > Sp[1])
        point = 0;
      else
        point = 1;
      if (Sp[2] > Sp[point]) point = 2;
    }

    if (point >= 0) {
      shown_disjoint = 1;

      V = T1[point] - T2[0];
      Z = Tn.cross(Tv[0]);
      if (V.dot(Z) > 0) {
        V = T1[point] - T2[1];
        Z = Tn.cross(Tv[1]);
        if (V.dot(Z) > 0) {
          V = T1[point] - T2[2];
          Z = Tn.cross(Tv[2]);
          if (V.dot(Z) > 0) {
            P = T1[point];
            Q = T1[point] + Tn * (Sp[point] / Tnl);
            return (P - Q).norm();
          }
        }
      }
    }
  }

  // Case 1 can't be shown.
  // If one of these tests showed the triangles disjoint,
  // we assume case 3 or 4, otherwise we conclude case 2,
  // that the triangles overlap.

  if (shown_disjoint) {
    P = minP;
    Q = minQ;
    return sqrt(mindd);
  } else
    return 0;
}

template <>
CoalScalar ShapeShapeDistance<TriangleP, TriangleP>(
    const CollisionGeometry* o1, const Transform3s& tf1,
    const CollisionGeometry* o2, const Transform3s& tf2,
    const GJKSolver* solver, const bool, Vec3s& p1, Vec3s& p2, Vec3s& normal) {
  // Transform the triangles in world frame
  const TriangleP& s1 = static_cast<const TriangleP&>(*o1);
  TriangleP t1(tf1.transform(s1.a), tf1.transform(s1.b), tf1.transform(s1.c));

  const TriangleP& s2 = static_cast<const TriangleP&>(*o2);
  TriangleP t2(tf2.transform(s2.a), tf2.transform(s2.b), tf2.transform(s2.c));

  // Vec3s T1s[] = {t1.a, t1.b, t1.c};
  // Vec3s T2s[] = {t2.a, t2.b, t2.c};
  // CoalScalar d = triDistance(T1s, T2s, p1, p2);
  // normal = (p2 - p1).normalized();
  // return d;

  // // Reset GJK algorithm
  // //   We don't need to take into account swept-sphere radius in GJK
  // iterations;
  // //   the result will be corrected after GJK terminates.
  // solver->minkowski_difference
  //     .set<::coal::details::SupportOptions::NoSweptSphere>(&t1, &t2);
  // solver->gjk.reset(solver->gjk_max_iterations, solver->gjk_tolerance);
  //
  // // Get GJK initial guess
  // Vec3s guess;
  // if (solver->gjk_initial_guess == GJKInitialGuess::CachedGuess ||
  //     solver->enable_cached_guess) {
  //   guess = solver->cached_guess;
  // } else {
  //   guess = (t1.a + t1.b + t1.c - t2.a - t2.b - t2.c) / 3;
  // }
  // support_func_guess_t support_hint;
  // solver->epa.status =
  //     details::EPA::DidNotRun;  // EPA is never called in this function
  //
  // details::GJK::Status gjk_status =
  //     solver->gjk.evaluate(solver->minkowski_difference, guess,
  //     support_hint);
  //
  // solver->cached_guess = solver->gjk.getGuessFromSimplex();
  // solver->support_func_cached_guess = solver->gjk.support_hint;
  //
  // // Retrieve witness points and normal
  // solver->gjk.getWitnessPointsAndNormal(solver->minkowski_difference, p1, p2,
  //                                       normal);
  // CoalScalar distance = solver->gjk.distance;
  //
  // if (gjk_status == details::GJK::Collision) {
  //   CoalScalar penetrationDepth =
  //       details::computePenetration(t1.a, t1.b, t1.c, t2.a, t2.b, t2.c,
  //       normal);
  //   distance = -penetrationDepth;
  // } else {
  //   // No collision
  //   // TODO On degenerated case, the closest point may be wrong
  //   // (i.e. an object face normal is colinear to gjk.ray
  //   // assert (dist == (w0 - w1).norm());
  //   assert(solver->gjk.ray.norm() > solver->gjk.getTolerance());
  // }
  // // assert(false && "should not reach this point");
  // // return false;
  //
  // return distance;

  Vec3s* T1s[] = {&t1.a, &t1.b, &t1.c};
  Vec3s* T2s[] = {&t2.a, &t2.b, &t2.c};
  auto detTest = [](const Vec3s& a, const Vec3s& b, const Vec3s& c,
                    const Vec3s& d) -> CoalScalar {
    return (a - d).cross(b - d).dot(a - c);
  };
  auto step = [](CoalScalar val) -> int_fast8_t {
    return val < 0 ? -1 : (val > 0 ? 1 : 0);
  };
  auto permutateTriangles = [step](Vec3s** ta, Vec3s** tb) {
    std::array<int_fast8_t, 3> pointSide;
	Vec3s n = (*tb[1] - *tb[0]).cross(*tb[2] - *tb[0]);
    pointSide[0] = step(n.dot(*ta[0] - *tb[0]));
    pointSide[1] = step(n.dot(*ta[1] - *tb[0]));
    pointSide[2] = step(n.dot(*ta[2] - *tb[0]));

    if ((pointSide[0] > 0 && pointSide[1] > 0 && pointSide[2] > 0) ||
        (pointSide[0] < 0 && pointSide[1] < 0 && pointSide[2] < 0)) {
      return false;
    }

    if (pointSide[1] * pointSide[2] >= 0) {
      if (pointSide[0] < 0) {
        std::swap(tb[1], tb[2]);
      }
    } else if (pointSide[0] * pointSide[2] >= 0) {
      std::swap(ta[0], ta[1]);
      std::swap(ta[2], ta[1]);
      if (pointSide[1] < 0) {
        std::swap(tb[1], tb[2]);
      }
    } else if (pointSide[0] * pointSide[1] >= 0) {
      std::swap(ta[0], ta[2]);
      std::swap(ta[2], ta[1]);
      if (pointSide[2] < 0) {
        std::swap(tb[1], tb[2]);
      }
    }
	return true;
  };
  if ( !permutateTriangles(T1s, T2s) || !permutateTriangles(T2s, T1s) ) {
	  // Vec3s T1[] = {t1.a, t1.b, t1.c};
	  // Vec3s T2[] = {t2.a, t2.b, t2.c};
	  // CoalScalar d = triDistance(T1, T2, p1, p2);
	  return 1;
  }
  if (detTest(*T1s[0], *T1s[1], *T2s[0], *T2s[1]) <= 0 &&
      detTest(*T1s[0], *T1s[2], *T2s[2], *T2s[0]) <= 0) {
    return 0;
  } else {
    return 1;
  }
}
}  // namespace internal

}  // namespace coal
