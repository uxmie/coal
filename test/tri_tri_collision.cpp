#include "coal/data_types.h"
#define BOOST_TEST_MODULE COAL_BOX_BOX_COLLISION
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Geometry>
#include "coal/narrowphase/narrowphase.h"
#include "coal/shape/geometric_shapes.h"
#include "coal/internal/tools.h"

#include "utility.h"

using coal::CoalScalar;
using coal::collide;
using coal::CollisionRequest;
using coal::CollisionResult;
using coal::ComputeCollision;
using coal::Transform3s;
using coal::TriangleP;
using coal::Vec3s;

BOOST_AUTO_TEST_CASE(tri_tri_collision) {
  // Define triangles
  TriangleP shape1(Vec3s(0, 0, 0), Vec3s(1, 0, 0), Vec3s(0, 1, 0));
  TriangleP shape2(Vec3s(1, 1, 0), Vec3s(0.5, 0.5, 1), Vec3s(0.5, 0.5, -1));

  // Define transforms
  Transform3s T1 = Transform3s::Identity();
  Transform3s T2 = Transform3s::Identity();

  // Compute collision
  CollisionRequest req;
  req.enable_cached_gjk_guess = true;
  req.distance_upper_bound = 1e-6;
  CollisionResult res;
  ComputeCollision collide_functor(&shape1, &shape2);

  T1.setTranslation(Vec3s(0.01, 0.01, 0));
  T1.setQuatRotation(Eigen::Quaternion<CoalScalar>(
      Eigen::AngleAxis<CoalScalar>(0.01, Vec3s(0, 1, 0))));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == true);
  std::cerr << "collision point: " << res.getContact(0).pos.transpose()
            << "\ncollision normal: " << res.getContact(0).normal.transpose()
            << std::endl;
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  T1.setTranslation(Vec3s(-0.01, -0.01, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  T1.setTranslation(Vec3s(0.001, 0.002, -0.2));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == true);
  std::cerr << "collision point: " << res.getContact(0).pos.transpose()
            << "\ncollision normal: " << res.getContact(0).normal.transpose()
            << std::endl;
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);

  // Triangles in plane
  shape2.a = Vec3s(1, 0, 0);
  shape2.b = Vec3s(1, 1, 0);
  shape2.c = Vec3s(0, 1, 0);
  T1.setTranslation(Vec3s::Zero());
  T2.setTranslation(Vec3s(0.001, 0.001, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == false);
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == false);

  T2.setTranslation(Vec3s(-0.001, -0.001, 0));
  res.clear();
  BOOST_CHECK(collide(&shape1, T1, &shape2, T2, req, res) == true);
  std::cerr << "collision point: " << res.getContact(0).pos.transpose()
            << "\ncollision normal: " << res.getContact(0).normal.transpose()
            << std::endl;
  res.clear();
  BOOST_CHECK(collide_functor(T1, T2, req, res) == true);
}
