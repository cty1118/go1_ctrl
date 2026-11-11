// Owner: Member 2. FK, analytic IK, and Jacobian unit tests.
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <gtest/gtest.h>

#include "go1_ctrl/kinematics.h"

namespace {

std::array<go1::LegGeometry, go1::kNumLegs> makeGeometry() {
  std::array<go1::LegGeometry, go1::kNumLegs> g;
  const double x = 0.1881;
  const double y = 0.04675;
  const double a2 = 0.213;
  const double a3 = 0.213;

  g[go1::FR].hipPosition_B = go1::Vec3( x, -y, 0.0);
  g[go1::FL].hipPosition_B = go1::Vec3( x,  y, 0.0);
  g[go1::RR].hipPosition_B = go1::Vec3(-x, -y, 0.0);
  g[go1::RL].hipPosition_B = go1::Vec3(-x,  y, 0.0);

  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    g[leg].lateralOffset = (leg == go1::FL || leg == go1::RL) ? 0.080 : -0.080;
    g[leg].thighLength = a2;
    g[leg].calfLength = a3;
  }
  return g;
}

// Independent reference: multiply the four DH/body transforms from the slides.
// This checks the closed-form position equations without reusing them.
go1::Vec3 footFromDH(const go1::LegGeometry& g, const go1::Vec3& q) {
  const double c1 = std::cos(q[go1::HIP]);
  const double s1 = std::sin(q[go1::HIP]);
  const double c2 = std::cos(q[go1::THIGH]);
  const double s2 = std::sin(q[go1::THIGH]);
  const double c3 = std::cos(q[go1::CALF]);
  const double s3 = std::sin(q[go1::CALF]);

  Eigen::Matrix4d T_B0 = Eigen::Matrix4d::Identity();
  T_B0.block<3, 3>(0, 0) << 0, 0, 1,
                              0, 1, 0,
                             -1, 0, 0;
  T_B0.block<3, 1>(0, 3) = g.hipPosition_B;

  Eigen::Matrix4d A1 = Eigen::Matrix4d::Identity();
  A1 << c1, 0, -s1, 0,
        s1, 0,  c1, 0,
         0,-1,   0, 0,
         0, 0,   0, 1;

  Eigen::Matrix4d A2 = Eigen::Matrix4d::Identity();
  A2 << c2, -s2, 0, g.thighLength * c2,
        s2,  c2, 0, g.thighLength * s2,
         0,   0, 1, g.lateralOffset,
         0,   0, 0, 1;

  Eigen::Matrix4d A3 = Eigen::Matrix4d::Identity();
  A3 << c3, -s3, 0, g.calfLength * c3,
        s3,  c3, 0, g.calfLength * s3,
         0,   0, 1, 0,
         0,   0, 0, 1;

  const Eigen::Matrix4d T = T_B0 * A1 * A2 * A3;
  return T.block<3, 1>(0, 3);
}

}  // namespace

TEST(KinematicsFK, NeutralAnglesForAllFourLegs) {
  const auto geometry = makeGeometry();
  const go1::Kinematics kin(geometry);
  const go1::Vec3 q = go1::Vec3::Zero();
  // q=0 is a mathematical test; it is not a safe hardware standing pose.
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    const go1::Vec3 p = kin.forwardKinematics(leg, q);
    EXPECT_NEAR(p.x(), geometry[leg].hipPosition_B.x(), 1e-12);
    EXPECT_NEAR(p.y(), geometry[leg].hipPosition_B.y() + geometry[leg].lateralOffset, 1e-12);
    EXPECT_NEAR(p.z(), -0.426, 1e-12);
  }
}

TEST(KinematicsFK, MatchesHomogeneousTransformFromDHDerivation) {
  const auto geometry = makeGeometry();
  const go1::Kinematics kin(geometry);
  const std::array<go1::Vec3, 4> angles = {{
      go1::Vec3(0.0, 0.67, -1.30),
      go1::Vec3(0.12, 0.70, -1.20),
      go1::Vec3(-0.24, 0.95, -1.60),
      go1::Vec3(0.38, 0.45, -1.05)
  }};
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    for (const go1::Vec3& q : angles) {
      const go1::Vec3 p = kin.forwardKinematics(leg, q);
      const go1::Vec3 expected = footFromDH(geometry[leg], q);
      for (int axis = 0; axis < 3; ++axis)
        EXPECT_NEAR(p[axis], expected[axis], 1e-12);
    }
  }
}

TEST(KinematicsFK, ReadsJointAnglesUsingMemberOneTypes) {
  const go1::Kinematics kin(makeGeometry());
  go1::RobotState s;
  const go1::Vec3 q(0.10, 0.67, -1.30);
  s.q.segment<3>(go1::jointIndex(go1::RL, go1::HIP)) = q;
  const go1::Vec3 p = kin.forwardKinematics(go1::RL, s.legQ(go1::RL));
  const go1::Vec3 p_direct = kin.forwardKinematics(go1::RL, q);
  EXPECT_NEAR((p - p_direct).norm(), 0.0, 1e-12);
}

TEST(KinematicsFK, RejectsInvalidLegIndex) {
  const go1::Kinematics kin(makeGeometry());
  EXPECT_THROW(kin.forwardKinematics(-1, go1::Vec3::Zero()), std::out_of_range);
  EXPECT_THROW(kin.forwardKinematics(4, go1::Vec3::Zero()), std::out_of_range);
}


TEST(KinematicsIK, RecoversJointAnglesForAllFourLegs) {
  const go1::Kinematics kin(makeGeometry());
  const std::array<go1::Vec3, 5> angles = {{
      go1::Vec3(0.0, 0.67, -1.30),
      go1::Vec3(0.12, 0.70, -1.20),
      go1::Vec3(-0.24, 0.95, -1.60),
      go1::Vec3(0.38, 0.45, -1.05),
      go1::Vec3(-0.25, 2.25, -1.35)
  }};
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    for (const go1::Vec3& q : angles) {
      const go1::Vec3 p = kin.forwardKinematics(leg, q);
      go1::Vec3 recovered = go1::Vec3::Zero();
      ASSERT_TRUE(kin.inverseKinematics(leg, p, q, recovered));
      EXPECT_NEAR((recovered - q).norm(), 0.0, 1e-8);
      EXPECT_NEAR((kin.forwardKinematics(leg, recovered) - p).norm(), 0.0, 1e-8);
    }
  }
}

TEST(KinematicsIK, RejectsUnreachableAndOutOfLimitTargetsWithoutChangingOutput) {
  const auto geometry = makeGeometry();
  const go1::Kinematics kin(geometry);
  const int leg = go1::FR;
  const go1::Vec3 qCurrent(0.0, 0.67, -1.30);
  const go1::Vec3 sentinel(99, 88, 77);
  go1::Vec3 output = sentinel;

  // The target projection cannot be closer to the HAA axis than |d2|.
  EXPECT_FALSE(kin.inverseKinematics(leg, geometry[leg].hipPosition_B,
                                      qCurrent, output));
  EXPECT_NEAR((output - sentinel).norm(), 0.0, 0.0);

  // Outside maximum leg reach.
  const go1::Vec3 pFar = geometry[leg].hipPosition_B +
                         go1::Vec3(1.0, geometry[leg].lateralOffset, 0.0);
  EXPECT_FALSE(kin.inverseKinematics(leg, pFar, qCurrent, output));
  EXPECT_NEAR((output - sentinel).norm(), 0.0, 0.0);

  // Geometrically reachable, but all knee branches violate q3 limits.
  const go1::Vec3 tooStraight(0.0, 0.0, -0.30);
  const go1::Vec3 pStraight = kin.forwardKinematics(leg, tooStraight);
  EXPECT_FALSE(kin.inverseKinematics(leg, pStraight, qCurrent, output));
  EXPECT_NEAR((output - sentinel).norm(), 0.0, 0.0);
}

TEST(KinematicsIK, HandlesTheUZeroHipSingularity) {
  const go1::Kinematics kin(makeGeometry());
  const double pi = 3.14159265358979323846;
  // For a2 == a3 and q3 = pi - 2*q2, U is zero.
  const go1::Vec3 q(0.0, 2.25, pi - 4.50);
  const go1::Vec3 p = kin.forwardKinematics(go1::FL, q);
  go1::Vec3 recovered = go1::Vec3::Zero();
  ASSERT_TRUE(kin.inverseKinematics(go1::FL, p, q, recovered));
  EXPECT_NEAR((kin.forwardKinematics(go1::FL, recovered) - p).norm(), 0.0, 1e-8);
}

TEST(KinematicsIK, PrefersNearbyBranchAndRemainsContinuous) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q0(0.12, 0.67, -1.30);
  go1::Vec3 qPrevious = q0;
  for (int k = 1; k <= 100; ++k) {
    const go1::Vec3 qTarget = q0 +
        go1::Vec3(0.0001 * k, -0.0002 * k, 0.0001 * k);
    const go1::Vec3 p = kin.forwardKinematics(go1::RL, qTarget);
    go1::Vec3 recovered;
    ASSERT_TRUE(kin.inverseKinematics(go1::RL, p, qPrevious, recovered));
    EXPECT_LT((recovered - qPrevious).norm(), 0.01);
    EXPECT_NEAR((recovered - qTarget).norm(), 0.0, 1e-7);
    qPrevious = recovered;
  }
}

TEST(KinematicsIK, ConfigurableJumpGuardRejectsLargeCommandChanges) {
  go1::IKOptions options;
  options.maxJointStep = go1::Vec3::Constant(0.025);
  const go1::Kinematics kin(makeGeometry(), options);

  const go1::Vec3 qTarget(0.15, 0.67, -1.30);
  const go1::Vec3 p = kin.forwardKinematics(go1::FR, qTarget);
  go1::Vec3 output = go1::Vec3::Constant(99);

  EXPECT_FALSE(kin.inverseKinematics(go1::FR, p,
                                      go1::Vec3(0.0, 0.67, -1.30), output));
  EXPECT_NEAR((output - go1::Vec3::Constant(99)).norm(), 0.0, 0.0);

  ASSERT_TRUE(kin.inverseKinematics(go1::FR, p,
                                    go1::Vec3(0.14, 0.67, -1.30), output));
  EXPECT_NEAR((output - qTarget).norm(), 0.0, 1e-8);
}

TEST(KinematicsIK, RejectsInvalidInputs) {
  const go1::Kinematics kin(makeGeometry());
  go1::Vec3 output = go1::Vec3::Zero();
  EXPECT_THROW(kin.inverseKinematics(-1, go1::Vec3::Zero(),
                                      go1::Vec3::Zero(), output), std::out_of_range);
  EXPECT_FALSE(kin.inverseKinematics(go1::FR,
              go1::Vec3(std::numeric_limits<double>::quiet_NaN(), 0, 0),
              go1::Vec3::Zero(), output));
}


// Independent end-effector orientation reference from DH rotation factors.
// Used to verify the angular velocity portion of the geometric Jacobian.
namespace {
Eigen::Matrix3d rotationFromDH(const go1::Vec3& q) {
  const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
  const double c2 = std::cos(q[1]), s2 = std::sin(q[1]);
  const double c3 = std::cos(q[2]), s3 = std::sin(q[2]);
  Eigen::Matrix3d R_B0;
  R_B0 << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  Eigen::Matrix3d R_01;
  R_01 << c1, 0, -s1, s1, 0, c1, 0, -1, 0;
  Eigen::Matrix3d R_12;
  R_12 << c2, -s2, 0, s2, c2, 0, 0, 0, 1;
  Eigen::Matrix3d R_23;
  R_23 << c3, -s3, 0, s3, c3, 0, 0, 0, 1;
  return R_B0 * R_01 * R_12 * R_23;
}
}

TEST(KinematicsJacobian, MatchesNumericalDerivativeOfFK_AllLegs) {
  const go1::Kinematics kin(makeGeometry());
  std::mt19937 rng(20261110);
  std::uniform_real_distribution<double> hip(-0.95, 0.95);
  std::uniform_real_distribution<double> thigh(-0.50, 2.70);
  std::uniform_real_distribution<double> calf(-2.65, -0.90);
  const double h = 1e-7;
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    for (int k = 0; k < 250; ++k) {
      const go1::Vec3 q(hip(rng), thigh(rng), calf(rng));
      const go1::Mat3 J = kin.jacobian(leg, q);
      for (int j = 0; j < 3; ++j) {
        go1::Vec3 qPlus = q, qMinus = q;
        qPlus[j] += h;
        qMinus[j] -= h;
        const go1::Vec3 numeric =
            (kin.forwardKinematics(leg, qPlus) -
             kin.forwardKinematics(leg, qMinus)) / (2*h);
        EXPECT_LT((numeric - J.col(j)).norm(), 3e-9);
      }
    }
  }
}

TEST(KinematicsJacobian, MatchesNumericalFootVelocity) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q(0.17, 0.69, -1.4);
  const go1::Vec3 dq(0.31, -0.21, 0.12); // [rad/s]
  const double h = 1e-7;
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    const go1::Vec3 velocity = kin.footVelocity(leg, q, dq);
    const go1::Vec3 numerical =
        (kin.forwardKinematics(leg, q + h*dq) -
         kin.forwardKinematics(leg, q - h*dq))/(2*h);
    EXPECT_LT((velocity-numerical).norm(), 3e-9);
  }
}

TEST(KinematicsJacobian, FullGeometricJacobianHasCorrectAngularVelocity) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q(0.34, 0.87, -1.53);
  const double h = 1e-7;
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    const go1::Mat63 J = kin.geometricJacobian(leg, q);
    EXPECT_LT((J.topRows<3>() - kin.jacobian(leg,q)).norm(), 1e-14);
    for (int j = 0; j < 3; ++j) {
      go1::Vec3 qp=q, qm=q;
      qp[j] += h; qm[j] -= h;
      const Eigen::Matrix3d Rdot =
          (rotationFromDH(qp)-rotationFromDH(qm))/(2*h);
      const Eigen::Matrix3d Omega = Rdot * rotationFromDH(q).transpose();
      const go1::Vec3 w(0.5*(Omega(2,1)-Omega(1,2)),
                       0.5*(Omega(0,2)-Omega(2,0)),
                       0.5*(Omega(1,0)-Omega(0,1)));
      EXPECT_LT((w - J.bottomRows<3>().col(j)).norm(), 3e-9);
    }
  }
}

TEST(KinematicsJacobian, DeterminantAndSingularConfigurations) {
  const auto geometry = makeGeometry();
  const go1::Kinematics kin(geometry);
  const go1::Vec3 q(0.17, 0.73, -1.47);
  for (int leg = 0; leg < go1::kNumLegs; ++leg) {
    const double U = geometry[leg].thighLength*std::cos(q[1]) +
                     geometry[leg].calfLength*std::cos(q[1]+q[2]);
    const double expected = geometry[leg].thighLength *
                            geometry[leg].calfLength * U * std::sin(q[2]);
    EXPECT_NEAR(kin.jacobian(leg,q).determinant(), expected, 1e-12);
    go1::Vec3 straight = q; straight[2] = 0.0;
    EXPECT_NEAR(kin.jacobian(leg, straight).determinant(), 0.0, 1e-12);
    // a2==a3 here. q3=pi-2*q2 makes U=0; q2=2.25 is valid for our IK config.
    go1::Vec3 zeroU(0.0, 2.25, 3.14159265358979323846 - 4.50);
    EXPECT_NEAR(kin.jacobian(leg, zeroU).determinant(), 0.0, 1e-12);
  }
}

TEST(KinematicsJacobian, JacobianUnaffectedByFixedHipTranslation) {
  auto geometry = makeGeometry();
  const go1::Kinematics kin(geometry);
  const go1::Vec3 q(0.10, 0.67, -1.30);
  const go1::Mat3 before = kin.jacobian(go1::FR, q);
  geometry[go1::FR].hipPosition_B += go1::Vec3(1.0, 2.0, -3.0);
  const go1::Kinematics shifted(geometry);
  EXPECT_LT((before-shifted.jacobian(go1::FR,q)).norm(), 1e-14);
}

TEST(KinematicsJacobian, RejectsInvalidLegAndNonfiniteInputs) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q = go1::Vec3::Zero();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(kin.jacobian(-1, q), std::out_of_range);
  EXPECT_THROW(kin.geometricJacobian(4, q), std::out_of_range);
  EXPECT_THROW(kin.jacobian(go1::FR, go1::Vec3(nan,0,0)), std::invalid_argument);
  EXPECT_THROW(kin.footVelocity(go1::FR, q, go1::Vec3(0,nan,0)),
               std::invalid_argument);
}


TEST(KinematicsJacobian, DampedVelocityInverseAtOrdinaryPose) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q(0.15, 0.65, -1.39);
  const go1::Vec3 dqTrue(0.19, -0.14, 0.25);
  const go1::Vec3 desired = kin.footVelocity(go1::RL, q, dqTrue);
  go1::Vec3 dqFound = go1::Vec3::Constant(99);
  ASSERT_TRUE(kin.inverseVelocityKinematics(go1::RL, q, desired, 1e-6,
                                          go1::Vec3::Constant(2.0), dqFound));
  EXPECT_LT((dqFound-dqTrue).norm(), 1e-7);
}

TEST(KinematicsJacobian, DampedVelocityInverseStaysFiniteAtSingularity) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q(0.0, 0.0, 0.0);  // fully extended, mathematical test
  go1::Vec3 dqFound = go1::Vec3::Constant(99);
  ASSERT_TRUE(kin.inverseVelocityKinematics(go1::FR, q,
                                          go1::Vec3(0.05, 0.01, -0.01),
                                          1e-3, go1::Vec3::Constant(20.0),
                                          dqFound));
  EXPECT_LT(dqFound.norm(), 20.0);
}

TEST(KinematicsJacobian, DampedVelocityInverseRejectsUnsafeRate) {
  const go1::Kinematics kin(makeGeometry());
  const go1::Vec3 q(0.1, 0.67, -1.3);
  const go1::Vec3 sentinel = go1::Vec3::Constant(99);
  go1::Vec3 dq = sentinel;
  EXPECT_FALSE(kin.inverseVelocityKinematics(go1::FR,q,
               go1::Vec3::Constant(20.0), 1e-3,
               go1::Vec3::Constant(0.2), dq));
  EXPECT_LT((dq-sentinel).norm(), 1e-14);
  EXPECT_FALSE(kin.inverseVelocityKinematics(go1::FR,q,
               go1::Vec3::Zero(), 0.0,
               go1::Vec3::Constant(1.0), dq));
  EXPECT_LT((dq-sentinel).norm(), 1e-14);
}
