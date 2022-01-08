//
// Created by abiel on 10/28/21.
//

#include "gtest/gtest.h"
#include "Control/Kinematics/Swerve/SwerveState.h"
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <Control/Kinematics/Swerve/SwerveKinematics.h>

TEST(SwerveState, toWpiTest){
    SwerveState test;
    test.topLeft.wheelVelocity = 1;
    test.topLeft.wheelAngle = 0.1;
    test.topRight.wheelVelocity = 2;
    test.topRight.wheelAngle = 0.2;
    test.backLeft.wheelVelocity = 3;
    test.backLeft.wheelAngle = 0.3;
    test.backRight.wheelVelocity = 4;
    test.backRight.wheelAngle = 0.4;

    auto wpiState = test.toWPI();
    ASSERT_EQ(test.topLeft.wheelVelocity, wpiState[0].speed.value());
    ASSERT_EQ(test.topLeft.wheelAngle, wpiState[0].angle.Radians().value());

    EXPECT_EQ(test.topRight.wheelVelocity, wpiState[1].speed.value());
    EXPECT_EQ(test.topRight.wheelAngle, wpiState[1].angle.Radians().value());

    EXPECT_EQ(test.backLeft.wheelVelocity, wpiState[2].speed.value());
    EXPECT_EQ(test.backLeft.wheelAngle, wpiState[2].angle.Radians().value());

    EXPECT_EQ(test.backRight.wheelVelocity, wpiState[3].speed.value());
    EXPECT_EQ(test.backRight.wheelAngle, wpiState[3].angle.Radians().value());
}

TEST(SwerveState, fromWpi){
    std::array<frc::SwerveModuleState, 4> wpiStates;
    for(int i = 0; i < 4; i++){
        wpiStates[i].speed = units::meters_per_second_t((double) i / 10.0);
        wpiStates[i].angle = frc::Rotation2d(units::radian_t((double) i / 100.0));
    }

    auto state = SwerveState(wpiStates);
    for(int i = 0; i < 4; i++){
        EXPECT_NEAR(state.wheels[i]->wheelVelocity, wpiStates[i].speed.value(), 0.001);
        EXPECT_NEAR(state.wheels[i]->wheelAngle, wpiStates[i].angle.Radians().value(), 0.001);
    }
}

TEST(SwerveState, swerveOdometry){
    frc::Translation2d frontLeft{0.381_m, 0.381_m};
    frc::Translation2d frontRight{0.381_m, -0.381_m};
    frc::Translation2d backLeft{-0.381_m, 0.381_m};
    frc::Translation2d backRight{-0.381_m, -0.381_m};

    frc::SwerveDriveKinematics<4> kinem(frontLeft, frontRight, backLeft, backRight);
    frc::SwerveDriveOdometry<4> odom(kinem, frc::Rotation2d(), frc::Pose2d());

    SwerveState test;
    test.topLeft.wheelVelocity = 1;
    test.topLeft.wheelAngle = 0.0;
    test.topRight.wheelVelocity = 1;
    test.topRight.wheelAngle = 0.0;
    test.backLeft.wheelVelocity = 1;
    test.backLeft.wheelAngle = 0.0;
    test.backRight.wheelVelocity = 1;
    test.backRight.wheelAngle = 0.0;

    double time = 0;
    while(time <= 10){
        odom.UpdateWithTime(units::second_t(time), frc::Rotation2d(), test.toWPI());
        time += 0.2;
    }

    EXPECT_NEAR(odom.GetPose().X().value(), 10, 0.01);
    EXPECT_NEAR(odom.GetPose().Y().value(), 0, 0.01);

    test.topLeft.wheelVelocity = -1;
    test.topRight.wheelVelocity = -1;
    test.backLeft.wheelVelocity = -1;
    test.backRight.wheelVelocity = -1;

    while(time <= 20){
        odom.UpdateWithTime(units::second_t(time), frc::Rotation2d(), test.toWPI());
        time += 0.2;
    }

    EXPECT_NEAR(odom.GetPose().X().value(), 0, 0.01);
    EXPECT_NEAR(odom.GetPose().Y().value(), 0, 0.01);
}

TEST(SwerveKinematics, wpiKinematics){
    frc::Translation2d frontLeft{1_m, 1_m};
    frc::Translation2d frontRight{1_m, -1_m};
    frc::Translation2d backLeft{-1_m, 1_m};
    frc::Translation2d backRight{-1_m, -1_m};

    frc::SwerveDriveKinematics<4> kinem(frontLeft, frontRight, backLeft, backRight);

    frc::ChassisSpeeds targetVel;
    targetVel.vx = 1_mps; targetVel.vy = 0.0_mps;
    targetVel.omega = 0_rad_per_s;

    auto frcStates = kinem.ToSwerveModuleStates(targetVel);

    for(int i = 0; i < 4; i++){
        EXPECT_NEAR(frcStates[i].speed.value(), 1, 0.01);
        EXPECT_NEAR(frcStates[i].angle.Radians().value(), 0, 0.01);
    }

    targetVel.vx = -1_mps;
    frcStates = kinem.ToSwerveModuleStates(targetVel);
    for(int i = 0; i < 4; i++){
        EXPECT_NEAR(frcStates[i].speed.value(), 1, 0.01);
        EXPECT_NEAR(frcStates[i].angle.Radians().value(), M_PI, 0.01);
    }

    targetVel.vx = 0_mps; targetVel.vy = 1_mps;
    frcStates = kinem.ToSwerveModuleStates(targetVel);
    for(int i = 0; i < 4; i++){
        EXPECT_NEAR(frcStates[i].speed.value(), 1, 0.01);
        EXPECT_NEAR(frcStates[i].angle.Radians().value(), M_PI_2, 0.01);
    }

    targetVel.vx = 0_mps; targetVel.vy = 0_mps;
    targetVel.omega = units::radians_per_second_t(M_PI);
    frcStates = kinem.ToSwerveModuleStates(targetVel);
    std::vector<std::string> names = {"Front Left", "Front Right", "Back Left", "Back Right"};
    for(int i = 0; i < 4; i++){
        EXPECT_TRUE(frcStates[i].speed.value() > 0);
    }

    EXPECT_NEAR(frcStates[0].angle.Degrees().value(), 135, 0.01);
    EXPECT_NEAR(frcStates[1].angle.Degrees().value(), 45, 0.01);
    EXPECT_NEAR(frcStates[2].angle.Degrees().value(), -135, 0.01);
    EXPECT_NEAR(frcStates[3].angle.Degrees().value(), -45, 0.01);
}

TEST(SwerveKinematics, transformToFieldOriented){
    auto vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, frc::Rotation2d());
    ASSERT_NEAR(vel.vx.value(), 1, 0.01);
    ASSERT_NEAR(vel.vy.value(), 0, 0.01);
    ASSERT_NEAR(vel.omega.value(), 0, 0.01);

    vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, frc::Rotation2d(units::radian_t(M_PI)));
    EXPECT_NEAR(vel.vx.value(), -1, 0.01);
    EXPECT_NEAR(vel.vy.value(), 0, 0.01);
    EXPECT_NEAR(vel.omega.value(), 0, 0.01);

    vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, frc::Rotation2d(units::radian_t(-M_PI)));
    EXPECT_NEAR(vel.vx.value(), -1, 0.01);
    EXPECT_NEAR(vel.vy.value(), 0, 0.01);
    EXPECT_NEAR(vel.omega.value(), 0, 0.01);

    vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, frc::Rotation2d(units::radian_t(M_PI_2)));
    EXPECT_NEAR(vel.vx.value(), 0, 0.01);
    EXPECT_NEAR(vel.vy.value(), -1, 0.01);
    EXPECT_NEAR(vel.omega.value(), 0, 0.01);

    vel = frc::ChassisSpeeds::FromFieldRelativeSpeeds(1_mps, 0_mps, 0_rad_per_s, frc::Rotation2d(units::radian_t(-M_PI_2)));
    EXPECT_NEAR(vel.vx.value(), 0, 0.01);
    EXPECT_NEAR(vel.vy.value(), 1, 0.01);
    EXPECT_NEAR(vel.omega.value(), 0, 0.01);
}