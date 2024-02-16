// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AprilTagConstants {
  }

  public static class ArmPivotConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // 2 NEO Motor
    public static final int MOTOR_ARM_PIVOT_A = 20;
    public static final int MOTOR_ARM_PIVOT_B = 21;

    // PID Locations
     public static final int POSITION_STARTING = 0;
    public static final int POSITION_INTAKE_FLOOR = 1;
    public static final int POSITION_INTAKE_FEEDER = 2;
    public static final int POSITION_TRAVEL = 3;
    public static final int POSITION_AMP_SCORING = 4;
    public static final int POSITION_SHOOTING = 5;
    public static final int POSITION_HUMAN_FEEDER = 6;
    public static final int POSITION_STORE = 7;
    public static final int POSITION_ERRECTED = 8;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS = 9;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG = 10;
    public static final int POSITION_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG = 11;
    public static final int AMP_SCORING_POS = 12;

    public static final double POSITION_PID_STARTING = 5.5;// Robot Will go here on start
    public static final double POSITION_PID_INTAKE_FLOOR = POSITION_STARTING;
    public static final double POSITION_PID_INTAKE_FEEDER = 1.31;
    public static final double POSITION_PID_AMP_SCORING = -2.3333;
    public static final double POSITION_PID_HUMAN_FEEDER = -2.904;
    public static final double POSITION_PID_STORE = 5.5;
    public static final double POSITION_PID_ERRECTED = 0;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS = 2.1666;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG = 3.41;
    public static final double POSITION_PID_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG = 3.15;
    public static final double AMP_PID_SCORING_POS = -1.9; //-1.738;

    // 36 inch -- 1.31
    // 42 inch -- 1.336
    // 48 inch -- 1.37
    // 54 inch -- 1.35
    // 60 inch -- 1.36
    // 66 inch -- 1.382
    // 72 inch -- 1.388
    // 78 inch -- 1.395
    // 84 inch -- 1.42
    // 90 inch -- 1.44
    // 96 inch -- 1.45
    // 102 inch -- 1.47
    // 108 inch -- 1.48
    // 114 inch -- 1.48
    // 120 inch -- 1.485
    // 126 inch -- 1.49
    // 132 inch -- 1.495
    // 138 inch -- 1.495
    // 144 inch -- 1.50
    // 150 inch -- 1.50
    // 156 inch -- 1.505
    // 162 inch -- 1.51
    // 168 inch -- 1.515
    // 174 inch -- 1.53
    // 180 inch -- 1.5375
    // 186 inch -- 1.5425
    // 192 inch -- 1.54525
    // 198 inch -- 1.54
    // 204 inch -- 1.535
    // 216 inch --1.5375 This value shoot good disk high and bad disk low

    // public static final double POSITION_PID_INTAKE_FEEDER = 1.4;
    public static final double POSITION_PID_TRAVEL = 0.22;

  }

  public static class ClimberConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */
  }

  public static class IntakeConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    public static final int MOTOR_INTAKE_FLOOR = 13;

  }

  public static class ShooterConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // Falcon Motor
    public static final int MOTOR_SHOOTER_A = 15;

    // Falcon Motor
    public static final int MOTOR_SHOOTER_B = 16;
  }

  public static class ShooterFeederConstants {
    /*
     * These numbers must match the Spark connecting to this motor.
     */

    // NEO Motor
    public static final int MOTOR_SHOOTER_FEEDER = 11;

  }

  public static final class SwerveConstants {

    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);

    /* Drive Controls */
    public static final int translationAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2; // was 4 on Xbox
    public static final int sliderAxis = 3;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5); // measured from center of each module
    public static final double wheelBase = Units.inchesToMeters(21.5);

    // nominal (real) divided by fudge factor
    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.0); // was 1.04085
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = 5.14; // Mk3 Standard drive ratio
    public static final double angleGearRatio = 12.8; // Mk3 Standard steer ratio (does this need encoder stuff??)

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left, ++ quadrant
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right, +- quadrant
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left, -+ quadrant
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right, -- quadrant
    );

    public static Translation2d mDriveRadius = new Translation2d(trackWidth/2, wheelBase / 2);
    /* Swerve Profiling Values */
    public static final double maxSpeed = 1.9; // NOT a speed unit; robot gets faster if this is lower 2.0//was 1.3
    public static final double maxAngularVelocity = 12.0;

    public static final int frontLeftRotationMotorId = 2;
    public static final int frontLeftDriveMotorId = 1;

    public static final int frontRightRotationMotorId = 8;
    public static final int frontRightDriveMotorId = 7;

    public static final int rearLeftRotationMotorId = 4;
    public static final int rearLeftDriveMotorId = 3;

    public static final int rearRightRotationMotorId = 6;
    public static final int rearRightDriveMotorId = 5;

    public static final int frontLeftRotationEncoderId = 1;
    public static final int frontRightRotationEncoderId = 4;
    public static final int rearLeftRotationEncoderId = 2;
    public static final int rearRightRotationEncoderId = 3;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

    public static final int PIGEON_SENSOR_ID = 0;

  }

  public static final class VisionConstants {

    //TODO Calculate and get these values, Units need to be in meters and radians
    /**
     * Physical location of the apriltag camera on the robot, relative to the center
     * of the robot.
     */
//Robot In         CAD        RealLife
//Height       0.567547072    0.562
//Front Back   0.0631137      0.063
//Left Right   0.3109483      0.30125
//
//BTY Values -0.06, 0.2, -0.2127
//
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
    //     new Translation3d(-0.063, 0.30125, -0.562),// Get from CAD Model In meters
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45.0)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
    //     new Translation3d(0.063, 0.30125, -0.562),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45.0)));
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
    //     new Translation3d(-0.063, -0.30125, -0.562),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135.0)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(0.063, -0.30125, -0.562),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135.0)));
    

    //     public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
    //     new Translation3d(-0.063, 0.30125, -0.562),// Get from CAD Model In meters
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45.0)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
    //     new Translation3d(-0.063, -0.30125, -0.562),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135)));
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
    //     new Translation3d(0.063, -0.30125, -0.562),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(0.063, 0.30125, -0.562),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45)));
    //  
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
        new Translation3d(0.063, -0.30125, -0.562),// Get from CAD Model In meters
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45.0)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
        new Translation3d(0.063, 0.30125, -0.562),
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135)));
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
        new Translation3d(-0.063, 0.30125, -0.562),
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135)));

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
        new Translation3d(-0.063, -0.30125, -0.562),
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45)));
    
    //Lime Light 
    //          CAD             RealLife
    // Height  0.6673348        0.669
    //Front    0.792804         0.059
    //Right    0.3028135        0.303
    //Angle    29.7169722 Deg   30 Deg
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(0.3109483, 0.0631137, -0.567547072),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135.0)));


    public static final double FIELD_LENGTH_METERS = 16.542;
    public static final double FIELD_WIDTH_METERS = 8.2042;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
      new Translation2d(FIELD_LENGTH_METERS , FIELD_WIDTH_METERS),
      new Rotation2d(Math.PI)
    );

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

}
