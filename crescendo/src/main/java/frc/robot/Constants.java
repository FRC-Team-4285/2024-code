// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
    public static final int POSITION_STARTING = 0;
    public static final int POSITION_FEED = 1;
    public static final int POSITION_TRAVEL = 2;

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
    public static final int MOTOR_INTAKE_CENTER = 11;

    // NEO Motor
    public static final int MOTOR_INTAKE_EDGE = 13;

    
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

    // Falcon Motor
    public static final int MOTOR_SHOOTER_FEEDER_A = 15;

    // Falcon Motor
    public static final int MOTOR_SHOOTER_FEEDER_B = 16;
  }

  public static final class SwerveConstants {

    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.1, 0.15, 0.01);

    /* Drive Controls */
    public static final int translationAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2; //was 4 on Xbox
    public static final int sliderAxis = 3;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5); //measured from center of each module
    public static final double wheelBase = Units.inchesToMeters(21.5);

    // nominal (real) divided by fudge factor
    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.0); //was 1.04085
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = 8.16; // Mk3 Standard drive ratio 
    public static final double angleGearRatio = 12.8; // Mk3 Standard steer ratio (does this need encoder stuff??)

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left, ++ quadrant
                    new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right, +- quadrant
                    new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left, -+ quadrant
                    new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right, -- quadrant
    );

    /* Swerve Profiling Values */
    public static final double maxSpeed = 1.3; // NOT a speed unit; robot gets faster if this is lower 2.0
    public static final double maxAngularVelocity = 12.0;

    public static final int frontLeftRotationMotorId = 2;
    public static final int frontLeftDriveMotorId = 1;

    public static final int frontRightRotationMotorId = 8;
    public static final int frontRightDriveMotorId = 7;

    public static final int rearLeftRotationMotorId = 4;
    public static final int rearLeftDriveMotorId = 3;

    public static final int rearRightRotationMotorId = 6;
    public static final int rearRightDriveMotorId = 5;

    public static final int frontLeftRotationEncoderId = 4;
    public static final int frontRightRotationEncoderId = 1;
    public static final int rearLeftRotationEncoderId = 3;
    public static final int rearRightRotationEncoderId = 2;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 7.5 / 4.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 3.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double cameraToFrontEdgeDistanceMeters = Units.inchesToMeters(7);

    public static final int PIGEON_SENSOR_ID = 0;

  }

}
