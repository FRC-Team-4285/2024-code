// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.beans.Expression;
import java.sql.Driver;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmPivotConstants;

public class ArmPivotSubsystem extends SubsystemBase {
  private final AprilTagFieldLayout field;
  private SwerveBase mSwerveBase; //private final SwerveBase mSwerveBase;  //
  private Pose2d speakerPose;
  private Pose2d ampPose;
  private Pose2d humanfeederPose;
  private Pose2d linePose;
  /** Pivots the arm. */

  private RelativeEncoder arm_pivot_encoder;

  // private PIDController arm_pid_controller;
  private double desired_location;

  //private static final int deviceID = 1;
  private CANSparkMax m_armMotorPivot1;
  private CANSparkMax m_armMotorPivot2;
  private SparkPIDController arm_pid_Controller;
  private SparkPIDController arm_pid_Controller2;
  // private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public static InterpolatingDoubleTreeMap angleTreeMap;

  private int active_mode;
  
  public ArmPivotSubsystem(SwerveBase mSwerveBase) {
    // Define the motors via spark max interface.
    m_armMotorPivot1 = new CANSparkMax(ArmPivotConstants.MOTOR_ARM_PIVOT_A, MotorType.kBrushless);
    m_armMotorPivot1.setIdleMode(IdleMode.kBrake);
    m_armMotorPivot2 = new CANSparkMax(ArmPivotConstants.MOTOR_ARM_PIVOT_B, MotorType.kBrushless);



    field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.mSwerveBase = mSwerveBase;
    //mSwerveBase = RobotContainer.m_swerveBase;
    // goalPose = DriverStation.getAlliance().get() == Alliance.Red ? field.getTagPose(4).get().toPose2d() : field.getTagPose(7).get().toPose2d();
    
    angleTreeMap = new InterpolatingDoubleTreeMap();

    m_armMotorPivot1.restoreFactoryDefaults();
    m_armMotorPivot2.restoreFactoryDefaults();
    
    m_armMotorPivot2.setIdleMode(IdleMode.kBrake);
    m_armMotorPivot1.setInverted(true);
    // m_armMotorPivot1.
    // Configure PID Controller.
    // arm_pid_controller = new PIDController(
      //   0.5, // Proportional gain
      //   0.0, // Integral gain
      //   0.0  // Derivative gain
      // );
      // arm_pid_controller.enableContinuousInput(-0.1, 0.1);
      // arm_pid_controller.setOutputRange(-0.1, 0.1);
      
      // desired_location = ArmPivotConstants.POSITION_PID_STARTING;
      
      // Define through-bore encoder attached to hex-shaft.
      // ENCA - DIO 8
      // ENCB - DIO 9
    arm_pivot_encoder = m_armMotorPivot1.getEncoder();


    m_armMotorPivot1.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    m_armMotorPivot2.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);

    m_armMotorPivot1.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    m_armMotorPivot2.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    m_armMotorPivot1.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float)6.6);
    m_armMotorPivot2.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float)6.6);

    m_armMotorPivot1.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float)-4);
    m_armMotorPivot2.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float)-4);

    // initialze PID controller and encoder objects
    arm_pid_Controller = m_armMotorPivot1.getPIDController();
    arm_pid_Controller2 = m_armMotorPivot2.getPIDController();

    m_armMotorPivot2.follow(m_armMotorPivot1);
    // PID coefficients
    kP = 0.0005; // 0.0005 
    kI = 0.0;
    kD = 0.00000001; 
    kIz = 0; 
    kFF = 0.000106;//0.000106 
    kMaxOutput = 0.15; 
    kMinOutput = -0.15;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    arm_pid_Controller.setP(kP);
    arm_pid_Controller.setI(kI);
    arm_pid_Controller.setD(kD);
    arm_pid_Controller.setIZone(kIz);
    arm_pid_Controller.setFF(kFF);
    arm_pid_Controller.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    arm_pid_Controller.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    arm_pid_Controller.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    arm_pid_Controller.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    arm_pid_Controller.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);

    // angleTreeMap.put(36.0, 1.31);
    // angleTreeMap.put(42.0, 1.336);
    // angleTreeMap.put(48.0, 1.37);
    // angleTreeMap.put(54.0, 1.35);
    // angleTreeMap.put(60.0, 1.36);
    // angleTreeMap.put(66.0, 1.382);
    // angleTreeMap.put(72.0, 1.388);
    // angleTreeMap.put(78.0, 1.395);
    // angleTreeMap.put(84.0 , 1.42);
    // angleTreeMap.put(90.0, 1.44);
    // angleTreeMap.put(96.0, 1.45);
    // angleTreeMap.put(102.0, 1.47);
    // angleTreeMap.put(108.0,  1.48);
    // angleTreeMap.put(114.0, 1.48);
    // angleTreeMap.put(120.0, 1.485);
    // angleTreeMap.put(126.0, 1.49);
    // angleTreeMap.put(132.0, 1.495);
    // angleTreeMap.put(138.0, 1.495);
    // angleTreeMap.put(144.0, 1.50);
    // angleTreeMap.put(150.0, 1.5);
    // angleTreeMap.put(156.0, 1.505);
    // angleTreeMap.put(162.0, 1.51);
    // angleTreeMap.put(168.0, 1.515);
    // angleTreeMap.put(174.0, 1.53);
    // angleTreeMap.put(180.0, 1.5375);
    // angleTreeMap.put(186.0, 1.5425);
    // angleTreeMap.put(192.0, 1.54525);
    // angleTreeMap.put(198.0, 1.54);
    // angleTreeMap.put(204.0, 1.535);

    // angleTreeMap.put(36.0, -1.57);
    // angleTreeMap.put(48.0, -1.75);
    // angleTreeMap.put(60.0, -2.0);
    // angleTreeMap.put(72.0, -2.25);
    // angleTreeMap.put(84.0 , -2.5);
    // angleTreeMap.put(96.0, -2.75);
    // angleTreeMap.put(108.0,  -3.0);
    // angleTreeMap.put(120.0, -3.125);
    // angleTreeMap.put(138.0, -3.25);
    // angleTreeMap.put(150.0, -3.375);
    // angleTreeMap.put(162.0, -3.5);
    // angleTreeMap.put(174.0, -3.675);
    // angleTreeMap.put(186.0, -3.75);
    // angleTreeMap.put(198.0, -3.875);
    // angleTreeMap.put(204.0, -4.0);

    // .4525 Meters From Center Of Robot 
    //Add 15 inches to measurements without bumpers retest with 6 to one
    //  angleTreeMap.put(30.0, 2.0);
    // angleTreeMap.put(36.0, 2.2381);
    // angleTreeMap.put(42.0, 2.3571);
    // angleTreeMap.put(48.0, 2.3571);
    // angleTreeMap.put(54.0, 2.5476);
    // angleTreeMap.put(60.0, 2.7142);
    // angleTreeMap.put(66.0, 2.7857);
    // angleTreeMap.put(72.0, 2.9285);
    // angleTreeMap.put(78.0, 2.8809);
    // angleTreeMap.put(84.0, 3.0238);
    // angleTreeMap.put(90.0, 3.0714);
    // angleTreeMap.put(96.0, 3.1666);
    // angleTreeMap.put(102.0, 3.1190);
    // angleTreeMap.put(108.0, 3.0714);
    // angleTreeMap.put(114.0, 3.1428);
    // angleTreeMap.put(120.0, 3.2380);
    // angleTreeMap.put(126.0, 3.2857);
    // angleTreeMap.put(132.0, 3.2619);
    // angleTreeMap.put(138.0, 3.2380);
    // angleTreeMap.put(144.0, 3.3333);
    // angleTreeMap.put(150.0, 3.3333);
    // angleTreeMap.put(156.0, 3.3809);
    // angleTreeMap.put(162.0, 3.3571);
    // angleTreeMap.put(168.0, 3.3809);
    // angleTreeMap.put(174.0, 3.3809);
    // angleTreeMap.put(180.0, 3.4285);
    


//with plus 15
    // angleTreeMap.put(45.0, 2.0);
    // angleTreeMap.put(51.0, 2.2381);
    // angleTreeMap.put(57.0, 2.3571);
    // angleTreeMap.put(63.0, 2.3571);
    // angleTreeMap.put(69.0, 2.5476);
    // angleTreeMap.put(75.0, 2.7142);
    // angleTreeMap.put(81.0, 2.7857);
    // angleTreeMap.put(87.0, 2.9285);
    // angleTreeMap.put(93.0, 2.8809);
    // angleTreeMap.put(99.0, 3.0238);
    // angleTreeMap.put(105.0, 3.0714);
    // angleTreeMap.put(111.0, 3.1666);
    // angleTreeMap.put(117.0, 3.1190);
    // angleTreeMap.put(123.0, 3.0714);
    // angleTreeMap.put(129.0, 3.1428);
    // angleTreeMap.put(135.0, 3.2380);
    // angleTreeMap.put(141.0, 3.2857);
    // angleTreeMap.put(147.0, 3.2619);
    // angleTreeMap.put(153.0, 3.2380);
    // angleTreeMap.put(159.0, 3.3333);
    // angleTreeMap.put(165.0, 3.3333);
    // angleTreeMap.put(171.0, 3.3809);
    // angleTreeMap.put(177.0, 3.3571);
    // angleTreeMap.put(183.0, 3.3809);
    // angleTreeMap.put(189.0, 3.3809);
    // angleTreeMap.put(195.0, 3.4285);
    // 180 inch -- 1.5375
    // 186 inch -- 1.5425
    // 192 inch -- 1.54525
    // 198 inch -- 1.54
    // 204 inch -- 1.535
    // 216 inch --1.5375

      //Tree Map Using Cameras At Home
    angleTreeMap.put(48.0, 2.190476);
    angleTreeMap.put(52.15, 2.23805);
    angleTreeMap.put(56.3, 2.285713);
    angleTreeMap.put(60.95, 2.333);
    angleTreeMap.put(65.6, 2.380951);
    angleTreeMap.put(68.2, 2.4404);
    angleTreeMap.put(70.8, 2.499999);
    angleTreeMap.put(75.75, 2.5859);
    angleTreeMap.put(80.7, 2.761903);
    angleTreeMap.put(84.1, 2.8214);
    angleTreeMap.put(87.5, 2.880950);
    angleTreeMap.put(91.9, 2.9047);
    angleTreeMap.put(96.3, 2.928569);
    angleTreeMap.put(99.65, 2.98775);
    angleTreeMap.put(103.0, 3.047617);
    angleTreeMap.put(106.45, 3.0238);
    angleTreeMap.put(109.9, 2.999998);
    angleTreeMap.put(112.9, 3.03569);
    angleTreeMap.put(115.9, 3.071426);
    angleTreeMap.put(119.95, 3.0833);
    angleTreeMap.put(124.0, 3.095236);
    angleTreeMap.put(128.9, 3.1428);
    angleTreeMap.put(133.8, 3.190474);
    angleTreeMap.put(137.9, 3.2142);
    angleTreeMap.put(142.0, 3.238093);
    angleTreeMap.put(145.45, 3.30945);
    angleTreeMap.put(148.9, 3.380949);
    angleTreeMap.put(155.05, 3.369);
    angleTreeMap.put(161.2, 3.357140);
    angleTreeMap.put(165.0, 3.3809);
    angleTreeMap.put(168.8, 3.404758);
    angleTreeMap.put(174.9, 3.41025);
    angleTreeMap.put(181.0, 3.415860);//Tested With Auto Align
    angleTreeMap.put(184.75, 3.41895);
    angleTreeMap.put(188.5, 3.422150);
  //   angleTreeMap.put(153.0, 3.2380);
  //   angleTreeMap.put(159.0, 3.3333);
  //   angleTreeMap.put(165.0, 3.3333);
  //   angleTreeMap.put(171.0, 3.3809);
  //   angleTreeMap.put(177.0, 3.3571);
  //   angleTreeMap.put(183.0, 3.3809);
  //   angleTreeMap.put(189.0, 3.3809);
  //   angleTreeMap.put(195.0, 3.4285);
  }


  public void setGoalPose() {
    speakerPose = DriverStation.getAlliance().get() == Alliance.Red ? field.getTagPose(4).get().toPose2d() : field.getTagPose(7).get().toPose2d();
  }

  public Pose2d getSpeakerPose(){
    return speakerPose;
  }

  public void setAmpPose(){
    ampPose = DriverStation.getAlliance().get() == Alliance.Red ? field.getTagPose(5).get().toPose2d() : field.getTagPose(6).get().toPose2d();
  }

  public Rotation2d getAmpAngle(){
    return DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(-270) : Rotation2d.fromDegrees(270);
  }

  public Pose2d getAmpPose(){
    return ampPose;
  }

  public void setHumanFeederPose(){
    humanfeederPose = DriverStation.getAlliance().get() == Alliance.Red ? field.getTagPose(9).get().toPose2d() : field.getTagPose(1).get().toPose2d();
  }

  public Rotation2d getHumanFeederAngle(){
    return DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(-120) : Rotation2d.fromDegrees(120);
  }

  public Pose2d getHumanFeederPose(){
    return humanfeederPose;
  }

  public void setLinePose() {
    speakerPose = DriverStation.getAlliance().get() == Alliance.Red ? field.getTagPose(4).get().toPose2d() : field.getTagPose(7).get().toPose2d();
  }

  public Rotation2d getLineAngle(){
    return DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(-0) : Rotation2d.fromDegrees(0);
  }

  public Pose2d getLinePose(){
    return linePose;
  }


 

  /**
   * Puts the arm into the desired mode.
   */
  public void go_to_mode(int MODE) {
      // if (MODE == ArmPivotConstants.POSITION_STARTING) {
      //   desired_location = ArmPivotConstants.POSITION_PID_STARTING;
      // }
      if (MODE == ArmPivotConstants.POSITION_INTAKE_FLOOR) {
        desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FLOOR;
      }
      else if (MODE == ArmPivotConstants.POSITION_INTAKE_FEEDER) {
        desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FEEDER;
      }
      else if (MODE == ArmPivotConstants.POSITION_AMP_SCORING) {
        desired_location = ArmPivotConstants.POSITION_PID_AMP_SCORING;
      }
      else if (MODE == ArmPivotConstants.POSITION_SHOOTING) {
       // SmartDashboard.putNumber("Distance to Goal", Units.metersToInches(mSwerveBase.getOdometry().getEstimatedPosition().getTranslation().getDistance(speakerPose.getTranslation())));
        desired_location = angleTreeMap.get(Units.metersToInches(mSwerveBase.getOdometry().getEstimatedPosition().getTranslation().getDistance(speakerPose.getTranslation())));
      }
      else if (MODE == ArmPivotConstants.POSITION_HUMAN_FEEDER) {
        desired_location = ArmPivotConstants.POSITION_PID_HUMAN_FEEDER;
      }
       else if (MODE == ArmPivotConstants.POSITION_STORE) {
        desired_location = ArmPivotConstants.POSITION_PID_STORE;
      }
       else if (MODE == ArmPivotConstants.POSITION_ERRECTED) {
        desired_location = ArmPivotConstants.POSITION_PID_ERRECTED;
      }
      else if (MODE == ArmPivotConstants.POSITION_SHOOTING_WITHOUT_CAMERAS) {
        desired_location = ArmPivotConstants.POSITION_PID_SHOOTING_WITHOUT_CAMERAS;
      }
      else if (MODE == ArmPivotConstants.POSITION_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG) {
        desired_location = ArmPivotConstants.POSITION_PID_SHOOTING_WITHOUT_CAMERAS_STAGE_LEG;
      }
       else if (MODE == ArmPivotConstants.POSITION_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG) {
        desired_location = ArmPivotConstants.POSITION_PID_SHOOTING_WITHOUT_CAMERAS_2ND_STAGE_LEG;
      }
       else if (MODE == ArmPivotConstants.POSITION_AMP_SCORING_POS) {
        desired_location = ArmPivotConstants.POSITION_PID_AMP_SCORING_POS;
      }
      else if (MODE == ArmPivotConstants.POSITION_SHOOTING_WITHOUT_CAMERAS_N1) {
        desired_location = ArmPivotConstants.POSITION_PID_SHOOTING_WITHOUT_CAMERAS_N1;
      }
       else if (MODE == ArmPivotConstants.POSITION_SHOOTING_DEFENCE) {
        desired_location = ArmPivotConstants.POSITION_PID_SHOOTING_DEFENCE;
      }
      // Example
      // else if (MODE == ArmPivotConstants. (Name of COnstant)) {
      //   desired_location = ArmPivotConstants.(Comand);
      // }
      else {
          System.out.println("Unknown position defined.");
          return;
      }
  }

// public void go_to_mode(int MODE, double POS) {
//       if (MODE == ArmPivotConstants.POSITION_STARTING) {
//         desired_location = ArmPivotConstants.POSITION_PID_STARTING;
//       }
//       else if (MODE == ArmPivotConstants.POSITION_INTAKE_FLOOR) {
//         desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FLOOR;
//       }
//       else if (MODE == ArmPivotConstants.POSITION_INTAKE_FEEDER) {
//         desired_location = ArmPivotConstants.POSITION_PID_INTAKE_FEEDER;
//       }
//       else if (MODE == ArmPivotConstants.POSITION_TRAVEL) {
//         desired_location = ArmPivotConstants.POSITION_PID_TRAVEL;
//       }
//       else if (MODE == ArmPivotConstants.POSITION_AMP_SCORING){
//         desired_location = ArmPivotConstants.POSITION_PID_AMP_SCORING;
//       }

//       else {
//           System.out.print("Unknown position defined.");
//           return;
//       }

//   }

  public void stop() {
    m_armMotorPivot1.set(0.0);
    m_armMotorPivot2.set(0.0);
  }

  public Command stopCommand(){
    return new InstantCommand(() -> stop());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // System.out.println(arm_pivot_encoder.isConnected() + " get" + arm_pivot_encoder.get() + " getDist " + arm_pivot_encoder.getDistance() + " getAbsPos" + arm_pivot_encoder.getAbsolutePosition() + " getPosOffset " + arm_pivot_encoder.getPositionOffset());
    
    speakerPose = field.getTagPose(4).get().toPose2d();
    System.out.println(Units.metersToInches(mSwerveBase.getOdometry().getEstimatedPosition().getTranslation().getDistance(speakerPose.getTranslation())));
  

    // // Get the current position from the encoder
    // double currentPosition = arm_pivot_encoder.get();

    // // Calculate the error (the difference between setpoint and current position)
    // double error = desired_location - currentPosition;

    // // Calculate the PID output
    // double unclampedOutput = arm_pid_controller.calculate(desired_location);

    // // Set motor output.
    // if (error < -0.01 || error > 0.01) {
    //   double flip = 1;
    //   if (error <= 0) {
    //     flip = -1;
    //   }

    //   double output = 0.0;
    //   if (unclampedOutput >= 0) {
    //       output = 0.05 + (unclampedOutput / 2.0);
    //       // Clamp the output to the specified range
    //       output = Math.max(0.05, Math.min(0.5, output));
    //   }
    //   else {
    //       output = -0.05 - (unclampedOutput / 2.0);
    //       // Clamp the output to the specified range
    //       output = Math.min(-0.05, Math.max(-0.5, output));
    //   }

    //   arm_pivot_motor_a.set(output * flip);
    //   arm_pivot_motor_b.set(output * flip);

    //   //System.out.println("\n\n\nDesired: " + desired_location + "\nRaw Output: " + unclampedOutput + "\nOutput: " + output + "\nError: " + error + "\n\n");
    // }
    // else {
    //   arm_pivot_motor_a.set(0.0);
    //   arm_pivot_motor_b.set(0.0);
    // }

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { arm_pid_Controller.setP(p); kP = p; }
    if((i != kI)) { arm_pid_Controller.setI(i); kI = i; }
    if((d != kD)) { arm_pid_Controller.setD(d); kD = d; }
    if((iz != kIz)) { arm_pid_Controller.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { arm_pid_Controller.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      arm_pid_Controller.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { arm_pid_Controller.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { arm_pid_Controller.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { arm_pid_Controller.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { arm_pid_Controller.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint, processVariable;

    setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
    //TODO You can comment the line under to set the setpoint to a desired angle in rotations
     //arm_pid_Controller.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);//Use for testing only

     //bad do not touch without permission wass i am talking to you 
    // arm_pid_Controller.setReference(angleTreeMap.get(Units.metersToInches(mSwerveBase.getOdometry().getEstimatedPosition().getTranslation().getDistance(goalPose.getTranslation()))), CANSparkMax.ControlType.kSmartMotion);
     arm_pid_Controller.setReference(desired_location, CANSparkMax.ControlType.kSmartMotion);

    processVariable = arm_pivot_encoder.getPosition();
    // SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", m_armMotorPivot1.getAppliedOutput());
    

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
