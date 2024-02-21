package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.sql.Driver;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;


public class SwerveBase extends SubsystemBase {  
  private final WPI_Pigeon2 pigeonSensor;
  private final AHRS navX;
  private Pigeon2Configuration pigeonConfig;
  private double oldPigeonYaw = 0.0;

  public SwerveBase() {
    navX = new AHRS(SPI.Port.kMXP);
    pigeonSensor = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_SENSOR_ID);
    pigeonConfig = new Pigeon2Configuration();
    pigeonSensor.configFactoryDefault();
    pigeonSensor.reset();
    zeroPigeon();

    // if(DriverStation.getAlliance().get() == Alliance.Red){
    //   pigeonSensor.addYaw(90);
    // }
    pigeonSensor.getAllConfigs(pigeonConfig);


    odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());

    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    rearLeft.initRotationOffset();
    rearRight.initRotationOffset();

    // reset the measured distance driven for each module
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

    rearRight.getDriveMotor().setInverted(true);
    rearLeft.getDriveMotor().setInverted(false);
    frontRight.getDriveMotor().setInverted(true);
    frontLeft.getDriveMotor().setInverted(false);

    rearRight.getRotationMotor().setInverted(false);
    rearLeft.getRotationMotor().setInverted(false);
    frontRight.getRotationMotor().setInverted(false);
    frontLeft.getRotationMotor().setInverted(false);

    double driveBaseRadius = Constants.SwerveConstants.mDriveRadius.getNorm();

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(4.0, 0.0, 0.885), // Translation PID constants   0-9.99  under  0.885 -0.8855  over           new PIDConstants(5.0, 0.0, 0.00005), // Translation PID constants 
                    new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants      0-9.99
                    5.7912, // Max module speed, in m/s
                    driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(true , true, 1 , 0.25) // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void zeroPigeon() {
    pigeonSensor.reset();
  }

  /**
   * Subsystem that controls the drivetrain of the robot
   * Handles all the odometry and base movement for the chassis
   */

  /**
   * absolute encoder offsets for the wheels
   * 180 degrees added to offset values to invert one side of the robot so that it
   * doesn't spin in place
   */
  private static final double frontLeftAngleOffset = Units.degreesToRadians(239.94);//
  private static final double frontRightAngleOffset = Units.degreesToRadians(159.7);//
  private static final double rearLeftAngleOffset = Units.degreesToRadians(202.85);//
  private static final double rearRightAngleOffset = Units.degreesToRadians(132.45);//

  private Pose2d m_pose = new Pose2d(0, 0, new Rotation2d());
  private final double SCALE_X = -1/0.9;
  private final double SCALE_Y = -1/0.9;

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor can ID
   * rotation motor can ID
   * external CANCoder can ID
   * measured CANCoder offset
   */

  private final SwerveModule frontLeft = new SwerveModule(
      SwerveConstants.frontLeftDriveMotorId,
      SwerveConstants.frontLeftRotationMotorId,
      SwerveConstants.frontLeftRotationEncoderId,
      frontLeftAngleOffset,
      this);

  private final SwerveModule frontRight = new SwerveModule(
      SwerveConstants.frontRightDriveMotorId,
      SwerveConstants.frontRightRotationMotorId,
      SwerveConstants.frontRightRotationEncoderId,
      frontRightAngleOffset,
      this);

  public SwerveModule getFrontRight() {
    return frontRight;
  }

  private final SwerveModule rearLeft = new SwerveModule(
      SwerveConstants.rearLeftDriveMotorId,
      SwerveConstants.rearLeftRotationMotorId,
      SwerveConstants.rearLeftRotationEncoderId,
      rearLeftAngleOffset,
      this);

  private final SwerveModule rearRight = new SwerveModule(
      SwerveConstants.rearRightDriveMotorId,
      SwerveConstants.rearRightRotationMotorId,
      SwerveConstants.rearRightRotationEncoderId,
      rearRightAngleOffset,
      this);


  /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */

  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, new Rotation2d(),
      getModulePositions(), new Pose2d());
  private boolean needPigeonReset = false;

  public SwerveDrivePoseEstimator getOdometry() {
    return odometry;
  }

  public Pose3d getPose3d(){
    return new Pose3d(getOdometry().getEstimatedPosition());
  }
  
  @Override
  public void periodic() {

    // update the odometry every 20ms
    odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putString("Robot pose",
        getPose().toString());
    SmartDashboard.putNumber("Bot Heading",
        getHeading().getDegrees());
    SmartDashboard.putString("Pigeon Rotation",
    pigeonSensor.getRotation2d().toString());
    SmartDashboard.putNumber("Pigeon Yaw",
    pigeonSensor.getYaw());
    SmartDashboard.putNumber("Pigeon Compass",
    pigeonSensor.getCompassHeading());

    SmartDashboard.putString("FL Wheel Angle", frontLeft.getCanCoderAngle().toString());
    SmartDashboard.putString("FR Wheel Angle", frontRight.getCanCoderAngle().toString());
    SmartDashboard.putString("RL Wheel Angle", rearLeft.getCanCoderAngle().toString());
    SmartDashboard.putString("RR Wheel Angle", rearRight.getCanCoderAngle().toString());
  }

  /**
   * method for driving the robot
   * Parameters:
   * forward linear value
   * sideways linear value
   * rotation value
   * if the control is field relative or robot relative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    if (!isFieldRelative) {
      if (!needPigeonReset) {
        needPigeonReset = true;
        oldPigeonYaw = pigeonSensor.getYaw();
      }

      zeroPigeon();
    }
    else if (needPigeonReset) {
      needPigeonReset = false;
      pigeonSensor.setYaw(oldPigeonYaw);
    }

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      forward, strafe, rotation, getHeading()
    );

    // new ChassisSpeeds(forward, strafe, rotation)

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states);

  }

  public Pose2d getScaledPose() {
    m_pose = getPose();
    final var translation = new Translation2d(m_pose.getX() * SCALE_X, m_pose.getY() * SCALE_Y);
    final var rotation = m_pose.getRotation().rotateBy(new Rotation2d(0));
    return new Pose2d(translation.getX(), translation.getY(), rotation);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeonSensor.getCompassHeading());
  }

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative, boolean isAutoBalancing) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */
    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states, isAutoBalancing);

  }

  public void robotRelativeDrive(ChassisSpeeds speeds){
    setModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
    rearRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates, boolean isAutoBalancing) {
    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
    frontLeft.setDesiredStateClosedLoop(moduleStates[0], isAutoBalancing);
    frontRight.setDesiredStateClosedLoop(moduleStates[1], isAutoBalancing);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2], isAutoBalancing);
    rearRight.setDesiredStateClosedLoop(moduleStates[3], isAutoBalancing);

  }

  // returns an array of SwerveModuleState
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
        new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return states;

  }

  // returns an array of SwerveModulePositions
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeft.getCurrentDistanceMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModulePosition(frontRight.getCurrentDistanceMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModulePosition(rearLeft.getCurrentDistanceMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModulePosition(rearRight.getCurrentDistanceMetersPerSecond(), rearRight.getIntegratedAngle())
    };

    return positions;

  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  // reset the current pose to a desired pose
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  // reset the measured distance driven for each module
  public void resetDriveDistances() {
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();
  }

  // get the current heading of the robot based on the gyro
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(pigeonSensor.getYaw() + 180);
  
    // navX: return Rotation2d.fromDegrees(-navX.getYaw() + 90);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  public WPI_Pigeon2 getPigeonSensor() {
    return pigeonSensor;
  }

  public AHRS getNavX() {
   return navX;
  }

  public SwerveDriveKinematics getKinematics() {
    return SwerveConstants.kinematics;
  }

//   public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
//     return new SequentialCommandGroup(
//          new InstantCommand(() -> {
//            // Reset odometry for the first path you run during auto
//            if(isFirstPath){
//                this.resetOdometry(traj.getInitialHolonomicPose());
//            }
//          }),
//          new PPSwerveControllerCommand(
//              traj, 
//              this::getPose, // Pose supplier
//              SwerveConstants.kinematics, // SwerveDriveKinematics
//              new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
//              new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
//              new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
//              this::setModuleStates, // Module states consumer
//              this // Requires this drive subsystem
//          )
//      );
//  }
  }

