// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private  ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static SwerveBase m_swerveBase = new SwerveBase();
  public AprilTagSubsystem m_aprilTag = new AprilTagSubsystem();
  //public static ArmPivotSubsystem m_armPivot;
  public ClimberSubsystem m_climber = new ClimberSubsystem();
  public IntakeSubsystem m_intake = new IntakeSubsystem();
  public LineBreak m_lineBreak = new LineBreak();
  public ShooterFeederSubsystem m_shooterFeeder = new ShooterFeederSubsystem(this);
  public ShooterSubsystem m_shooter = new ShooterSubsystem();
  //public PowerDistributionPanel newPower = new PowerDistributionPanel(0);

  /* Human interfaces */
  private final Joystick driverJoystick;
  private final Joystick loupedeck;
  //private JoystickButton btn_arm_pivot_down;
  private JoystickButton btn_arm_pivot_up;
  private JoystickButton btn_shooter_feeder;
  private JoystickButton btn_floor_feeder;
  private JoystickButton btn_armP_pivot_stop;
 // private JoystickButton btn_amp_scoring_pos;
  private JoystickButton btn_shooting;
 private JoystickButton btn_human_feeder;
 private JoystickButton btn_store;
 private JoystickButton btn_shooting_without_cameras;
 private JoystickButton btn_shooting_without_cameras_stage_leg;
 private JoystickButton btn_shooting_without_cameras_2nd_stage_leg;
 private JoystickButton btn_amp_scoring_pos;
 
  /* Subsystems */
  private ArmPivotSubsystem m_ArmPivotSubsystem;

  /* Parent Class */
  private final Robot m_robot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {
    m_robot = robot;
    
    driverJoystick = new Joystick(0);
    loupedeck = new Joystick(1);

    DoubleSupplier limit = () -> 0.55 - 0.45*driverJoystick.getRawAxis(SwerveConstants.sliderAxis);
    /*maps sliderAxis to be between 0.1 and 1.0*/
    DoubleSupplier stopRotation = () -> driverJoystick.getRawButton(12) ? 0.0 : 1.0;
    /* clamps rotation to zero if button 12 is pressed */
    BiFunction<Double, Double, Double> Clamp = (val,lim) -> (Math.abs(val) < lim) ? val : Math.copySign(lim,val);

    

    //m_swerveBase = new SwerveBase();
    m_swerveBase.setDefaultCommand(
      new TeleopSwerve(
        m_swerveBase,
        () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
        () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
        () -> -Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.rotationAxis), limit.getAsDouble() * stopRotation.getAsDouble()),
        () -> !driverJoystick.getRawButton(1) //inverted=fieldCentric, non-inverted=RobotCentric
      )
    );
    

    m_ArmPivotSubsystem= new ArmPivotSubsystem(m_swerveBase);

    // Configure the trigger bindings
    configureBindings();
  }

  public Joystick getJoystick() {
    return driverJoystick;
  }

  public Joystick getLoupedeck() {
    return loupedeck;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // btn_arm_pivot_down = new JoystickButton(loupedeck, 11);
    // btn_arm_pivot_down.whileTrue(new ArmPivotDown(m_robot.getArmPivotSubsystem()));

    // btn_arm_pivot_up = new JoystickButton(loupedeck, 12);
    // btn_arm_pivot_up.whileTrue(new ArmPivotUp(m_robot.getArmPivotSubsystem()));

            //Intakes Note From Floor And Uses Line Breaks To Stop Note At Specific Position
    btn_floor_feeder = new JoystickButton(loupedeck, 1);
    //btn_floor_feeder.whileTrue(new FloorFeederTest(m_intake));
    //btn_floor_feeder.whileTrue(new ArmPivotErrected (m_ArmPivotSubsystem));
    btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));
    //btn_floor_feeder.whileFalse(new ArmPivotStore (m_robot.getArmPivotSubsystem()));

    //Turns on Shooter Wheels when held
    // btn_arm_pivot_up = new JoystickButton(loupedeck, 2);
    // btn_arm_pivot_up.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));

            //Will Push The Note At Full Speed Into Shooter
    btn_arm_pivot_up = new JoystickButton(loupedeck, 2);
    btn_arm_pivot_up.whileTrue(new ShooterFeederFire(m_shooterFeeder));
  
            //Feeds Note At Slower Speed For AMP 
    btn_shooter_feeder = new JoystickButton(loupedeck, 3);
    btn_shooter_feeder.whileTrue(new ShooterFeederAMP(m_shooterFeeder));

    //Emergency Stop For Pivot
    // btn_armP_pivot_stop = new JoystickButton(loupedeck, 4);
    // btn_armP_pivot_stop.toggleOnTrue(m_ArmPivotSubsystem.stopCommand());
    //Moves Pivit Based of of feild position
    
            //IDK
    btn_shooting = new JoystickButton(driverJoystick, 12);
    btn_shooting.whileTrue(new ArmPivotShooting (m_ArmPivotSubsystem));

            //Gets Note From Human Feeder And Uses Line Breaks To Stop Note At Specific Position
    btn_human_feeder = new JoystickButton(loupedeck, 4);
    btn_human_feeder.whileTrue(new ArmPivotHumanFeeder (m_ArmPivotSubsystem));
    btn_human_feeder.whileTrue(new ShooterFeederHuman(m_shooterFeeder));
    //btn_human_feeder.whileFalse(new ArmPivotStore (m_robot.getArmPivotSubsystem()));
    // btn_human_feeder.whileTrue(new ShooterFeederReverse(m_shooterFeeder));
    
            //Moves Arm Into Travel Position
    btn_store = new JoystickButton(loupedeck, 5);
    btn_store.whileTrue(new ArmPivotStore (m_ArmPivotSubsystem));

            //Moves Arm Into Starting Position
    btn_store = new JoystickButton(loupedeck, 6);
    btn_store.whileTrue(new ArmPivotErrected (m_ArmPivotSubsystem));
    
            //Shoots Note From Directly In Front Of Speaker
    btn_shooting_without_cameras = new JoystickButton(loupedeck, 7);
    //btn_shooting_without_cameras.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    btn_shooting_without_cameras.whileTrue(new ShootingWithoutCameras(m_ArmPivotSubsystem));
    btn_shooting_without_cameras.whileTrue(new ShooterTest(m_shooter));
            //Shoots Note From Front Leg Of Stage
    btn_shooting_without_cameras_stage_leg = new JoystickButton(loupedeck, 8);
    //btn_shooting_without_cameras_stage_leg.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    btn_shooting_without_cameras_stage_leg.whileTrue(new ShootingWithoutCamerasStageLeg(m_ArmPivotSubsystem));
    btn_shooting_without_cameras_stage_leg.whileTrue(new ShooterTest(m_shooter));

            ////Shoots Note From Right Leg Of Stage
    btn_shooting_without_cameras_2nd_stage_leg = new JoystickButton(driverJoystick, 11);
    //btn_shooting_without_cameras_2nd_stage_leg.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShootingWithoutCameras2ndStageLeg(m_ArmPivotSubsystem));
    btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShooterTest(m_shooter));

            //Shoots Note To Go Into AMP
    btn_amp_scoring_pos = new JoystickButton(loupedeck, 10);
    //btn_amp_scoring_pos.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    btn_amp_scoring_pos.whileTrue(new AMPScoringPos(m_ArmPivotSubsystem));
    btn_amp_scoring_pos.whileTrue(new ShooterAMP(m_shooter));

    //Lets Pathplanner acsess commands
    //NamedCommands.registerCommand("Shoot", (new ArmPivotShooting (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Store", (new ArmPivotStore (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootFrontofSpeaker", (new ShootingWithoutCameras (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootLeg1", (new ShootingWithoutCamerasStageLeg (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootLeg2", (new ShootingWithoutCameras2ndStageLeg (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Shoot", (new ShooterTest (m_shooter)));
    NamedCommands.registerCommand("Feed", (new ShooterFeederFire (m_shooterFeeder)));
    NamedCommands.registerCommand("Intake", (new FloorFeederTest (m_intake)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return AutoBuilder.buildAuto("New Auto");
  }

  public SwerveBase getSwerveSubsytem() {
    return m_swerveBase;
  }

  public LineBreak getLineBreakSubsystem() {
    return m_lineBreak;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return m_shooter;
  }
}
