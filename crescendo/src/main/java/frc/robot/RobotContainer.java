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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static SwerveBase m_swerveBase = new SwerveBase();
  public static AprilTagSubsystem m_aprilTag = new AprilTagSubsystem();
  // do not uncomment public static ArmPivotSubsystem m_armPivotSubsystem = new ArmPivotSubsystem(m_swerveBase);
  public static IntakeSubsystem m_intake = new IntakeSubsystem();
  public static LineBreak m_lineBreak = new LineBreak();
  public ShooterFeederSubsystem m_shooterFeeder = new ShooterFeederSubsystem(this);
  public static ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static LEDSubsystem m_led = new LEDSubsystem();
  public static ArmPivotSubsystem m_ArmPivotSubsystem;
 private SendableChooser<String> mChooser;

  // public PowerDistributionPanel newPower = new PowerDistributionPanel(0);
  // public ClimberSubsystem m_climber = new ClimberSubsystem();
  // private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /* Human interfaces */
  private final Joystick driverJoystick;
  private final Joystick streamdeck;
  // private JoystickButton btn_arm_pivot_down;
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
  private JoystickButton btn_reset_yaw;
  private JoystickButton btn_aim_speaker;
  private JoystickButton btn_aim_amp;
  private JoystickButton btn_aim_human_feeder;
  private JoystickButton btn_led_win;
  private JoystickButton btn_shooter;
  private JoystickButton btn_defence;
  private JoystickButton btn_errected;
  private JoystickButton btn_line_scoring;
  private JoystickButton btn_cross_field;
  private JoystickButton btn_reverse_feeder;
  private JoystickButton btn_aim_line;
  private JoystickButton btn_driver_N1;
  private JoystickButton btn_trap_scoring;
  private JoystickButton btn_far_feeder;


  private JoystickButton btn_shooting_with_driver;  
  private JoystickButton btn_driver_fire;  


  private DoubleSupplier limit;
  private DoubleSupplier stopRotation;
  private BiFunction<Double, Double, Double> Clamp;
  
    private PIDController angleController;

  /* Subsystems */
  // to bring back arm pivot

  /* Parent Class */
  private final Robot m_robot;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot) {
    m_robot = robot;

    driverJoystick = new Joystick(0);
    streamdeck = new Joystick(1);

    limit = () -> 0.55 - 0.45 * driverJoystick.getRawAxis(SwerveConstants.sliderAxis);
    /* maps sliderAxis to be between 0.1 and 1.0 */
    stopRotation = () -> driverJoystick.getRawButton(9) ? 0.0 : 1.0; //Locks Rotation
    Clamp  = (val, lim) -> (Math.abs(val) < lim) ? val : Math.copySign(lim, val);
    // mChooser = new SendableChooser<>();
    // mChooser.setDefaultOption("Default Auto", "C-Shoot3-N2-Shoot6");
    // mChooser.addOption("6 Piece", "C-N3-Shoot6-N7-Shoot6");
    // mChooser.addOption("Test Aim", "AlignShooterTest");
    // mChooser.addOption("3 By 3", "3 By 3");
    // mChooser.addOption("Hellos", "Hellos");
    // mChooser.addOption("B-Shoot2-N2-Shoot5", "B-Shoot2-N2-Shoot5");
    // mChooser.addOption("New New Auto", "New New Auto");
    // SmartDashboard.putData("Auto Choices" ,  mChooser);

    // Controles rotaion Whne Auto Targeting
    angleController = new PIDController(1.0, 0.0, 0.0);//9   changed 3/21/2024 by cal ask wessly to make a sepret speed cosntant for this
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    // m_swerveBase = new SwerveBase();
    m_swerveBase.setDefaultCommand(
        new TeleopSwerve(
            m_swerveBase,
            () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
            () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
            () -> -Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.rotationAxis),
                limit.getAsDouble() * stopRotation.getAsDouble()),
            () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
            //Sandman 
        ));

    // to bring back arm pivot
    m_ArmPivotSubsystem = new ArmPivotSubsystem(m_swerveBase);

    // Configure the trigger bindings
    configureBindings();
  }

  public Joystick getJoystick() {
    return driverJoystick;
  }

  //
  public Joystick getstreamdeck() {
    return streamdeck;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  btn_reset_yaw = new JoystickButton(driverJoystick, 7);
    btn_reset_yaw.onTrue(new InstantCommand(() -> m_swerveBase.setNeedPigeonReset(true)));

    
    btn_shooter_feeder = new JoystickButton(driverJoystick, 11);
    btn_shooter_feeder.whileTrue(new ShooterFeederAMP(m_shooterFeeder));
   
    btn_driver_fire = new JoystickButton(driverJoystick, 12);
    btn_driver_fire.whileTrue(new ShooterFeederFire(m_shooterFeeder));

    // btn_driver_N1 = new JoystickButton(driverJoystick, 10);
    // btn_driver_N1.whileTrue(new ShootingWithoutCamerasN1(m_ArmPivotSubsystem));


    // btn_arm_pivot_up = new JoystickButton(driverJoystick, 3);
    // btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));

    // btn_aim_speaker = new JoystickButton(streamdeck, 1);
    // btn_aim_speaker.whileTrue(
    //     new TeleopSwerve(
    //         m_swerveBase,
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
    //         () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_swerveBase.getAngleToSpeaker().getRadians()),
    //         () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
    //     ));
    // btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));

    // btn_aim_speaker = new JoystickButton(streamdeck, 6);
    // btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));
   
   btn_aim_speaker = new JoystickButton(driverJoystick, 6);
    btn_aim_speaker.whileTrue(new ArmPivotShooting(m_ArmPivotSubsystem));
    btn_aim_speaker.whileTrue(
        new TeleopSwerve(
            m_swerveBase,
            () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
            () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
            () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_swerveBase.getAngleToSpeaker().getRadians()),
            () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
        ));

        // btn_aim_line = new JoystickButton(streamdeck, 1);
        // btn_aim_line.whileTrue(
        // new TeleopSwerve(
        //     m_swerveBase,
        //     () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
        //     () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
        //     () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_ArmPivotSubsystem.getLineAngle().getRadians()),
        //    () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
        // ));
   
    // btn_aim_amp = new JoystickButton(streamdeck, 10);
    // btn_aim_amp.whileTrue(
    //     new TeleopSwerve(
    //         m_swerveBase,
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
    //         () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
    //         () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_ArmPivotSubsystem.getAmpAngle().getRadians()),
    //         () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
    //     ));

    btn_aim_human_feeder = new JoystickButton(driverJoystick, 2);
    btn_aim_human_feeder.whileTrue(
        new TeleopSwerve(
            m_swerveBase,
            () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.translationAxis), limit.getAsDouble()),
            () -> Clamp.apply(driverJoystick.getRawAxis(SwerveConstants.strafeAxis), limit.getAsDouble()),
            () -> angleController.calculate(m_swerveBase.getPose().getRotation().getRadians(), m_ArmPivotSubsystem.getHumanFeederAngle().getRadians()),
           () -> !driverJoystick.getRawButton(1) // inverted=fieldCentric, non-inverted=RobotCentric
        ));
    
    // Intakes Note From Floor And Uses Line Breaks To Stop Note At Specific Position
    btn_floor_feeder = new JoystickButton(streamdeck, 5);
    btn_floor_feeder.whileTrue(new FloorFeederTest(m_intake));
    btn_floor_feeder.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
    btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));

    btn_reverse_feeder = new JoystickButton(streamdeck, 4);
    btn_reverse_feeder.whileTrue(new ReverseIntake(m_intake));
    btn_reverse_feeder.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
  


// // Note intake no robot centric
//     btn_floor_feeder = new JoystickButton(streamdeck, 15);
//     btn_floor_feeder.whileTrue(new FloorFeederTest(m_intake));
//     btn_floor_feeder.whileTrue(new ArmPivotErrected(m_ArmPivotSubsystem));
//     btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));
    // Turns on Shooter Wheels when held
    // btn_arm_pivot_up = new JoystickButton(streamdeck, 2);
    // btn_arm_pivot_up.whileTrue(new ShooterTest(m_robot.getShooterSubsystem()));

    // // Will Push The Note At Full Speed Into Shooter
    // btn_arm_pivot_up = new JoystickButton(driverJoystick, 1);
    // btn_arm_pivot_up.whileTrue(new ShooterFeederFire(m_shooterFeeder));

    btn_shooter = new JoystickButton(streamdeck, 6);
    btn_shooter.whileTrue(new ShooterTest(m_shooter));

    // Feeds Note At Slower Speed For AMP
    btn_shooter_feeder = new JoystickButton(streamdeck, 8);
    btn_shooter_feeder.whileTrue(new ShooterFeederAMP(m_shooterFeeder));

    // btn_led_win = new JoystickButton(streamdeck, 2);
    // btn_led_win.whileFalse(new LEDWinYes(m_led));
    // btn_led_win.whileTrue(new LEDWinNo(m_led));

    btn_far_feeder = new JoystickButton(streamdeck, 2);
    btn_far_feeder.whileTrue(new ArmPivotFarHumanFeeder(m_ArmPivotSubsystem));
    btn_far_feeder.whileTrue(new ShootingFarHumanFeeder(m_shooter));

    // Gets Note From Human Feeder And Uses Line Breaks To Stop Note At Specific Position
    btn_human_feeder = new JoystickButton(streamdeck, 9);
    btn_human_feeder.whileTrue(new ArmPivotHumanFeeder (m_ArmPivotSubsystem));
    btn_human_feeder.whileTrue(new ShooterFeederHuman(m_shooterFeeder));

    
    // Moves Arm Into Travel Position
    btn_store = new JoystickButton(streamdeck, 10);
    btn_store.whileTrue(new ArmPivotStore (m_ArmPivotSubsystem));

    // Shoots Note From Directly In Front Of Speaker
    btn_shooting_without_cameras = new JoystickButton(streamdeck, 3);
    btn_shooting_without_cameras.whileTrue(new ShootingWithoutCameras(m_ArmPivotSubsystem));
    btn_shooting_without_cameras.whileTrue(new ShooterTest(m_shooter));

    //Shoots Note Cross Field
    btn_cross_field = new JoystickButton(streamdeck, 11);
    btn_cross_field.whileTrue(new ShootingWithoutCameras(m_ArmPivotSubsystem));
    btn_cross_field.whileTrue(new ShooterCrossField(m_shooter));

    //Shoots Note Behind Black Line
    btn_line_scoring = new JoystickButton(streamdeck, 12);
    btn_line_scoring.whileTrue(new ArmPivotLineScoring(m_ArmPivotSubsystem));
    btn_line_scoring.whileTrue(new ShooterTest(m_shooter));

    btn_trap_scoring = new JoystickButton(streamdeck, 1);
    btn_trap_scoring.whileTrue(new ArmPivotTrap(m_ArmPivotSubsystem));
    btn_trap_scoring.whileTrue(new ShooterTrapScoring(m_shooter));

    // // Shoots Note From Front Leg Of Stage
    // btn_shooting_without_cameras_stage_leg = new JoystickButton(streamdeck,8);
    // // btn_shooting_without_cameras_stage_leg.whileFalse(new ArmPivotStore(m_robot.getArmPivotSubsystem()));
    // btn_shooting_without_cameras_stage_leg.whileTrue(new ShootingWithoutCamerasStageLeg(m_ArmPivotSubsystem));
    // btn_shooting_without_cameras_stage_leg.whileTrue(new ShooterTest(m_shooter));

    // // Shoots Note From Right Leg Of Stage
    // btn_shooting_without_cameras_2nd_stage_leg = new JoystickButton(streamdeck, 9);
    // // btn_shooting_without_cameras_2nd_stage_leg.whileFalse(newArmPivotStore(m_robot.getArmPivotSubsystem()));
    // btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShootingWithoutCameras2ndStageLeg(m_ArmPivotSubsystem));
    // btn_shooting_without_cameras_2nd_stage_leg.whileTrue(new ShooterTest(m_shooter));

    // Shoots Note To Go Into AMP
    btn_amp_scoring_pos = new JoystickButton(streamdeck, 7);
    btn_amp_scoring_pos.whileTrue(new AMPScoringPos(m_ArmPivotSubsystem));
    btn_amp_scoring_pos.whileTrue(new ShooterAMP(m_shooter));

    // btn_defence = new JoystickButton(streamdeck, 13);
    // btn_defence.whileTrue(new ShootingDefencePos(m_ArmPivotSubsystem));
    // btn_defence.whileTrue(new ShooterDefence(m_shooter));
    // btn_defence.whileTrue(new FloorFeederTest(m_intake));
    // btn_floor_feeder.whileTrue(new ShooterFeederPickUp(m_shooterFeeder));

    // Lets Pathplanner acsess commands 
    NamedCommands.registerCommand("Shoot", (new ArmPivotShooting(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Store", (new ArmPivotStore(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootFrontofSpeaker", (new ShootingWithoutCameras (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootN1", (new ShootingWithoutCamerasN1(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootLeg1", (new ShootingWithoutCamerasStageLeg (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("ShootLeg2", (new ShootingWithoutCameras2ndStageLeg (m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Errected", (new ArmPivotErrected(m_ArmPivotSubsystem)));
    NamedCommands.registerCommand("Shoot", (new ShooterTest (m_shooter)));
    NamedCommands.registerCommand("Feed", (new ShooterFeederFire(m_shooterFeeder)));
    NamedCommands.registerCommand("Intake", (new FloorFeederTest (m_intake)));
    NamedCommands.registerCommand("FeederIntake", (new ShooterFeederPickUp(m_shooterFeeder)));
    NamedCommands.registerCommand("Align", (new AlignPoseSpeaker(m_swerveBase)));
    NamedCommands.registerCommand("AlignShooter", (new ArmPivotShooting(m_ArmPivotSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return AutoBuilder.buildAuto("A-N1-N4");//"test"mChooser.getSelected()
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
