// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

public class AprilTagSubsystem extends SubsystemBase {
  
  boolean sawTag = false;
  private OriginPosition originPosition;
  //TODO Tune
  PhotonRunnable photonEstimator1 = new PhotonRunnable("Camera1", VisionConstants.APRILTAG_CAMERA_TO_ROBOT_1);
  PhotonRunnable photonEstimator2 = new PhotonRunnable("Camera2", VisionConstants.APRILTAG_CAMERA_TO_ROBOT_2);
  PhotonRunnable photonEstimator3 = new PhotonRunnable("Camera3", VisionConstants.APRILTAG_CAMERA_TO_ROBOT_3);
  PhotonRunnable photonEstimator4 = new PhotonRunnable("Camera4", VisionConstants.APRILTAG_CAMERA_TO_ROBOT_4);

  EstimatedRobotPose visionPose1;
  EstimatedRobotPose visionPose2;
  EstimatedRobotPose visionPose3;
  EstimatedRobotPose visionPose4;
  StructPublisher<Pose3d> publisher;

  /** Creates a new ExampleSubsystem. */
  
  
  public AprilTagSubsystem() {
    originPosition = OriginPosition.kBlueAllianceWallRightSide;
    
    publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();
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

  private Pose2d flipAlliance(Pose2d poseToFlip){
    return poseToFlip.relativeTo(Constants.VisionConstants.FLIPPING_POSE);
  }

  public void setAlliance(Alliance alliance){
    boolean allianceChanged = false;
    switch (alliance) {
      case Blue:
      allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
        originPosition = OriginPosition.kBlueAllianceWallRightSide;
        break;
        case Red:
        allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
        originPosition = OriginPosition.kRedAllianceWallRightSide;
        break;
      default:
        break;
      }

    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(Robot.m_robotContainer.m_swerveBase.getOdometry().getEstimatedPosition());
      Robot.m_robotContainer.m_swerveBase.getOdometry().resetPosition(Robot.m_robotContainer.m_swerveBase.getOdometry().getEstimatedPosition().getRotation(), Robot.m_robotContainer.m_swerveBase.getModulePositions(), newPose);
    }
  }

  
  //Runs only on Tele because it is call to run peridical durning in in robot.Java
  public void updatedPoseFromTag(){
    photonEstimator1.run();
    photonEstimator2.run();
    photonEstimator3.run();
    photonEstimator4.run();
    
    // Pose3d cam1Pose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    
    // double lowestAmbiguity = Math.min(photonEstimator1.photonResults.getBestTarget().getPoseAmbiguity() , photonEstimator2.grabLatestEstimatedPose().)
      visionPose1 = photonEstimator1.grabLatestEstimatedPose();
      visionPose2 = photonEstimator2.grabLatestEstimatedPose();
      visionPose3 = photonEstimator3.grabLatestEstimatedPose();
      visionPose4 = photonEstimator4.grabLatestEstimatedPose();
 
     //System.out.println("Camera5 Vision P+ose:" + visionPose5);
     
    if (visionPose1 != null) {
      // New pose from vision
      sawTag = true;
      Pose2d pose2d1 = visionPose1.estimatedPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d1 = flipAlliance(pose2d1);
      }
      Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d1, visionPose1.timestampSeconds);
    }
    if (visionPose2 != null) {
      // New pose from vision
      sawTag = true;
      Pose2d pose2d2 = visionPose2.estimatedPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d2 = flipAlliance(pose2d2);
      }
      Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d2, visionPose2.timestampSeconds);
    }
    if (visionPose3 != null) {
      // New pose from vision
      sawTag = true;
      Pose2d pose2d3 = visionPose3.estimatedPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d3 = flipAlliance(pose2d3);
      }
      Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d3, visionPose3.timestampSeconds);
    }
    if (visionPose4 != null) {
      // New pose from vision
      sawTag = true;
      Pose2d pose2d4 = visionPose4.estimatedPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d4 = flipAlliance(pose2d4);
      }
      Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d4, visionPose4.timestampSeconds);
    }
    
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-tags");
    if(limelightMeasurement.tagCount >= 1){
          Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
    // if (visionPose5 != null) {
    //   // New pose from vision
    //   sawTag = true;
    //   Pose2d pose2d5 = visionPose5.estimatedPose.toPose2d();
    //   if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
    //     pose2d5 = flipAlliance(pose2d5);
    //   }
    //   Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d5, visionPose5.timestampSeconds);
    // }
    // This method will be called once per scheduler run
    // System.out.println("Pose: " + Robot.m_robotContainer.m_swerveBase.getPose3d());
    publisher.set(Robot.m_robotContainer.m_swerveBase.getPose3d());
  }









//Runs all the time (Both auto and teleOp)
  @Override
  public void periodic() {  
    

    photonEstimator1.run();
    //photonEstimator2.run();
    //photonEstimator3.run();
    photonEstimator4.run();
    
    // Pose3d cam1Pose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    
    // double lowestAmbiguity = Math.min(photonEstimator1.photonResults.getBestTarget().getPoseAmbiguity() , photonEstimator2.grabLatestEstimatedPose().)
      visionPose1 = photonEstimator1.grabLatestEstimatedPose();
      //visionPose2 = photonEstimator2.grabLatestEstimatedPose();
      //visionPose3 = photonEstimator3.grabLatestEstimatedPose();
      visionPose4 = photonEstimator4.grabLatestEstimatedPose();
 
     //System.out.println("Camera5 Vision P+ose:" + visionPose5);
     
    if (visionPose1 != null) {
      // New pose from vision
      sawTag = true;
      Pose2d pose2d1 = visionPose1.estimatedPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d1 = flipAlliance(pose2d1);
      }
      Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d1, visionPose1.timestampSeconds);
    }
    // if (visionPose2 != null) {
    //   // New pose from vision
    //   sawTag = true;
    //   Pose2d pose2d2 = visionPose2.estimatedPose.toPose2d();
    //   if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
    //     pose2d2 = flipAlliance(pose2d2);
    //   }
    //   Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d2, visionPose2.timestampSeconds);
    // }
    // if (visionPose3 != null) {
    //   // New pose from vision
    //   sawTag = true;
    //   Pose2d pose2d3 = visionPose3.estimatedPose.toPose2d();
    //   if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
    //     pose2d3 = flipAlliance(pose2d3);
    //   }
    //   Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d3, visionPose3.timestampSeconds);
    // }
    if (visionPose4 != null) {
      // New pose from vision
      sawTag = true;
      Pose2d pose2d4 = visionPose4.estimatedPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d4 = flipAlliance(pose2d4);
      }
      Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d4, visionPose4.timestampSeconds);
    }
    
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-tags");
    if(limelightMeasurement.tagCount >= 1){
          Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
        }
    // if (visionPose5 != null) {
    //   // New pose from vision
    //   sawTag = true;
    //   Pose2d pose2d5 = visionPose5.estimatedPose.toPose2d();
    //   if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
    //     pose2d5 = flipAlliance(pose2d5);
    //   }
    //   Robot.m_robotContainer.m_swerveBase.getOdometry().addVisionMeasurement(pose2d5, visionPose5.timestampSeconds);
    // }
    // This method will be called once per scheduler run
    // System.out.println("Pose: " + Robot.m_robotContainer.m_swerveBase.getPose3d());
    publisher.set(Robot.m_robotContainer.m_swerveBase.getPose3d());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
