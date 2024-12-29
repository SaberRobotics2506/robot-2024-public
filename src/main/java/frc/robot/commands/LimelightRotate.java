// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.SwerveConstants;

import static frc.robot.Constants.LauncherConstants.armPeakCurrentLimit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AprilTagEstimator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PoseEstimator;

/** An example command that uses an example subsystem. */
public class LimelightRotate extends Command {
  /**
   * Creates a new AngleArm.
   *
   * @param subsystem The subsystem used by this command.
   */
  Intake intake;
  SwerveSubsystem swerveSubsystem;
  double finalAngle;
  Rotation2d angleRotation;
  PIDController turnPID = new PIDController(Constants.SwerveConstants.TURN_DRIVER_KP, 0,.005 );

  public LimelightRotate(Intake intake, SwerveSubsystem swerveSubsystem ) {
    this.intake = intake;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    turnPID.enableContinuousInput(-180, 180);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distanceFromTagX;
    distanceFromTagX = swerveSubsystem.poseEstimator.getPoseX();
    if(PoseEstimator.onRed())
    {
      distanceFromTagX =  16.579 - swerveSubsystem.poseEstimator.getPoseX();
      distanceFromTagX *= -1;
    }
    
    double distanceFromTagY = swerveSubsystem.poseEstimator.getPoseY() - 5.5;
    finalAngle = Math.atan(distanceFromTagY/distanceFromTagX);
    angleRotation = new Rotation2d(finalAngle); 
    if(PoseEstimator.onRed())
    {
      angleRotation = angleRotation.rotateBy(Rotation2d.fromDegrees(180));
    }
    swerveSubsystem.poseEstimator.setCurrAngle(angleRotation.getDegrees());


    SmartDashboard.putNumber("FinalAngle", finalAngle);
    
  }

  // Called every tnime the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Roborot Radians", swerveSubsystem.poseEstimator.getPoseRotation().getRadians());
    Rotation2d roboRot = swerveSubsystem.poseEstimator.getPoseRotation();
    double rotationDriver =  turnPID.calculate(roboRot.getDegrees(), angleRotation.getDegrees());
    if(GlobalVariables.fieldRelative){
          swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
         0,
          0,
          rotationDriver,
          roboRot));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("setting", "ending");
    swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPoseRotation().getDegrees());
    swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose());
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(swerveSubsystem.poseEstimator.getCurrAngle().getDegrees()-finalAngle < 3){
    //   SmartDashboard.putBoolean("Rotation Running", false);
    //   return true;
    // }
    // SmartDashboard.putBoolean("Rotation Running", true);
    // return false;
    
    if(Math.abs(swerveSubsystem.poseEstimator.getPoseRotation().getRadians() - angleRotation.getRadians()) < Math.PI/180){
      return true;
    }
    return false;
  }
}