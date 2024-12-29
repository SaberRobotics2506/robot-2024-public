// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithJoysticks extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;

  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final BooleanSupplier relative;
  private final DoubleSupplier maxSpeed;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
  private PIDController turnPID;
  private double rotationDriver = 0.0;
  public DriveWithJoysticks(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier relative, DoubleSupplier maxSpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.relative = relative;
    this.maxSpeed = maxSpeed;
    this.xLimiter = new SlewRateLimiter(Constants.SwerveConstants.DRIVE_SLEWRATE);
    this.yLimiter = new SlewRateLimiter(Constants.SwerveConstants.DRIVE_SLEWRATE);
    this.turnLimiter = new SlewRateLimiter(Constants.SwerveConstants.DRIVE_TURN_SLEWRATE);

    turnPID  = new PIDController(Constants.SwerveConstants.TURN_DRIVER_KP, 0.005,.005 );
    
    turnPID.enableContinuousInput(-180,180);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Joy Y", translationX.getAsDouble());
    SmartDashboard.putNumber("Joy X", translationY.getAsDouble());
    SmartDashboard.putNumber("Joy Rot", rotation.getAsDouble());
    
    Rotation2d roboRot = poseEstimator.getPoseRotation();
    Rotation2d driftRot = poseEstimator.getDriftRotation();
    
    double transX = translationX.getAsDouble();
    double transY = translationY.getAsDouble();
    double rot = rotation.getAsDouble();

    if(PoseEstimator.onRed())
    {
      transX *= -1; 
      transY *= -1;
    }
    
    poseEstimator.addToCurrAngle(Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/GlobalVariables.maxTurnSpeed * MathUtil.applyDeadband(rotation.getAsDouble(),.2));
    rotationDriver = turnPID.calculate(driftRot.getDegrees(), poseEstimator.getCurrAngle().getDegrees());
    SmartDashboard.putNumber("currAngle", poseEstimator.getCurrAngle().getDegrees());
    SmartDashboard.putNumber("turnDriver", rotationDriver);
    SmartDashboard.putNumber("driftPose", driftRot.getDegrees());

    if(relative.getAsBoolean()){
        swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        modifyAxis(transX, maxSpeed.getAsDouble(), yLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(transY, maxSpeed.getAsDouble(), xLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        MathUtil.applyDeadband(rotationDriver, 0.25),
        roboRot));
    } else {
      swerveSubsystem.drive(new ChassisSpeeds(
      modifyAxis(transX, maxSpeed.getAsDouble(), yLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(transY, maxSpeed.getAsDouble(), xLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(rot, maxSpeed.getAsDouble(), turnLimiter) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    }

  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Applies various modifications to the input (also squares the joystick input)
   * @param value the input of the joystick
   * @param speedModifyer how much to slow the joystick down by
   * @param limiter how much to slow the inputs down by
   * @return the modified joystick values
   */
  private double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter){
    value = MathUtil.applyDeadband(value, 0.02);
    value = Math.copySign(value * value, value);
    value = value*speedModifyer;
    value = limiter.calculate(value);
    if(Math.abs(value)*Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND <= Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND*0.01){
      value = 0.0;
    }
    return value;
  }
}