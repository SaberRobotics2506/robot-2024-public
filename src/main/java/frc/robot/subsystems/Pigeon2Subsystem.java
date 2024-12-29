// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pigeon2Subsystem extends SubsystemBase {

  private final Pigeon2 pigeon2 = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, Constants.DRIVETRAIN_CANBUS);


  private StatusSignal<Double> yawPosition;
  private StatusSignal<Double> yawVelocity;
  private StatusSignal<Double> pitchPosition;
  private StatusSignal<Double> rollPosition;
  private BaseStatusSignal[] signals;

  public Pigeon2Subsystem() {
    pigeon2.getConfigurator().apply(new Pigeon2Configuration());
    var pigeon2Configs = new Pigeon2Configuration();
    pigeon2Configs.MountPose.MountPosePitch = 0.7112829089164734;
    pigeon2Configs.MountPose.MountPoseRoll = -0.9117752909660339;
    pigeon2Configs.MountPose.MountPoseYaw = -85.3637924194336;
    pigeon2.getConfigurator().apply(pigeon2Configs);

    yawPosition = pigeon2.getYaw();
    yawVelocity = pigeon2.getAngularVelocityZDevice();
    pitchPosition = pigeon2.getPitch();
    rollPosition = pigeon2.getRoll();

    pigeon2.setYaw(0);

    signals = new BaseStatusSignal[2];
    signals[0] = yawPosition;
    signals[1] = yawVelocity;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pitch", getPigeonPitch());
    SmartDashboard.putNumber("Roll", getPigeonRoll());
    SmartDashboard.putNumber("Yaw", getPigeonYaw(true));
  }

  /**
   * getGyroRotation - this is the Yaw value (rotate around...)
   * @param refresh Is new data needed or has the data already been updated this loop
   * @return Rotation2d
   */
  public Rotation2d getGyroRotation(boolean refresh) {
    if(refresh) {
      yawPosition.refresh();
      yawVelocity.refresh();
    }
    double yawRotation = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return Rotation2d.fromDegrees(yawRotation);
  }

  /**
   * getPigeonPitch - this is the pitch value (tilt up or down)
   * @return double
   */
  public double getPigeonPitch(){
    pitchPosition.refresh();
    return pitchPosition.getValue();
  }

  /**
   * getPigeonRoll - this is the roll value (tilt left or right)
   * @return double
   */
  public double getPigeonRoll(){
    rollPosition.refresh();
    return rollPosition.getValue();
  }

  /**
   * getPigeonYaw - this is the Yaw value (rotate around...)
   * @param refresh Is new data needed or has the data already been updated this loop
   * @return double representing angle in degrees
   */
  public double getPigeonYaw(boolean refresh){
    if(refresh){
      yawPosition.refresh();
      yawVelocity.refresh();
    }

    double yawRotation = BaseStatusSignal.getLatencyCompensatedValue(yawPosition, yawVelocity);
    return yawRotation;
  }

  /**
   * getSignals - signals for pigeon yaw and yaw angular velocity. This gives control of the updates and is used for synchronous updates
   * @return BaseStatusSignal[] of yaw and yaw angular velocity
   */
  public BaseStatusSignal[] getSignals() {
    return signals;
  }
}
