// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;

/** An example command that uses an example subsystem. */
public class AngleArm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new AngleArm.
   *
   * @param subsystem The subsystem used by this command.
   */
  Arm arm;
  private final double degrees;


  public AngleArm(Arm arm, double degrees) {
    this.degrees = degrees;
    this.arm = arm;
    addRequirements(arm);
  }
  
  public double calculateDegreesToTicks(){
    return degrees * Constants.LauncherConstants.kDegreesTicksMultiplier;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putString("setting", "setting");
  }

  // Called every tnime the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmNoSaftey(calculateDegreesToTicks());

    SmartDashboard.putNumber("Angle of Arm", degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
    SmartDashboard.putString("setting", "ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(arm.checkSetPoint() || arm.stopAtLimitSwitch()){
      return true;
    }
    return false;
  }
}
