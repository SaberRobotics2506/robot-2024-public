// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.unusedcommands;

import static frc.robot.Constants.LauncherConstants.*;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// import frc.robot.subsystems.CANLauncher;

public class LaunchNote extends Command {
  Shooter m_launcher;
  Intake m_intake;
  Double m_launcherSpeed;
  Double m_feederSpeed;
  // CANLauncher m_launcher;

  /** Creates a new PrepareLaunch. */
  public LaunchNote(Shooter m_launcher, Intake m_intake, double m_launcherSpeed, double m_feederSpeed) {
    // save the launcher system internally
    this.m_launcher = m_launcher;
    this.m_intake = m_intake;
    this.m_launcherSpeed = m_launcherSpeed;
    this.m_feederSpeed = m_feederSpeed;

    // indicate that this command requires the launcher system
    addRequirements(m_launcher);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set launch wheel to speed, keep feed wheel at 0 to let launch wheel spin up.
    m_launcher.setIndividualLaunchWheelsRPM(m_launcherSpeed, m_launcherSpeed);
    m_intake.shootIntake(kintakeSpeed);
    m_intake.setUTBtoShooter(kintakeSpeed);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method
    // and the default blank method
    // of the base class will run.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
    m_launcher.stop();
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    // decorator on the command to end it.
    return false;
  }
}
