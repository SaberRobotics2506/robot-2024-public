// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.unusedcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


// import frc.robot.subsystems.CANLauncher;

public class PrepareLaunch extends Command {
  Shooter m_launcher;
  Double m_launcherSpeed;
  // Intake m_intake;
    
  // CANLauncher m_launcher;

  /** Creates a new PrepareLaunch. */
  public PrepareLaunch(Shooter m_launcher, double m_launcherSpeed) {
    // save the launcher system internally
    this.m_launcher = m_launcher;
    this.m_launcherSpeed = m_launcherSpeed;

    // indicate that this command requires the launcher system
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set launch wheel to speed, keep feed wheel at 0 to let launch wheel spin up.
    m_launcher.setIndividualLaunchWheelsRPM(m_launcherSpeed, m_launcherSpeed);
    // m_intake.shootIntake(kintakeSpeed);
     }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method
    // and the default blank method
    // of the base class will run.
    // m_launcher.setLaunchWheel(kLauncherSpeed);
    // m_launcher.setFeedWheel(kLaunchFeederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
    if(interrupted)
    {
      m_launcher.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use a timeout
    // decorator on the command to end it.
  //  if(irSensor.get() ){
  //          return false;
  //       }else{
  //          return true;
  //       }
  return m_launcher.getLeftVelocity() > m_launcherSpeed-2 && m_launcher.getRightVelocity() > m_launcherSpeed-2;
  }
}
