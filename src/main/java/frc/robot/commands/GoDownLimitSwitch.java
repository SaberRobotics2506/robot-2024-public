package frc.robot.commands;

import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class GoDownLimitSwitch extends Command{
  public final Climb climbSystem;
 
  public GoDownLimitSwitch(Climb climb){
    
      climbSystem = climb;
      addRequirements(climb);
    }
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climbSystem.getRightLimitSwitch()){
    climbSystem.stopRightMotor();
    } else{
    climbSystem.setRightClimbDown(ClimbConstants.kGoDownSpeedRight);
    }
    if(climbSystem.getLeftLimitSwitch()){
    climbSystem.stopLeftMotor();
    } else{
    climbSystem.setLeftClimbDown(ClimbConstants.kGoDownSpeedLeft);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      climbSystem.stopRightMotor();
      climbSystem.stopLeftMotor();
      climbSystem.resetRightEncoder();
      climbSystem.resetLeftEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(climbSystem.getRightLimitSwitch() == true && climbSystem.getLeftLimitSwitch() == true){  
      return true;
    } else{
      return false;
    }
  }
}