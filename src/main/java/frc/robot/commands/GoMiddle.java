package frc.robot.commands;

import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;


public class GoMiddle extends Command{
    public final Climb climbSystem;
    
    public GoMiddle(Climb climb){
      climbSystem = climb;
      addRequirements(climb);
    }
  

  @Override
  public void initialize() {
    climbSystem.setRightClimbUp(ClimbConstants.kRightClimbEncoderMiddleDistance);
    climbSystem.setLeftClimbUp(ClimbConstants.kLeftClimbEncoderMiddleDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        
      if(climbSystem.getRightAtSetpoint() ){
        climbSystem.stopRightMotor();
      } else{
        climbSystem.setRightClimbUp(ClimbConstants.kRightClimbEncoderMiddleDistance);
      }
      if(climbSystem.getLeftAtSetpoint()){
        climbSystem.stopLeftMotor();
      }else{
         climbSystem.setLeftClimbUp(ClimbConstants.kLeftClimbEncoderMiddleDistance);
      }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSystem.stopRightMotor();
    climbSystem.stopLeftMotor();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(((climbSystem.getRightAtSetpoint()) &&
      (climbSystem.getLeftAtSetpoint()))){
       
      return true;
    } else{
      return false;
    } 
  }
}
