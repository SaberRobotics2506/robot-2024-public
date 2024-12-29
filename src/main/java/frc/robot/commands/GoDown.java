package frc.robot.commands;
import frc.robot.Constants.ClimbConstants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;

public class GoDown extends Command{
  public final Climb climbSystem;
  public boolean finished = true;
 
  public GoDown(Climb climb){
      climbSystem = climb;
      addRequirements(climb);
    }
  @Override
  public void initialize() {
    climbSystem.setRightClimbDownReal(ClimbConstants.kRightClimbEncoderDownDistance);    
    climbSystem.setLeftClimbDownReal(ClimbConstants.kLeftClimbEncoderDownDistance);


    GlobalVariables.fieldRelative = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(climbSystem.getLeftAtSetpoint() || climbSystem.getLeftLimitSwitch()){
      climbSystem.calculatePidLeft(ClimbConstants.kLeftClimbEncoderDownDistance);
      climbSystem.stopLeftMotor();
    }else{
       climbSystem.setLeftClimbDownReal(ClimbConstants.kLeftClimbEncoderDownDistance);
    }
    if(climbSystem.getRightAtSetpoint() || climbSystem.getRightLimitSwitch()){
      climbSystem.calculatePidRight(ClimbConstants.kRightClimbEncoderDownDistance);
      climbSystem.stopRightMotor();
    }else{
       climbSystem.setRightClimbDownReal(ClimbConstants.kRightClimbEncoderDownDistance);
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
      return false;
  
  }
}