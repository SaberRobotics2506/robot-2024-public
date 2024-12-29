package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class PigeonClimb extends Command {
     public final Climb climbSystem;

     public PigeonClimb(Climb climb){
        climbSystem = climb;
        addRequirements(climb);
    
      }
      @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        
   
       
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
    
  }

    
     

    

}
