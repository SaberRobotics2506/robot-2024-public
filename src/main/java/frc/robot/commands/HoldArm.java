package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class  HoldArm extends Command {
    Arm arm;
    public HoldArm(Arm arm){
        this.arm = arm;
        addRequirements(arm);

    }

    @Override
    public void initialize() {
        
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setArm(arm.getSetpoint());
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
