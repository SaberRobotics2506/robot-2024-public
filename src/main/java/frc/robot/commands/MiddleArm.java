package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MiddleArm extends Command{

    Arm arm;

    public MiddleArm(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void initialize() {
      // Set launch wheel to speed, keep feed wheel at 0 to let launch wheel spin up.
 
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // There is nothing we need this command to do on each iteration. You could remove this method
      // and the default blank method
      // of the base class will run.
      arm.centerArm();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
      arm.stopArm();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if(arm.checkSetPoint()){
        return true;
      }
      return false;
    }
}