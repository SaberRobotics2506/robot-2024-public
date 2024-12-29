package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeNote extends Command{

    Intake intake;
    DigitalInput irSensor;
    //DigitalInput armLimitSwitch = new DigitalInput(0);

    public IntakeNote(Intake intake, DigitalInput irSensor){
        this.intake = intake;
        this.irSensor = irSensor;
        addRequirements(intake);
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
      intake.pickup();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      // Do nothing when the command ends. The launch wheel needs to keep spinning in order to launch
      intake.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      // Always return false so the command never ends on it's own. In this project we use a timeout
      // decorator on the command to end it.
      if(intake.pickup()){
        return false;
     }
     else{
        return true;
     }

    }
}

