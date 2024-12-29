package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

import java.util.function.DoubleSupplier;

public class ManualClimb extends Command{
    public final Climb climbSystem;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightY;

    

    public ManualClimb(Climb climb, DoubleSupplier leftY, DoubleSupplier rightY){
      this.climbSystem = climb;
      this.rightY = rightY;
      this.leftY = leftY;
      addRequirements(climb);
    }

    @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    
    double leftInput = leftY.getAsDouble();
    double rightInput = rightY.getAsDouble();

    leftInput = applyDeadband(-leftInput);
    rightInput = applyDeadband(-rightInput);

    
    climbSystem.leftManualControl(leftInput);
    climbSystem.rightManualControl(rightInput);    
    
  }
  private double applyDeadband(double value) {
    final double deadband = 0.1; 
    if (Math.abs(value) > deadband) {
        return value;
    } else {
        return 0.0;
    }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    climbSystem.stopLeftMotor();
    climbSystem.stopRightMotor();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}