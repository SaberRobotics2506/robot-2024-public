package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalVariables;

public class RampSpeedTrigger extends Command {

    DoubleSupplier triggerValue;
    double startSpeed;
    double startTurnSpeed;

    public RampSpeedTrigger(DoubleSupplier triggerValue) {
        this.triggerValue = triggerValue;
    }

    @Override
    public void initialize() {
        // Add initialization code here
        startSpeed = GlobalVariables.maxSpeed;
        startTurnSpeed = GlobalVariables.maxTurnSpeed;
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        GlobalVariables.maxSpeed = .2;
        GlobalVariables.maxTurnSpeed = 400;
    }

    /**
     * Called once the command ends or is interrupted.
     * @param boolean interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // Add code to run when the command ends or is interrupted
        GlobalVariables.maxSpeed = startSpeed;
        GlobalVariables.maxTurnSpeed = startTurnSpeed;
    }

    /**
     * Returns true when the command should end.
     * @return False
     */
    @Override
    public boolean isFinished() {
        // Add code to determine if the command should end
        return false;
    }
}