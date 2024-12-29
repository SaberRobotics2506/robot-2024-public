package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionHub extends SubsystemBase {
    private final PowerDistribution powerDistribution = new PowerDistribution();

    double totalPDHCurrent;
    DoublePublisher totalCurrent;

    double totalPDHVoltage;
    DoublePublisher totalVoltage;

    double leftClimb;
    DoublePublisher leftClimbCurrent;

    double rightClimb;
    DoublePublisher rightClimbCurrent;

    double leftArm;
    DoublePublisher leftArmCurrent;

    double rightArm;
    DoublePublisher rightArmCurrent;

    double utbIntake;
    DoublePublisher utbIntakeCurrent;

    double utbIntaketoShooter;
    DoublePublisher utbIntaketoShooterCurrent;

    double intake;
    DoublePublisher intakeCurrent;

    double leftShooter;
    DoublePublisher leftShooterCurrent;

    double rightShooter;
    DoublePublisher rightShooterCurrent;

    public PowerDistributionHub() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("PowerDistributionHub");
        
        totalCurrent = table.getDoubleTopic("Total PDH Current").publish();
        totalVoltage = table.getDoubleTopic("PDH Voltage").publish();
        leftClimbCurrent = table.getDoubleTopic("Left Climb Motor Current").publish();
        rightClimbCurrent = table.getDoubleTopic("Right Climb Motor Current").publish();
        leftArmCurrent = table.getDoubleTopic("Left Arm Motor Current").publish();
        rightArmCurrent = table.getDoubleTopic("Right Arm Motor Current").publish();
        utbIntakeCurrent = table.getDoubleTopic("URB Intake Motor Current").publish();
        utbIntaketoShooterCurrent = table.getDoubleTopic("UTB IntakeToShooter Motor Current").publish();
        intakeCurrent = table.getDoubleTopic("Intake Motor Current").publish();
        leftShooterCurrent = table.getDoubleTopic("Left Shooter Motor Current").publish();
        rightShooterCurrent = table.getDoubleTopic("Right Shooter Motor Current").publish();
    }

    public void periodic() {
        totalPDHCurrent = powerDistribution.getTotalCurrent();
        totalPDHVoltage = powerDistribution.getVoltage();
        leftClimb = powerDistribution.getCurrent(16);
        rightClimb = powerDistribution.getCurrent(7);
        leftArm = powerDistribution.getCurrent(17);
        rightArm = powerDistribution.getCurrent(15);
        utbIntake = powerDistribution.getCurrent(13);
        utbIntaketoShooter = powerDistribution.getCurrent(14);
        intake = powerDistribution.getCurrent(12);
        leftShooter = powerDistribution.getCurrent(5);
        rightShooter = powerDistribution.getCurrent(6);

        totalCurrent.set(totalPDHCurrent);
        totalVoltage.set(totalPDHVoltage);
        leftClimbCurrent.set(leftClimb);
        rightClimbCurrent.set(rightClimb);
        leftArmCurrent.set(leftArm);
        rightArmCurrent.set(rightArm);
        utbIntakeCurrent.set(utbIntake);
        utbIntaketoShooterCurrent.set(utbIntaketoShooter);
        intakeCurrent.set(intake);
        leftShooterCurrent.set(leftShooter);
        rightShooterCurrent.set(rightShooter);
    }
}
