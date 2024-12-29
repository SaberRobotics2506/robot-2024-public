package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LauncherConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;


public class Intake extends SubsystemBase{
    //intake motor    
    private WPI_TalonSRX shooterIntakeMotor;
    private CANSparkMax UTB_IntakeMotor;
    private CANSparkMax UTB_IntakeToShooter;
    private DigitalInput irSensor;
    DoublePublisher intakeMotorOutput;
    DoublePublisher shooterIntakeMotorOutput;

    public Intake( WPI_TalonSRX shooterIntakeMotor, CANSparkMax UTB_IntakeMotor, CANSparkMax UTB_IntakeToShooter, DigitalInput irSensor) {
      this.shooterIntakeMotor = shooterIntakeMotor;
      this.UTB_IntakeToShooter = UTB_IntakeToShooter;
      this.irSensor = irSensor;
      this.UTB_IntakeMotor = UTB_IntakeMotor;
      initializeIntake();

      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable("intake");

      intakeMotorOutput = table.getDoubleTopic("IntakeMotorOutput").publish();

      shooterIntakeMotorOutput = table.getDoubleTopic("Shooter Intake Motor Output").publish();
    }
    //configure motors to defualt and correct direcction
    public void initializeIntake(){
      // shooterIntakeMotor.configFactoryDefault();

      shooterIntakeMotor.setInverted(false);
      UTB_IntakeMotor.setInverted(false);
      UTB_IntakeToShooter.setInverted(false);

      UTB_IntakeMotor.setSmartCurrentLimit(UTBintakeCurrentLimit);
      UTB_IntakeToShooter.setSmartCurrentLimit(UTBintakeCurrentLimit);
    }

    //debug
    public void runUTB()
    {
      UTB_IntakeMotor.set(.5);
      UTB_IntakeToShooter.set(.5);
    }
    //picks up a game piece
    // Checks if the IR Sensor

    public boolean getIRsensor()
    {
      return irSensor.get();
    }
    public boolean pickup(){
        
        SmartDashboard.putNumber("Feeder motor output", shooterIntakeMotor.get());
        if(irSensor.get()){
            // SmartDashboard.putNumber("telling", kintakeSpeed );
            shooterIntakeMotor.set(ControlMode.PercentOutput, kintakeSpeed);
            UTB_IntakeMotor.set(kintakeSpeed);
            UTB_IntakeToShooter.set(kintakeSpeed);
            return true;
        }else{
            // SmartDashboard.putNumber("telling", 0 );
            shooterIntakeMotor.set(ControlMode.PercentOutput, kNoArmSpeed);
            UTB_IntakeMotor.stopMotor();
            UTB_IntakeToShooter.stopMotor();
            return false;
        }
        //m_motor2.set(ControlMode.PercentOutput, -.2);
    }

    public void noSpin(){
      shooterIntakeMotor.set(ControlMode.PercentOutput, kNoArmSpeed);
      // m_armMotor.set(kNoArmSpeed);
      UTB_IntakeMotor.stopMotor();
      UTB_IntakeToShooter.stopMotor();
    }
    
    public void shootIntake(double speed){
      shooterIntakeMotor.set(TalonSRXControlMode.PercentOutput,speed);
    }
    //spit out a game piece
    public void release(){
      SmartDashboard.putBoolean("IR Sensor Value", irSensor.get());
      UTB_IntakeMotor.set(-kintakeSpeed);
      shooterIntakeMotor.set(-kintakeSpeed);
      UTB_IntakeToShooter.set(-kintakeSpeed);
    }
  
  public void setUTBtoShooter(double speed)
  {
    UTB_IntakeToShooter.set(speed);
  }
  
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    intakeMotorOutput.set(shooterIntakeMotor.get());

    
    SmartDashboard.putBoolean("IR Sensor Value", irSensor.get());
    SmartDashboard.putNumber("Feeder Speed", shooterIntakeMotor.get());
  }
  
  public void stop() {
    shooterIntakeMotor.stopMotor();
    UTB_IntakeMotor.stopMotor();
    UTB_IntakeToShooter.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}