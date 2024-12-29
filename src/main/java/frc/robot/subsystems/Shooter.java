// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.LauncherConstants.*;


public class Shooter extends SubsystemBase {
   TalonFX m_launchWheelLeft;
   TalonFX m_launchWheelRight;
   WPI_TalonSRX m_intakeMotor;

   DoublePublisher leftLaunchWheelOutput;
   DoublePublisher rightLaunchWheelOutput;
   final VelocityVoltage m_velocity = new VelocityVoltage(0);


  /** Creates a new Launcher. */
  public Shooter() {
    m_launchWheelLeft = new TalonFX(kLauncherLeftID);
    m_launchWheelRight = new TalonFX(kLauncherRightID);
    var talonfxConfigs = new TalonFXConfiguration();

    talonfxConfigs.CurrentLimits.SupplyCurrentLimit = launcherContinuousCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = launcherEnableCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = launcherPeakCurrentLimit;
    talonfxConfigs.CurrentLimits.SupplyTimeThreshold = launcherPeakCurrentDuration;

    m_launchWheelLeft.setNeutralMode(NeutralModeValue.Coast);
    m_launchWheelRight.setNeutralMode(NeutralModeValue.Coast);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.17;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.01;
    talonfxConfigs.withSlot0(slot0Configs);
    m_launchWheelLeft.getConfigurator().apply(talonfxConfigs, 0.050);
    m_launchWheelRight.getConfigurator().apply(talonfxConfigs, 0.050);

    m_launchWheelLeft.setInverted(true);
    m_launchWheelRight.setInverted(false);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("shooter");

    leftLaunchWheelOutput = table.getDoubleTopic("launchWheelOutput").publish();
    rightLaunchWheelOutput = table.getDoubleTopic("rightLaunchWheelOutput").publish();
  }

  public double getLeftVelocity()
  {
    return m_launchWheelLeft.getVelocity().getValueAsDouble()*60;
  }

  public double getRightVelocity()
  {
    return m_launchWheelRight.getVelocity().getValueAsDouble()*60;
  }

  public void setIndividualLaunchWheelsRPM(double Lrpm, double Rrpm)
  {
    m_launchWheelLeft.setControl(m_velocity.withVelocity(Lrpm/60));
    m_launchWheelRight.setControl(m_velocity.withVelocity(Rrpm/60));

    SmartDashboard.putNumber("left RPM", m_launchWheelLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("right RPM", m_launchWheelRight.getVelocity().getValueAsDouble());

    // double leftMotorRPM = m_launchWheelLeft.getVelocity().getValueAsDouble();
    // double rightMotorRPM = m_launchWheelRight.getVelocity().getValueAsDouble();
    // SmartDashboard.putNumber("left PID calc", leftShooterPID.calculate(leftMotorRPM, rpm));
    // setLaunchWheelsIndependent(leftShooterPID.calculate(leftMotorRPM, rpm),rightShooterPID.calculate(rightMotorRPM, rpm));
  }

  public void setIntake(double speed){
    m_intakeMotor.set(TalonSRXControlMode.PercentOutput,speed);
  }
  // public boolean getIrSensor() {
  //   SmartDashboard.putData(this);
  //   SmartDashboard.putData(ir);
  //   return ir.get();
    
  // }

  public void periodic() {
    leftLaunchWheelOutput.set(m_launchWheelLeft.get());
    rightLaunchWheelOutput.set(m_launchWheelRight.get());
    SmartDashboard.putData("Current Command for Shooter", this);
    SmartDashboard.putData(this);
  }
  
  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheelLeft.stopMotor();
    m_launchWheelRight.stopMotor();
    // getIrSensor();
    }
    @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // getIrSensor();
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  // public Command getIntakeCommand() {
  //   // The startEnd helper method takes a method to call when the command is initialized and one to
  //   // call when it ends
  //   return this.startEnd(
  //       // When the command is initialized, set the wheels to the intake speed values
  //       () -> {
  //         setLaunchWheels(kIntakeLauncherSpeed);
  //         getIrSensor();
  //       },
  //       // When the command stops, stop the wheels
  //       () -> {
  //         stop();
  //       });
  // }

    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
  // public void setLaunchWheels(double speed) {
  //   m_launchWheelLeft.set(speed);
  //   m_launchWheelRight.set(speed);
  //   SmartDashboard.putNumber("left RPM", m_launchWheelLeft.getVelocity().getValueAsDouble());
  // }
  // public void setLaunchWheelsIndependent(double speedLeft, double speedRight) {
  //   m_launchWheelLeft.set(speedLeft);
  //   m_launchWheelRight.set(speedRight);
  //   SmartDashboard.putNumber("left RPM", m_launchWheelLeft.getVelocity().getValueAsDouble());
  // }

  //   public void setLaunchWheelsRPM(double rpm)
  // {
  //   m_launchWheelLeft.setControl(m_velocity.withVelocity(rpm/60));
  //   m_launchWheelRight.setControl(m_velocity.withVelocity(rpm/60));
  //   // double leftMotorRPM = m_launchWheelLeft.getVelocity().getValueAsDouble();
  //   // double rightMotorRPM = m_launchWheelRight.getVelocity().getValueAsDouble();
  //   SmartDashboard.putNumber("left RPM", m_launchWheelLeft.getVelocity().getValueAsDouble());
  //   // SmartDashboard.putNumber("left PID calc", leftShooterPID.calculate(leftMotorRPM, rpm));

  //   // setLaunchWheelsIndependent(leftShooterPID.calculate(leftMotorRPM, rpm),rightShooterPID.calculate(rightMotorRPM, rpm));
  // }

  // public double getAvgVelocity()
  // {
  //   return (m_launchWheelLeft.getVelocity().getValueAsDouble()*60 +m_launchWheelRight.getVelocity().getValueAsDouble()*60)/2;
  // }
}

