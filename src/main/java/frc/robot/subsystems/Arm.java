package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LauncherConstants.*;

public class Arm extends SubsystemBase{
    private TalonFX m_armMotor;
    private TalonFX m_armMotor2;
    private DutyCycleEncoder arm_encoder;
    public DigitalInput bottomLimit = new DigitalInput(1);
    public DigitalInput topLimit = new DigitalInput(6);
    PIDController pid = new PIDController(10, 0, 0);
    DoublePublisher armMotorOutput;
    DoublePublisher armMotorPosition;
    DoublePublisher speedLog;
    DoublePublisher armEncoderAbsolutePosition;
    DoublePublisher armEncoderPositionOffset;
    double distance;
    double speed;
    boolean encoder_status = true;
    boolean failsafe = false;
    LED leds;
    public Arm(TalonFX m_armMotor, TalonFX m_armMotor2, DutyCycleEncoder m_encoder, LED leds) {
      this.m_armMotor = m_armMotor;
      this.m_armMotor2 = m_armMotor2;
      this.arm_encoder = m_encoder;
      this.leds = leds;
      initializeArm();

      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable("arm");

      armMotorOutput = table.getDoubleTopic("ArmMotorOutput").publish();
      armMotorPosition = table.getDoubleTopic("ArmMotorPosition").publish();
      speedLog = table.getDoubleTopic("ArmMotorPIDValue").publish();
      armEncoderAbsolutePosition = table.getDoubleTopic("ArmEncoderAbsolutePosition").publish();
      armEncoderPositionOffset = table.getDoubleTopic("ArmEncoderPositionOffset").publish();
    }
    //configure motors to defualt and correct direcction
    public void initializeArm(){
      
      var talonfxConfigs = new TalonFXConfiguration();

      talonfxConfigs.CurrentLimits.SupplyCurrentLimit = armContinuousCurrentLimit;
      talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = armEnableCurrentLimit;
      talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = armPeakCurrentLimit;
      talonfxConfigs.CurrentLimits.SupplyTimeThreshold = armPeakCurrentDuration;

      m_armMotor.getConfigurator().apply(talonfxConfigs);
      
      arm_encoder.setDistancePerRotation(10);
      if(bottomLimit.get()){
        arm_encoder.setPositionOffset(arm_encoder.getAbsolutePosition());
      }
      m_armMotor2.getConfigurator().apply(talonfxConfigs);
      m_armMotor2.setControl(new Follower(m_armMotor.getDeviceID(), true));
      m_armMotor.setNeutralMode(NeutralModeValue.Brake);
      m_armMotor2.setNeutralMode(NeutralModeValue.Brake);
      
      pid.setTolerance(0.0035);
      pid.setSetpoint(getDistance());
    }

    public boolean getBottomLimitSwitch(){
      if(bottomLimit.get()){
        return true;
      }
      return false;
    }
    
    // Moves the arm from up to down 
    // would need limit switches and encoders
  public void lowerArm(){
      setArm(kLowerArmSetpoint);
  }

  public void raiseArm(){
      setArm(kraiseArmSetPoint);
  }

  public void setArm(double setPoint){
    if(setPoint <= kraiseArmSetPoint && setPoint >= kLowerArmSetpoint && encoder_status){
      speed = MathUtil.clamp(pid.calculate(getDistance(), setPoint), -kpidClampOutput, kpidClampOutput);
      m_armMotor.set(speed);
      m_armMotor2.set(-speed);
    }
  }

    public void setArmNoSaftey(double setPoint){
      speed = MathUtil.clamp(pid.calculate(getDistance(), setPoint), -kpidClampOutput, kpidClampOutput);
      m_armMotor.set(speed);
      m_armMotor2.set(-speed);
  }
  public double getSpeed(){
    return speed;
  }

  public void stopArm(){
    m_armMotor.set(0);
    m_armMotor2.set(0);
  }

  public boolean getTopLimit()
  {
    return topLimit.get();
  }

  public double getDistance(){
    distance = arm_encoder.getAbsolutePosition() - arm_encoder.getPositionOffset();
    return distance;
  }
  
  public void centerArm(){
    setArm(kcenterArmSetPoint);
  }
  
  public boolean checkSetPoint(){
    if(pid.atSetpoint()){
      return true;
    }else{
      return false;
    }
  }

  public double getSetpoint(){
    return pid.getSetpoint();
  }

  public boolean stopAtLimitSwitch(){
    if(getSetpoint() > getDistance() && topLimit.get()){
        return true;
    }
    else if (getSetpoint() < getDistance() && bottomLimit.get()) {
      return true;
    }
    return false;
    
  }
  public void setEncoderFailsafe(){
    if(!bottomLimit.get()){
      failsafe = true;
      m_armMotor.set(-0.1);
      m_armMotor2.set(0.1);
    }else{
      m_armMotor.set(0);
      m_armMotor2.set(0);
      failsafe=false;
    }

  }
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    armMotorOutput.set(m_armMotor.getVelocity().refresh().getValue());
    armMotorPosition.set(getDistance());
    armEncoderAbsolutePosition.set(arm_encoder.getAbsolutePosition());
    armEncoderPositionOffset.set(arm_encoder.getPositionOffset());
    speedLog.set(speed);

    if(bottomLimit.get()){
      arm_encoder.setPositionOffset(arm_encoder.getAbsolutePosition());
      pid.setSetpoint(0);
      leds.bottomLimitDot = false;
    }else{
      leds.bottomLimitDot = true;
    }

    if(topLimit.get()){
      pid.setSetpoint(getDistance());
    }

    if(arm_encoder.getAbsolutePosition() == 0){
      encoder_status = false;
      setEncoderFailsafe();
    }else {
      encoder_status = true;
    }

    
    SmartDashboard.putBoolean("Failsafe Running", failsafe);
    SmartDashboard.putBoolean("Arm Encoder Status", encoder_status);
    SmartDashboard.putBoolean("Arm top limit switch", getTopLimit());
    SmartDashboard.putNumber("Arm Encoder Value (with offset)", getDistance());
    SmartDashboard.putBoolean("Bottom Limit Switch value", bottomLimit.get());
    SmartDashboard.putNumber("Arm Setpoint", getSetpoint());
    SmartDashboard.putBoolean("Check Arm Setpoint", checkSetPoint());
    SmartDashboard.putNumber("PID Speed", getSpeed());
    SmartDashboard.putNumber("Arm Abs Position", arm_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Offset Position", arm_encoder.getPositionOffset());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}