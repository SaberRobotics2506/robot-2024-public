package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.networktables.NetworkTable;
import com.ctre.phoenix6.configs.TalonFXConfiguration;



public class Climb extends SubsystemBase{
    TalonFX m_rightMotor = new TalonFX(ClimbConstants.kRightMotorID);
    TalonFX m_leftMotor = new TalonFX(ClimbConstants.kLeftMotorID);
    double climbLeftPos;
    double climbRightPos;
    boolean leftLimitPressed;
    boolean rightLimitPressed;
    DoublePublisher leftClimbOutput;
    DoublePublisher leftClimbPosition;
    DoublePublisher rightClimbOutput;
    DoublePublisher rightClimbPosition;
    BooleanPublisher leftLimitSwitch;
    BooleanPublisher hangingRangeClimb;
    DoublePublisher leftcurrentLimiting;
    DoublePublisher rightcurrentLimiting;


    public double leftClimbSetpoint;
    public double rightClimbSetpoint;
    
    private final DigitalInput leftLimit = new DigitalInput(ClimbConstants.kLeftLimitSwitchPort);
    private final DigitalInput rightLimit = new DigitalInput(ClimbConstants.kRightLimitSwitchPort);
    private final PIDController pidLeft = new PIDController(1.1, 0, 0);
    private final PIDController pidRight = new PIDController(1.1, 0, 0);
    



    public Climb(){
      if(getLeftLimitSwitch()){
              resetLeftEncoder();
      }
      if(getRightLimitSwitch()){
              resetRightEncoder();
      }
      

       pidLeft.setTolerance(ClimbConstants.kClimbPIDError);
       pidRight.setTolerance(ClimbConstants.kClimbPIDError);


        var talonfxConfigs = new TalonFXConfiguration();
        talonfxConfigs.CurrentLimits.SupplyCurrentLimit = ClimbConstants.climbContinuousCurrentLimit;
        talonfxConfigs.CurrentLimits.SupplyCurrentLimitEnable = ClimbConstants.climbEnableCurrentLimit;
        talonfxConfigs.CurrentLimits.SupplyCurrentThreshold = ClimbConstants.climbPeakCurrentLimit;
        talonfxConfigs.CurrentLimits.SupplyTimeThreshold = ClimbConstants.climbPeakCurrentDuration;

        m_rightMotor.getConfigurator().apply(talonfxConfigs);
        m_leftMotor.getConfigurator().apply(talonfxConfigs);
        
        m_rightMotor.setInverted(false);
        m_leftMotor.setInverted(false);
        m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leftMotor.setNeutralMode(NeutralModeValue.Brake);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("climb");

        leftClimbOutput = table.getDoubleTopic("leftClimbOutput").publish();
        rightClimbOutput = table.getDoubleTopic("rightClimbOutput").publish();
        leftClimbPosition = table.getDoubleTopic("leftClimbPosition").publish();
        rightClimbPosition = table.getDoubleTopic("rightClimbPosition").publish();
        leftLimitSwitch = table.getBooleanTopic("Left Limit Switch").publish();
        hangingRangeClimb = table.getBooleanTopic("Hanging Range for climb").publish();
        leftcurrentLimiting = table.getDoubleTopic("left current Limit").publish();
        rightcurrentLimiting = table.getDoubleTopic("right current limit").publish();
    }

    
     @Override
    public void periodic() {
      climbLeftPos = getLeftPosition().getValueAsDouble();
      climbRightPos = getRightPosition().getValueAsDouble();
      leftLimitPressed = getLeftLimitSwitch();
      rightLimitPressed = getRightLimitSwitch();

      SmartDashboard.putNumber("Climb Encoder Value Left", climbLeftPos);
      SmartDashboard.putNumber("Climb Encoder Value Right", climbRightPos);
      SmartDashboard.putBoolean("Left Limit Switch", leftLimitPressed);
      SmartDashboard.putBoolean("Right Limit Switch", rightLimitPressed);
      leftClimbOutput.set(m_leftMotor.getDutyCycle().getValueAsDouble());
      rightClimbOutput.set(m_rightMotor.getDutyCycle().getValueAsDouble());
      hangingRangeClimb.set(!getLeftHangingLoose());
      leftcurrentLimiting.set(m_leftMotor.getSupplyCurrent().getValueAsDouble());
      rightcurrentLimiting.set(m_rightMotor.getSupplyCurrent().getValueAsDouble());

      if(leftLimitPressed && !(MathUtil.applyDeadband(climbLeftPos, 0.3) == 0)){
              resetLeftEncoder();
      }
      if(rightLimitPressed && !(MathUtil.applyDeadband(climbRightPos, 0.3) == 0)){
              resetRightEncoder();
      }
    }

    public void resetRightEncoder() {
      m_rightMotor.setPosition(0);

      }

    public void resetLeftEncoder(){
      m_leftMotor.setPosition(0);
    }

    public void setLeftClimbUp(double setpoint){
      leftClimbSetpoint = setpoint;

      pidLeft.setPID(1.1, 0, 0);

      double pidLeftValue = pidLeft.calculate(climbLeftPos, setpoint);

      m_leftMotor.set(pidLeftValue);
    }

    public void setRightClimbUp(double setpoint){
      rightClimbSetpoint = setpoint;

      pidRight.setPID(1.1, 0, 0);

      double pidRightValue = pidRight.calculate(climbRightPos, setpoint);
      
      m_rightMotor.set(pidRightValue);
    }

    public void setLeftClimbDown(double speed){
       
      m_leftMotor.set(speed);
    }

    public void setRightClimbDown(double speed){
      m_rightMotor.set(speed);

    }
    public void setLeftClimbDownReal(double setpoint) {
      leftClimbSetpoint = setpoint;

      pidLeft.setPID(0.4, 0, 0);

      m_leftMotor.set(pidLeft.calculate(climbLeftPos, setpoint));

    }

    public void calculatePidLeft ( double setpoint){
      pidLeft.calculate(climbLeftPos, setpoint);
    }
    
    public void calculatePidRight ( double setpoint){
      pidRight.calculate(climbLeftPos, setpoint);
    }

    public void setRightClimbDownReal(double setpoint) {
      rightClimbSetpoint = setpoint;
      
      pidRight.setPID(0.4, 0, 0);

      m_rightMotor.set(pidRight.calculate(climbRightPos, setpoint));

     }

    public boolean getLeftAtSetpoint(){
      return pidLeft.atSetpoint();
    }

    public boolean getRightAtSetpoint(){
      return pidRight.atSetpoint();    
    }
    public boolean getRightHangingLoose(){
      return Math.abs(climbRightPos) > 15;
    }
    public boolean getLeftHangingLoose(){
      return Math.abs(climbLeftPos) > 15;
    }
    
    public boolean getLeftLimitSwitch() {
      return !leftLimit.get();
    }

    public boolean getRightLimitSwitch() {
      return !rightLimit.get();
    }

    public StatusSignal<Double> getLeftPosition(){
      return (m_leftMotor.getPosition());
    }

    public StatusSignal<Double> getRightPosition(){
      return m_rightMotor.getPosition();
    }

    public double getLeftClimbSetpoint(){
      return leftClimbSetpoint;
    }

    public double getRightClimbSetpoint(){
      return rightClimbSetpoint;
    }

    public void stopLeftMotor(){
      m_leftMotor.set(0);
    }

    public void stopRightMotor(){
      m_rightMotor.set(0);
    }

    public void leftManualControl(double leftInput){
      if(leftLimitPressed && leftInput < 0) {
          m_leftMotor.set(0);
        }else if(climbLeftPos < ClimbConstants.kLeftClimbEncoderUpDistance && leftInput > 0) {
          m_leftMotor.set(0);
        }
        else {
          m_leftMotor.set(-leftInput/2);
        }
    }

    public void rightManualControl(double rightInput){
      if(rightLimitPressed && rightInput < 0) {
            m_rightMotor.set(0);
        }else if(climbRightPos > ClimbConstants.kRightClimbEncoderUpDistance && rightInput > 0) {
            m_rightMotor.set(0);
        }else {
            m_rightMotor.set(rightInput/2);
        }
    }

    public void rightPigeonClimb(){}

    public void leftPigeonClimb(){}
}