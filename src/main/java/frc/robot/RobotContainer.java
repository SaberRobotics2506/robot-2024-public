package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.unusedcommands.LaunchNote;
import frc.robot.unusedcommands.PrepareLaunch;

import static frc.robot.Constants.LauncherConstants.farRightSpeakerShot;
import static frc.robot.Constants.LauncherConstants.kirAmpSensorLeft;
import static frc.robot.Constants.LauncherConstants.kirAmpSensorRight;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.NamedCommands;



public class RobotContainer {
  
  
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);
  

  private final CANSparkMax UTB_IntakeMotor = new CANSparkMax(Constants.LauncherConstants.UTB_MotorID, MotorType.kBrushless);
  private final CANSparkMax UTB_IntakeToShooter = new CANSparkMax(Constants.LauncherConstants.UTB_MotorToShooterID, MotorType.kBrushed);
  private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(Constants.LauncherConstants.intakeMotorID);
  private final TalonFX m_armMotor = new TalonFX(Constants.LauncherConstants.armMotor1ID);
  private final TalonFX m_armMotor2 = new TalonFX(Constants.LauncherConstants.armMotor2ID);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(Constants.LauncherConstants.LauncherEncoderID);
  private final DigitalInput irSensor = new DigitalInput(Constants.LauncherConstants.IRsensorChannel);
  private final DigitalInput irAmpSensorRight = new DigitalInput(kirAmpSensorRight);
  private final Shooter m_launcher = new Shooter();
  private final Intake m_intake = new Intake(m_intakeMotor, UTB_IntakeMotor,UTB_IntakeToShooter, irSensor);
  LED leds = new LED(m_intake);
  private final Arm m_arm = new Arm(m_armMotor, m_armMotor2, m_encoder, leds);
  private final Climb m_climb = new Climb();
  private final Camera m_camera = new Camera(); //leave this in dont delete
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(m_launcher, m_intake, m_arm);
  AprilTagEstimator aprilTagEstimator = new AprilTagEstimator(swerveSubsystem);
  public final double[] testArray = {0,0,0,0,4100,2050};

    public RobotContainer()
  {
    //Auto Commands
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(m_intake, irSensor));
    NamedCommands.registerCommand("SpeakerShot", getAngleArmSpinShot(Constants.LauncherConstants.operatorSpeaker));
    NamedCommands.registerCommand("MidSpeakerShot", getAngleArmSpinShot(Constants.LauncherConstants.midSpeakerShot));
    NamedCommands.registerCommand("LeftMidSpeakerShot", getAngleArmSpinShot(Constants.LauncherConstants.leftSpeakerMidShot));
    NamedCommands.registerCommand("RightMidSpeakerShot", getAngleArmSpinShot(Constants.LauncherConstants.rightSpeakerMidShot));
    NamedCommands.registerCommand("PodiumSpeakerShot", getAngleArmSpinShot(Constants.LauncherConstants.podiumSpeakerShot));



    swerveSubsystem.poseEstimator.setCurrAngle(0);
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
      swerveSubsystem,
      swerveSubsystem.poseEstimator,
      () -> -driverController.getHID().getLeftY(),
      () -> -driverController.getHID().getLeftX(),
      () -> -driverController.getHID().getRightX(),
      () -> GlobalVariables.fieldRelative,
      () -> GlobalVariables.maxSpeed));

    m_arm.setDefaultCommand(new HoldArm(m_arm));
    m_climb.setDefaultCommand(new ManualClimb(
        m_climb,
        () -> operatorController.getHID().getLeftY(),
        () -> operatorController.getHID().getRightY()));
    configureBindings();  
  }


  /*
   * FULL BINDINGS!!!
   * 
   * DRIVER:
   
    left joystick - translation
    right joystick - rotation

    left trigger - quick slowdown
    right trigger - pathfind to speaker

    left bumper - pathfind to amp
    right bumper - pathfind to source

    A button - toggle slow mode
    X button - lower arm
    B button - raise arm
    y button - middle arm


    Start Button - hard reset robot pose (DONT USE DURING MATCH UNLESS NEEDED)
    Back Button - toggle field oriented driving


    D-pad down - shoot (hold to shoot)
    D-pad right - extake
    D-pad left - Intake


    OPERATOR:
      
    left bumper - climb down
    right bumper - climb up

    d-pad down - go down to limit switch
    d-pad left - go to the middle


      
   */



  private void configureBindings() {


    ///////SHOOTER/INTAKE///////SHOOTER/INTAKE//////////////////SHOOTER/INTAKE/////////////////////SHOOTER/INTAKE//////////////////////SHOOTER/INTAKE//////////
    /*BUTTONS USED:

    DRIVER:
    
    y button - shoot (hold to shoot)
    b button - extake
    x button - Intake


    OPERATOR:
    N/A

    */

   

    driverController.b().whileTrue(new ReleaseNote(m_intake));
    driverController.x().and(() -> m_arm.getBottomLimitSwitch()).toggleOnTrue(new IntakeNote(m_intake, irSensor));

    ///////SHOOTER/INTAKE///////SHOOTER/INTAKE//////////////////SHOOTER/INTAKE/////////////////////SHOOTER/INTAKE//////////////////////SHOOTER/INTAKE//////////


    /////ARM//////////ARM///////////////////ARM/////////////ARM///////////ARM//////////////ARM//////
    /*BUTTONS USED:

    DRIVER:
    
    D-Pad left - lower arm
    D-Pad right - raise arm
    D-pad up - middle arm


    OPERATOR:
    N/A

    */
    // driverController.povDown().onTrue(fastShooting(Constants.LauncherConstants.midSpeakerShot));
    driverController.y().onTrue(getAngleArmSpinShot(Constants.LauncherConstants.midSpeakerShot));
    driverController.povLeft().onTrue(new LowerArm(m_arm));
    driverController.povRight().onTrue(new RaiseArm(m_arm));
    //driverController.y().onTrue(new MiddleArm(m_intake));
    // driverController.povUp().onTrue(new AngleArm(m_intake,45));
    operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .1).onTrue(shootAmpNoIR());
    operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, .1).onTrue(getAngleArmSpinShot(Constants.LauncherConstants.operatorSpeaker));


    // Far Shot Only 
    operatorController.a().whileTrue(new DeferredCommand(() -> swerveSubsystem.FarFixedDistance(Constants.LauncherConstants.farRightSpeakerShot), swerveSubsystem.FarFixedDistance(Constants.LauncherConstants.farRightSpeakerShot).getRequirements()))
    .onFalse(new InstantCommand(() -> swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose()))
    .alongWith(new InstantCommand(() -> swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPose().getRotation().getDegrees()))));

    operatorController.b().whileTrue(new DeferredCommand(() -> swerveSubsystem.getAngleArmWithRamp(Constants.LauncherConstants.boomShot), swerveSubsystem.getAngleArmWithRamp(Constants.LauncherConstants.boomShot).getRequirements()))
    .onFalse(new InstantCommand(() -> swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose()))
    .alongWith(new InstantCommand(() -> swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPose().getRotation().getDegrees()))));

    operatorController.x().toggleOnTrue(getAngleArmSpinShot(testArray));

    /////ARM//////////ARM///////////////////ARM/////////////ARM///////////ARM//////////////ARM//////


    /////DRIVE//////////DRIVE///////////////////DRIVE/////////////DRIVE///////////DRIVE//////////////DRIVE//////
    /*BUTTONS USED:

    DRIVER:
    left joystick - translation
    right joystick - rotation

    left trigger - quick slowdown
    right trigger - pathfind to speaker

    left bumper - pathfind to amp
    right bumper - pathfind to source

    A button - toggle slow mode
    Start Button - hard reset robot pose (DONT USE DURING MATCH UNLESS NEEDED)
    Back Button - toggle field oriented driving

    OPERATOR:
    N/A

    */

    driveBindings();

    ///LED/////////////////////////////////////////
    driverController.rightBumper().onTrue(new InstantCommand(() -> leds.pathfinding = true)).onFalse(new InstantCommand(() -> leds.pathfinding = false));
    driverController.leftBumper().onTrue(new InstantCommand(() -> leds.pathfinding = true)).onFalse(new InstantCommand(() -> leds.pathfinding = false));
    operatorController.a().onTrue(new InstantCommand(() -> leds.operatorPathfinding = true)).onFalse(new InstantCommand(() -> leds.operatorPathfinding = false));
    operatorController.b().onTrue(new InstantCommand(() -> leds.boomPathfinding= true)).onFalse(new InstantCommand(() -> leds.boomPathfinding = false));

    ///////////////////////LED///////////////////////////////

      ////////CLIMB//////////////////////////CLIMB///////////CLIMB//////////////CLIMB//////////////////////////CLIMB//////
      /*BUTTONS USED:

      OPERATOR:
      
      left bumper - climb down
      right bumper - climb up

      d-pad down - go down to limit switch
      d-pad left - go to the middle

    */
    operatorController.leftBumper().toggleOnTrue(getGoDown(m_climb));
    operatorController.rightBumper().onTrue(getGoUp(m_climb));

  
      


  }

  public Command getGoUp(Climb climb){
    return new GoUp(climb);
  }
  public Command getGoDown(Climb climb){
    return new GoDown(climb);
  }
  public Command getGoDownTest(Climb climb){
    return new GoDownLimitSwitch(climb);
  }
  public Command getGoMiddle(Climb climb){
    return new GoMiddle(climb);
  }

  public Command getRotateLimelight(SwerveSubsystem swerveSubsystem) {
    return new LimelightRotate(m_intake, swerveSubsystem);
  }

   
    public Command getNothing(){
      return new SequentialCommandGroup();
    }

    public void driveBindings()
    {

      //TOGGLE FIELD ORIENTED DRIVING
      driverController.back().onTrue(new InstantCommand( () -> GlobalVariables.fieldRelative = !GlobalVariables.fieldRelative));
      driverController.back().onTrue(new InstantCommand(()->leds.robotOrient = !leds.robotOrient));
      //TOGGLE SLOW MODE
      driverController.a().onTrue(new InstantCommand(() -> swerveSubsystem.toggleSpeed()).alongWith(new InstantCommand(() -> swerveSubsystem.toggleDriveTurnSpeed())));


      /*
      This comment speaks for all 4 line chunks of commands.
      
      The actual command part is the "AutoBuilder.pathfindToPose" or in this case the "swerveSubsystem.ClosestFixedDistance()" The reason it
      is a deffered command is because the command changes ba
      */
      driverController.rightBumper().whileTrue(new DeferredCommand(() -> swerveSubsystem.ClosestFixedDistance(), swerveSubsystem.ClosestFixedDistance().getRequirements()))
      .onFalse(new InstantCommand(() -> swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose()))
      .alongWith(new InstantCommand(() -> swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPose().getRotation().getDegrees()))));



      //PATHFIND TO AMP
      driverController.leftBumper().whileTrue(getAmp())
      .onFalse(new InstantCommand(() -> swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose()))
      .alongWith(new InstantCommand(() -> swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPose().getRotation().getDegrees()))));

   

      

      //HARD RESET ROBOT POSE
      driverController.start().and(() -> PoseEstimator.onRed()).onTrue(new InstantCommand(() ->swerveSubsystem.poseEstimator.setPose(new Pose2d(5,5, new Rotation2d(0))), swerveSubsystem.poseEstimator)
      .alongWith(new InstantCommand(() ->swerveSubsystem.poseEstimator.setCurrAngle(0))
      .alongWith(new InstantCommand(() ->swerveSubsystem.poseEstimator.setDriftPose(new Pose2d(5,5, new Rotation2d(0)))))));
      driverController.start().and(() -> !PoseEstimator.onRed()).onTrue(new InstantCommand(() ->swerveSubsystem.poseEstimator.setPose(new Pose2d(5,5, new Rotation2d(Math.PI))), swerveSubsystem.poseEstimator)
      .alongWith(new InstantCommand(() ->swerveSubsystem.poseEstimator.setCurrAngle(180))
      .alongWith(new InstantCommand(() ->swerveSubsystem.poseEstimator.setDriftPose(new Pose2d(5,5, new Rotation2d(Math.PI)))))));

      //SLOWDOWN ON LEFT TRIGGER
      driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, .1).whileTrue(new RampSpeedTrigger(() -> driverController.getHID().getRawAxis(XboxController.Axis.kLeftTrigger.value)));

      driverController.povDown().onTrue(getLimelightArm());

      

    }


    //x,y,rot,armAngle,leftShooterRPM,rightShooterRPM
    public Command getAngleArmSpinShot(double[] values){
      double leftSpeed = values[4];
      double rightSpeed = values[5];

      return new SequentialCommandGroup(
        new ParallelCommandGroup(
          new AngleArm(m_arm, values[3]), 
          new PrepareSpinLaunch(m_launcher, leftSpeed, rightSpeed)
        ),
        new LaunchSpinShot(m_launcher, m_intake, leftSpeed, rightSpeed, Constants.LauncherConstants.kNextToSpeakerSpeed).withTimeout(0.75),
        new LowerArm(m_arm)
      );
    }
    public Command getLimelightArm(){
      return new SequentialCommandGroup(
        // new AngleArm(m_intake, swerveSubsystem.getFinalAngle())
        new LimelightRotate(m_intake, swerveSubsystem)
      );
    }

      public Command getAngleArmShoot(double[] values){
        return new SequentialCommandGroup(
        new ParallelCommandGroup(
        new AngleArm(m_arm, values[3]),         
        new PrepareLaunch(m_launcher, values[4])
        ),
        new LaunchNote(m_launcher, m_intake, values[4], Constants.LauncherConstants.kNextToSpeakerSpeed).withTimeout(1.5),
        new InstantCommand(() -> m_intake.noSpin()),
        new InstantCommand(() -> m_launcher.stop()),
        new LowerArm(m_arm)
      );
    }
    public Command setLeds(){
      return new InstantCommand(() -> leds.set(200, 200, 200)); // does only one periodic loop and doesnt work
    }
    public Command shootAmp(){
      if(irAmpSensorRight.get()){
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
            new AngleArm(m_arm, 100),
            new PrepareLaunch(m_launcher, 1000)
          ),
          new LaunchNote(m_launcher, m_intake, 1000, Constants.LauncherConstants.kNextToSpeakerSpeed).withTimeout(0.5),
          new InstantCommand(() -> m_intake.noSpin()),
          new InstantCommand(() -> m_launcher.stop()),
          new LowerArm(m_arm)
        );
      } else {
        return setLeds();
      }
    }
    public Command shootAmpNoIR(){
      // if(irAmpSensorLeft.get() == true || irAmpSensorRight.get() == true){
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
            new AngleArm(m_arm, 100),
            new PrepareLaunch(m_launcher, 1000)
          ),
          new LaunchNote(m_launcher, m_intake, 1000, Constants.LauncherConstants.kNextToSpeakerSpeed).withTimeout(0.5),
          new InstantCommand(() -> m_intake.noSpin()),
          new InstantCommand(() -> m_launcher.stop()),
          new LowerArm(m_arm)
        );
      }
    public Command getAmp(){
      if(!PoseEstimator.onRed()){
        return AutoBuilder.pathfindToPose(Constants.blueAmpPose, Constants.SwerveConstants.pathfindingConstraints)
        .andThen(new InstantCommand(() -> swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose()))
        .alongWith(new InstantCommand(() -> swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPose().getRotation().getDegrees())))
        .andThen(shootAmp()));
      }else{
        return AutoBuilder.pathfindToPoseFlipped(Constants.blueAmpPose, Constants.SwerveConstants.pathfindingConstraints)
        .andThen(new InstantCommand(() -> swerveSubsystem.poseEstimator.setDriftPose(swerveSubsystem.poseEstimator.getPose()))
        .alongWith(new InstantCommand(() -> swerveSubsystem.poseEstimator.setCurrAngle(swerveSubsystem.poseEstimator.getPose().getRotation().getDegrees())))
        .andThen(shootAmp()));
      }
    }
}
