// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LauncherConstants.allTImesBlue;

// import static frc.robot.Constants.LauncherConstants.allTimes;

import com.ctre.phoenix6.BaseStatusSignal;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BetterSwerveKinematics;
import frc.lib.BetterSwerveModuleState;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.GlobalVariables;
import frc.robot.commands.*;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveModules;
  BaseStatusSignal[] signals;

  public DoublePublisher batteryVoltage;

  public DoublePublisher speed0;
  public DoublePublisher speed1;
  public DoublePublisher speed2;
  public DoublePublisher speed3;

  public Shooter mShooter;
  public Intake intake;
  public Arm arm;

  public DoublePublisher FL;
  public DoublePublisher FR;
  public DoublePublisher BL;
  public DoublePublisher BR;
  public DoublePublisher FLM;
  public DoublePublisher FRM;
  public DoublePublisher BLM;
  public DoublePublisher BRM;
  public int minDistance;
  
  public final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  public final PoseEstimator poseEstimator;
  public Pose2d closest = new Pose2d(5,5, new Rotation2d(0));


  public SwerveSubsystem(Shooter launcher, Intake intake, Arm arm) {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("RobotController");
    NetworkTable swerveTable = inst.getTable("swerve");

    batteryVoltage = table.getDoubleTopic("batteryVoltage").publish();

    
    this.mShooter = launcher;
    this.intake = intake;
    this.arm = arm;

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, new SwerveModuleConstants(
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_ENCODER, 
      SwerveConstants.FRONT_LEFT_STEER_OFFSET)),

      new SwerveModule(1, new SwerveModuleConstants(
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET)),

      new SwerveModule(2, new SwerveModuleConstants(
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_ENCODER, 
      SwerveConstants.BACK_LEFT_STEER_OFFSET)),

      new SwerveModule(3, new SwerveModuleConstants(
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET))

    };

    signals = new BaseStatusSignal[16];
    for(int i = 0; i<4; i++) {
      BaseStatusSignal[] tempSignals = swerveModules[i].getSignals();
      signals[i*4+0] = tempSignals[0];
      signals[i*4+1] = tempSignals[1];
      signals[i*4+2] = tempSignals[2];
      signals[i*4+3] = tempSignals[3];
    }
    
    poseEstimator = new PoseEstimator(this, pigeon2Subsystem);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      poseEstimator::getPose, // Robot pose supplier
      poseEstimator::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentChassisSpeedsEpic, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(Constants.ModuleConstants.driveKP, Constants.ModuleConstants.driveKI, Constants.ModuleConstants.driveKD), // DONT CHANGE THIS!!!
              new PIDConstants(5, Constants.ModuleConstants.angleKI, 0), // Rotation PID constants
              Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
              .325, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig(true,true,.05,.05) // Default path replanning config. See the API for the options here
      ),
      () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this // Reference to this subsystem to set requirements
    );

    speed0 = swerveTable.getDoubleTopic("0 Speed").publish();
    speed1 = swerveTable.getDoubleTopic("1 Speed").publish();
    speed2 = swerveTable.getDoubleTopic("2 Speed").publish();
    speed3 = swerveTable.getDoubleTopic("3 Speed").publish();

    FL = swerveTable.getDoubleTopic("FL").publish();
    FR = swerveTable.getDoubleTopic("FR").publish();
    BL = swerveTable.getDoubleTopic("BL").publish();
    BR = swerveTable.getDoubleTopic("BR").publish();

    FLM = swerveTable.getDoubleTopic("FLM").publish();
    FRM = swerveTable.getDoubleTopic("FRM").publish();
    BLM = swerveTable.getDoubleTopic("BLM").publish();
    BRM = swerveTable.getDoubleTopic("BRM").publish();

  }
  

  public Command ClosestFixedDistance(){
    
    int min = 0;

    double xPose = poseEstimator.getPoseX();
    double yPose = poseEstimator.getPoseY();
    Rotation2d rotPose = poseEstimator.getPoseRotation();

    if(PoseEstimator.onRed())
    {
      xPose = 16.579 - xPose;
    }

    Pose2d roboPose = new Pose2d(xPose,yPose, rotPose);
    Pose2d closestPose = roboPose.nearest(Constants.LauncherConstants.allTimesBlueAsPoses);

    if(!PoseEstimator.onRed())
    {
      return AutoBuilder.pathfindToPose(closestPose, Constants.SwerveConstants.pathfindingConstraints)
      .andThen(new InstantCommand(() -> this.poseEstimator.setDriftPose(this.poseEstimator.getPose()))
      .alongWith(new InstantCommand(() -> this.poseEstimator.setCurrAngle(this.poseEstimator.getPose().getRotation().getDegrees())))
      .andThen(getAngleArmSpinShot(allTImesBlue[min])));
    }
    else
    {
      return AutoBuilder.pathfindToPoseFlipped(closestPose, Constants.SwerveConstants.pathfindingConstraints)
      .andThen(new InstantCommand(() -> this.poseEstimator.setDriftPose(this.poseEstimator.getPose()))
      .alongWith(new InstantCommand(() -> this.poseEstimator.setCurrAngle(this.poseEstimator.getPose().getRotation().getDegrees())))
      .andThen(getAngleArmSpinShot(allTImesBlue[min])));
    }
    
  }

  
  public Command FarFixedDistance(double[] poses){
    
    Pose2d setPose = new Pose2d(poses[0], poses[1], Rotation2d.fromDegrees(poses[2]));
    
      if(!PoseEstimator.onRed())
      {
        return AutoBuilder.pathfindToPose(setPose, Constants.SwerveConstants.pathfindingConstraints)
        .andThen(new InstantCommand(() -> this.poseEstimator.setDriftPose(this.poseEstimator.getPose()))
        .alongWith(new InstantCommand(() -> this.poseEstimator.setCurrAngle(this.poseEstimator.getPose().getRotation().getDegrees())))
        .andThen(getAngleArmSpinShot(poses)));
      }
      else
      {
        return AutoBuilder.pathfindToPoseFlipped(setPose, Constants.SwerveConstants.pathfindingConstraints)
        .andThen(new InstantCommand(() -> this.poseEstimator.setDriftPose(this.poseEstimator.getPose()))
        .alongWith(new InstantCommand(() -> this.poseEstimator.setCurrAngle(this.poseEstimator.getPose().getRotation().getDegrees())))
        .andThen(getAngleArmSpinShot(poses)));
      }
  }


//  public Command pathfindAndShoot(double[] vals)
//  {
//      return AutoBuilder.pathfindToPoseFlipped(new Pose2d(vals[0],vals[1], new Rotation2d(Math.toRadians(vals[2]))), Constants.SwerveConstants.pathfindingConstraints)
//      .andThen(new InstantCommand(() -> this.poseEstimator.setDriftPose(this.poseEstimator.getPose()))
//      .alongWith(new InstantCommand(() -> this.poseEstimator.setCurrAngle(this.poseEstimator.getPose().getRotation().getDegrees())))
//      .andThen(getAngleArmShoot(vals)));
//  }
  public Command getAngleArmWithRamp(double[] poses){
    Pose2d setPose = new Pose2d(poses[0], poses[1], Rotation2d.fromDegrees(poses[2]));

    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(setPose, Constants.SwerveConstants.pathfindingConstraints),
        new InstantCommand(() -> this.poseEstimator.setDriftPose(this.poseEstimator.getPose())),
        new InstantCommand(() -> this.poseEstimator.setCurrAngle(this.poseEstimator.getPose().getRotation().getDegrees())),
        new AngleArm(arm, poses[3]),
        new PrepareSpinLaunch(mShooter, poses[4], poses[5])
      ),
        // new AngleArm(arm, poses[3]),
        // new PrepareSpinLaunch(mShooter, poses[4], poses[5]),
        new LaunchSpinShot(mShooter, intake, poses[4], poses[5], Constants.LauncherConstants.kNextToSpeakerSpeed).withTimeout(0.5),
        new LowerArm(arm) 
    );
  }
  
  public double getShootingValues(){
    return allTImesBlue[minDistance][3];
  }

  public void toggleDriveTurnSpeed()
  {
      if(GlobalVariables.maxTurnSpeed == Constants.SwerveConstants.SWERVE_TURN_REDUCER)
      {
        GlobalVariables.maxTurnSpeed = Constants.SwerveConstants.SWERVE_TURN_REDUCER_SLOW;
      }
      else
      {
        GlobalVariables.maxTurnSpeed = Constants.SwerveConstants.SWERVE_TURN_REDUCER;
      }
  }


  @Override
  public void periodic() {
    // SmartDashboard.putNumber("0 Speed", swerveModules[0].getMotorSpeed());
    // SmartDashboard.putNumber("1 Speed", swerveModules[1].getMotorSpeed());
    // SmartDashboard.putNumber("2 Speed", swerveModules[2].getMotorSpeed());
    // SmartDashboard.putNumber("3 Speed", swerveModules[3].getMotorSpeed());
    
    // SmartDashboard.putNumber("FL", swerveModules[0].getEncoderAngle().getRotations());
    // SmartDashboard.putNumber("FR", swerveModules[1].getEncoderAngle().getRotations());
    // SmartDashboard.putNumber("BL", swerveModules[2].getEncoderAngle().getRotations());
    // SmartDashboard.putNumber("BR", swerveModules[3].getEncoderAngle().getRotations());

    // SmartDashboard.putNumber("FLP", swerveModules[0].getPosition(true).distanceMeters);
    // SmartDashboard.putNumber("FRP", swerveModules[1].getPosition(true).distanceMeters);
    // SmartDashboard.putNumber("BLP", swerveModules[2].getPosition(true).distanceMeters);
    // SmartDashboard.putNumber("BRP", swerveModules[3].getPosition(true).distanceMeters);

    // SmartDashboard.putNumber("FLM", swerveModules[0].getMotorAngle().getRotations());
    // SmartDashboard.putNumber("FRM", swerveModules[1].getMotorAngle().getRotations());
    // SmartDashboard.putNumber("BLM", swerveModules[2].getMotorAngle().getRotations());
    // SmartDashboard.putNumber("BRM", swerveModules[3].getMotorAngle().getRotations());
    SmartDashboard.putNumber("Final Angle", getFinalAngle());
    batteryVoltage.set(RobotController.getBatteryVoltage());

    speed0.set(swerveModules[0].getMotorSpeed());
    speed1.set(swerveModules[1].getMotorSpeed());
    speed2.set(swerveModules[2].getMotorSpeed());
    speed3.set(swerveModules[3].getMotorSpeed());

    FL.set(swerveModules[0].getEncoderAngle().getRotations());
    FR.set(swerveModules[1].getEncoderAngle().getRotations());
    BL.set(swerveModules[2].getEncoderAngle().getRotations());
    BR.set(swerveModules[3].getEncoderAngle().getRotations());

    FLM.set(swerveModules[0].getEncoderAngle().getRotations());
    FRM.set(swerveModules[1].getEncoderAngle().getRotations());
    BLM.set(swerveModules[2].getEncoderAngle().getRotations());
    BRM.set(swerveModules[3].getEncoderAngle().getRotations());
  }

  /**
   * Main controlling method for driving swerve based on desired speed of drivetrian
   * @param chassisSpeeds Desired speed of drivetrain
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  /**
   * Main controlling method for driving swerve based on desired speed of drivetrian
   * @param chassisSpeeds Desired speed of drivetrain
   */
  public void betterDrive(ChassisSpeeds chassisSpeeds) {
    BetterSwerveModuleState[] states = SwerveConstants.BETTER_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    betterSetModuleStates(states);
  }

  /**
   * Set the desired state of all the modules
   * @param states Desired module states
   */
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0]);
    swerveModules[1].setDesiredState(states[1]);
    swerveModules[2].setDesiredState(states[2]); 
    swerveModules[3].setDesiredState(states[3]); 
  }

  public void toggleSpeed()
  {
    if(GlobalVariables.maxSpeed == Constants.BOOST_SPEED)
      {
        GlobalVariables.maxSpeed = Constants.DRIVE_SPEED;
      }
      else{
        GlobalVariables.maxSpeed = Constants.BOOST_SPEED;
      }
  }
  /**
   * shows what speed mode we're going
   * @return true if we're fast
   */
  public boolean getSpeedMode()
  {
    if(GlobalVariables.maxSpeed == Constants.BOOST_SPEED)
      {
        return true;
      }
    return false;
  }

  /**
   * Set the desired state of all the modules
   * @param states Desired module states
   */
  public void betterSetModuleStates(BetterSwerveModuleState[] states){
    BetterSwerveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0]);
    swerveModules[1].setDesiredState(states[1]);
    swerveModules[2].setDesiredState(states[2]); 
    swerveModules[3].setDesiredState(states[3]); 
  }

  /**
   * Stops all movement for swerve modules
   */
  public void stop(){
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  /**
   * Rotates modules in an X shape which makes it hard to push
   */
  public void lock(){
    swerveModules[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    swerveModules[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    swerveModules[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    swerveModules[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Get the position of all the modules including distance and angle of each
   * @return Position of all the modules as an array
   */
  public SwerveModulePosition[] getPositions(boolean refresh) {
    return new SwerveModulePosition[] {
      swerveModules[0].getPosition(refresh),
      swerveModules[1].getPosition(refresh),
      swerveModules[2].getPosition(refresh),
      swerveModules[3].getPosition(refresh)
    };
  }

  public BaseStatusSignal[] getSignals() {
    return signals;
  }

  /**
   * Get the state of all the modules including velocity and angle of each
   * @return The state of all the modules as an array
   */
  public SwerveModuleState[] getStates(boolean refresh) {
    return new SwerveModuleState[] {
      swerveModules[0].getState(refresh),
      swerveModules[1].getState(refresh),
      swerveModules[2].getState(refresh),
      swerveModules[3].getState(refresh)
    };
  }

  /**
   * @return the current speed of the robot in whatever direction it is traveling
   */
  public double getCurrentChassisSpeeds(boolean refresh) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(refresh));
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }
  public ChassisSpeeds getCurrentChassisSpeedsEpic()
  {
    return SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(true));
  }

  /**
   * @param currentPose of the robot from the pose estimator
   * @param refresh 
   * @return the current direction the robot is traveling in
   */
  public Rotation2d getCurrentChassisHeading(Pose2d currentPose, boolean refresh) {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates(refresh));
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(currentPose.getRotation());
    return currentHeading;
  }

  public Command getAngleArmSpinShot(double[] values){
      double leftSpeed = values[4];
      double rightSpeed = values[5];

      return new SequentialCommandGroup(
        new ParallelCommandGroup(
           new AngleArm(arm, values[3]), 
        // new WaitCommand(.5),
        new PrepareSpinLaunch(mShooter, leftSpeed, rightSpeed)
        ),
        new LaunchSpinShot(mShooter, intake, leftSpeed, rightSpeed, Constants.LauncherConstants.kNextToSpeakerSpeed).withTimeout(0.5),
        new InstantCommand(() -> intake.noSpin()),
        new InstantCommand(() -> mShooter.stop()),
        new LowerArm(arm)
        
      );
    }

  public double getFinalAngle() {
    double distanceFromTag = poseEstimator.getPoseX()+0.0381;
    double aprilTagHeight = 1.133602;
    double finalAngleDegree = Math.atan(aprilTagHeight/distanceFromTag);
    double finalAngle = Math.toDegrees(finalAngleDegree);
    return finalAngle;
  }
}
