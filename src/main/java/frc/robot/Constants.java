// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import javax.swing.text.html.HTMLDocument.BlockElement;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.BetterSwerveKinematics;

public class Constants {
    public static class OperatorConstants{
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    }

    public static final int DRIVETRAIN_PIGEON_ID = 25; // Pigeon ID
    //public static final int CANDLE_ID = 3;
    public static final String DRIVETRAIN_CANBUS = "Trex";
    public static final String NORMAL_CANBUS = "rio";



    public static final double DRIVE_SPEED = 0.6;
    public static final double BOOST_SPEED = 1.0;


    //public static final double PERCISION_SPEED = 1;
    
    // public static final Pose2d redAmpPose = new Pose2d(14.700758, 8.2042, new Rotation2d(Math.PI/2));
    public static final Pose2d blueAmpPose = new Pose2d(1.837, 7.77, new Rotation2d(Math.PI/2+ Math.PI));
    
    public static final Pose2d redSourcePose = new Pose2d(0.35, 0.88, new Rotation2d(Math.PI));
    public static final Pose2d blueSourcePose = new Pose2d(16.18, 0.88, new Rotation2d(Math.PI));


    public static final double aprilTagLeniancy = .1;
    public static final int BlinknPort = 0;

    public static class ClimbConstants{
        public static final int kLeftMotorID = 41;
        public static final int kRightMotorID = 42;

        public static final int climbContinuousCurrentLimit = 30;
        public static final boolean climbEnableCurrentLimit = true;
        public static final int climbPeakCurrentLimit = 40;
        public static final double climbPeakCurrentDuration = 0.01;


  
        public static final double kRightClimbEncoderUpDistance = 150;
        public static final double kLeftClimbEncoderUpDistance = -140;
        public static final double kRightClimbEncoderMiddleDistance = 75;
        public static final double kLeftClimbEncoderMiddleDistance = -70;
        public static final double kRightClimbEncoderDownDistance = 1;
        public static final double kLeftClimbEncoderDownDistance = -1;
        public static final double kClimbLeftSpeed = 0.3;
        public static final double kClimbRightSpeed = 0.3;
        public static final int kLeftLimitSwitchPort = 2;
        public static final int kRightLimitSwitchPort = 3;
  
        public static final int kLeftEncoderChannelA = 4;
        public static final int kLeftEncoderChannelB = 5;
        public static final int kRightEncoderChannelA = 6;
        public static final int kRightEncoderChannelB = 7;

        public static final double kClimbPIDError = 1;

        public static final double kGoDownSpeedRight = -0.5;
        public static final double kGoDownSpeedLeft = 0.5;

        public static final double kManualLeftSpeed = 0.5;
        public static final double kManualRightSpeed = -0.5;

  
        public static final double kUpError = 10.00;
  
        public static final int kRelayPort = 0;
  
        public static final double kTimeLimit = 10;
  
    }

    public static class LauncherConstants {
      // PWM ports/CAN IDs for motor controllers
      public static final int kFeederID = 54;
      public static final int kLauncherLeftID = 55;
      public static final int kLauncherRightID = 54;
      public static final int kirAmpSensorLeft = 7;
      public static final int kirAmpSensorRight = 8;
      
  
      // Current limit for launcher and feed wheels
      public static final int kLauncherCurrentLimit = 80;
      public static final int kFeedCurrentLimit = 80;
  
      // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
      // in reverse
      //public static final double kRaiseArmSpeed = -0.2;
      //public static final double kLowerArmSpeed = 0.2;

      public static final int launcherContinuousCurrentLimit = 30;
      public static final boolean launcherEnableCurrentLimit = true;
      public static final int launcherPeakCurrentLimit = 40;
      public static final double launcherPeakCurrentDuration = 0.01;
  
      public static final int armContinuousCurrentLimit = 35;
      public static final int armPeakCurrentLimit = 150;
      public static final double armPeakCurrentDuration = 0.1;
      public static final boolean armEnableCurrentLimit = true;
      
      public static final int UTBintakeCurrentLimit = 40;
      
      
      public static final double kNoArmSpeed = 0;
      public static final double kLauncherSpeed = 0.4;
      public static final double kNextToSpeakerSpeed = 0.8;
      public static final double kNextToSpeakerAngle = 25;
      public static final double kMiddleToSpeakerSpeed = 0.9;
      public static final double kMiddleToSpeakerAngle = 50;
      public static final double kLauncherSpeedRPM = 2500;
      public static final double kLimelightDistanceAngle = 40;
      public static final double kLimelightDistanceSpeed = 0.8;
      public static final double kLaunchFeederSpeed = 0.9;
      public static final double kIntakeLauncherSpeed = -1;
      public static final double kIntakeFeederSpeed = -.20;
      public static final double kraiseArmSetPoint = .16;
      public static final double kcenterArmSetPoint = 0.05;
      public static final double kAmpAngle = 90;
      public static final double kSpeakerAngle = 0;
      public static final double kpidClampOutput = 0.5;
      public static final double kLauncherDelay = 0.75;
      public static final double kintakeSpeed = 0.7;
      public static final double kLowerArmSetpoint = -0.02;
      public static final double kAmpSpeedRPM = 800;
      public static final double kAmpFeederSpeed = 0.9;

      // [x,y,rot, angle, launch]
      public static final double[] operatorSpeaker = {15.4, 5.5, 180, 0,4100, 2050};
      // public static final double[] rightSpeaker = {15.6, 4.4, -134, 0, 2500};
      // public static final double[] leftSpeaker = {15.2,6.7, 132.6, 10, 2500};
      // public static final double[] pole = {14.2,4.9, -166, 18, 3000};
      public static final double[] operatorAmp = {14.7,7.7,-90, 0,4000,2000};
      // public static double[] operatorTestSpeaker = {2.0,7,21, 16, 3000};
      public static double[] operatorTestSpeaker = {0,0,0, 20, 4000, 2500};

      // public static double[] testArray = {0,0,0, 20, 2500};

      // public static final double[] closeFrontSpeakerShot = {1.3,5.5, 0, 0, 2500};



      
      public static final double[] podiumSpeakerShot = {2.07,5.8,0, 13, 4100, 2050};

      public static final double[] midSpeakerShot = {2.576,5.43,0, 17.5, 4100, 2050  };
    //   public static final double[] BADmidSpeakerShot = {2.36,5.41,0, 17.5, 4100, 2050  };

      // public static final double[] BAD2midSpeakerShot = {3.12,6.1,0, 17.5, 4100, 2050};

    //   public static final double[] BADrightSpeakerShot = {2.21,6.69,29, 17.5, 4100, 2050};
        public static final double[] rightSpeakerShot = {2.576,6.63,25, 18, 4100, 2050  };

    //   public static final double[] BADleftSpeakerShot = {2.31,4.15,-29, 17.5, 4100, 2050};
        public static final double[] leftSpeakerShot = {2.56,4.12,-25, 18, 4100, 2050  };

      public static final double[] leftSubwooferShot = {1.22,4.71,-30, 0, 3000, 1750};

            public static final double[] rightSubwooferShot = {1.05,6.2,30, 0, 3000, 1750};


        public static final double[] boomShot = {9.6, 0.89, -38, 0,4100, 2050};







      public static final double[] farMidSpeakerShot = {4.96,5.31,0, 35, 4200, 2200};
    public static final double[] farRightSpeakerShot = {5.16,6.92,19, 34.5, 4350, 2350};

   
      // public static final double[] closeRightShotSpeaker = {.8,6.3,53,0,2500};
      public static final double[] leftSpeakerMidShot = {2.3,4.2,-24.1, 17.25, 4100, 2050};


      public static final double[] rightSpeakerMidShot = {2.2,6.6,26.5, 16.5, 3000,4000};
      // public static final double[] closeLeftShotSpeaker = {.7,4.4,-55,0,2500};
      // public static final double[] poleShot = {2.73,6.5,22.67,23,4000};
      // public static final double[] bluePoleShotBAD = {2.0,7,21, 16, 3000};
    
      public static final double[][] allTImesBlue = {
        rightSpeakerShot,
        leftSpeakerShot,
        midSpeakerShot,
    


        // farRightSpeakerShot,
        // Messes up angle
        // leftSubwooferShot,
        // rightSubwooferShot
        // farRightSpeakerShot, 
        // leftSpeakerMidShot,
        // rightSpeakerMidShot,
         // farMidSpeakerShot,
      };

      public static final ArrayList<Pose2d> allTimesBlueAsPoses = getInList(allTImesBlue);
      // public static final double[][] allTimesRed = redValues(allTImesBlue);

      public static final double kMaxArmAngleDegrees = 90;
      public static final double kLowArmAngleDegrees = 0;

      public static final double kDegreesTicksMultiplier = ( Constants.LauncherConstants.kraiseArmSetPoint/Constants.LauncherConstants.kMaxArmAngleDegrees);
      public static final int UTB_MotorID = 56;
      public static final int armMotor2ID = 52;
      public static final int IRsensorChannel = 4;
      public static final int armMotor1ID = 51;
      public static final int LauncherEncoderID = 5;
      public static final int UTB_MotorToShooterID = 57;
      public static int intakeMotorID = 53;
  
    }
    public static final class ModuleConstants {

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 80;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
    
        public static final int driveContinuousCurrentLimit = 35;
        public static final int driveStatorCurrentLimit = 80; 
        public static final boolean driveStatorCurrentLimitEnabled = true;
        public static final int drivePeakCurrentLimit = 150;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
    
        // Angle Motor PID Values
        public static final double angleKP = 50.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.5;
    
        // Drive Motor PID Values
        public static final double driveKP = 2.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
    
        // Drive Motor Characterization Values
        public static final double driveKS = 0.2;
        public static final double driveKV = 2.0;
    
        // Angle Encoder Invert
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;
    
        // Motor Inverts
        public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
    
        // Neutral Modes
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double driveGearRatio = (50.0/14.0)*(17.0/27.0)*(45.0/15.0); //6.75:1
        public static final double angleGearRatio = (32.0/15.0)*(60.0/10.0); //12.8:1
        public static final double rotationsPerMeter = driveGearRatio / wheelCircumference;
    }





    public static final class SwerveConstants {
        
        
        public static final double PID_LENIANCY = .1;// lower = slower
        public static final double DRIVE_SLEWRATE = 1; // lower = slower
        public static final double DRIVE_TURN_SLEWRATE = 100;// lower = slower
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
        public static final double WHEELBASE_METERS = Units.inchesToMeters(19.5);
  
        public static final double TURN_DRIVER_KP = .1;

        public static final double SWERVE_TURN_REDUCER = 150; // lower = faster
        public static final double SWERVE_TURN_REDUCER_SLOW = 300;

        public static final double MAX_VOLTAGE = 12.0;
  
        //public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)) * 0.10033 * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.0;
  
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);
        
        public static final PathConstraints pathfindingConstraints =new PathConstraints(
            MAX_VELOCITY_METERS_PER_SECOND, //Max velocity M/s
             5.0, //Max Acceleration M/s^2
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, //Max Angular Velocity
            Units.degreesToRadians(720)); //Max Angular Acceleration

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Front Left
            new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0), // Front Right
            new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Back Left
            new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)); // Back Right

        public static final BetterSwerveKinematics BETTER_KINEMATICS = new BetterSwerveKinematics(
            new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Front Left
            new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0), // Front Right
            new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0), // Back Left
            new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)); // Back Right
            
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1; // Front left module drive motor ID
            public static final int FRONT_LEFT_STEER_MOTOR = 11; // Front left module steer motor ID
            public static final int FRONT_LEFT_STEER_ENCODER = 21; // Front left steer encoder ID
            public static final double FRONT_LEFT_STEER_OFFSET = 0.071; // Front left steer offset//TODO: change
      
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 2; // Front right drive motor ID
            public static final int FRONT_RIGHT_STEER_MOTOR = 12; // Front right steer motor ID
            public static final int FRONT_RIGHT_STEER_ENCODER = 22; // Front right steer encoder ID
            public static final double FRONT_RIGHT_STEER_OFFSET = 0.106; // Front right steer offset//TODO: change
      
            public static final int BACK_LEFT_DRIVE_MOTOR = 4; // Back left drive motor ID
            public static final int BACK_LEFT_STEER_MOTOR = 14; // Back left steer motor ID
            public static final int BACK_LEFT_STEER_ENCODER = 24; // Back left steer encoder ID 
            public static final double BACK_LEFT_STEER_OFFSET = .106; // Back left steer offset//TODO: change
      
            public static final int BACK_RIGHT_DRIVE_MOTOR = 3; // Back right drive motor ID
            public static final int BACK_RIGHT_STEER_MOTOR = 13; // Back right steer motor ID
            public static final int BACK_RIGHT_STEER_ENCODER = 23; // Back right steer encoder ID
            public static final double BACK_RIGHT_STEER_OFFSET = 0.081; // Back right steer offset//TODO: change
            
    }
    
    public static ArrayList<Pose2d> getInList(double[][] poses)
    {
        ArrayList<Pose2d> poseInArrayList = new ArrayList<Pose2d>();
        for(int i = 0; i < poses.length; i++)
        {
            poseInArrayList.add(new Pose2d(poses[i][0], poses[i][1], Rotation2d.fromDegrees(poses[i][2])));
        }

        return poseInArrayList;
    }
}
