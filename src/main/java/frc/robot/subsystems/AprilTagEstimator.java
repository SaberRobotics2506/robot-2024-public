package frc.robot.subsystems;

import frc.lib.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AprilTagEstimator extends SubsystemBase {
    
    private final PoseEstimator poseEstimator;
    private Pose3d limeLightPose;
    private Pose2d limelightPose2D;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tz = table.getEntry("tz");
    NetworkTableEntry ta = table.getEntry("ta");

    
    public AprilTagEstimator(SwerveSubsystem driveTrain) {
        this.poseEstimator = driveTrain.poseEstimator;
    }

    @Override
    public void periodic() {  
        SmartDashboard.putNumber("Fiducial ID", LimelightHelpers.getFiducialID("limelight"));
        SmartDashboard.putBoolean("Getting Target", LimelightHelpers.getTV("limelight"));
        limeLightPose = LimelightHelpers.getBotPose3d_wpiBlue("limelight");
        limelightPose2D = limeLightPose.toPose2d();
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        SmartDashboard.putNumber("LimelightX", limeLightPose.getX());
        SmartDashboard.putNumber("LimelightY", limeLightPose.getY());
        SmartDashboard.putNumber("LimelightZ", limeLightPose.getZ());
        SmartDashboard.putNumber("LimelightRot", limeLightPose.getRotation().getAngle());
        SmartDashboard.putNumber("LimelightArea", ta.getDouble(0));


        if(ta.getDouble(0) > GlobalVariables.limelightTolerance) {
            updatePoseEstimatorWithVisionBotPose(limelightPose2D, limelightMeasurement);
        }
        

      
    }

    public void updatePoseEstimatorWithVisionBotPose(Pose2d limelightPose, LimelightHelpers.PoseEstimate limelightMeasurement) {
        // invalid LL data
        if(limelightPose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(limelightPose.getTranslation());


        if (null != limelightPose && LimelightHelpers.getTV("limelight")) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (limelightMeasurement.tagCount >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            // else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
            else if (limelightMeasurement.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            // else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
            else if (limelightMeasurement.avgTagArea > 0.3 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds)));
            poseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - 
                 (LimelightHelpers.getLatency_Pipeline("limelight")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight")/1000.0));
        }
    }

  
   /** getCurrentPose - returns poseEstimator current pose
   * @return Pose2d
   */
    public Pose2d getCurrentPose() {
        return poseEstimator.getPose();
    }

 

}