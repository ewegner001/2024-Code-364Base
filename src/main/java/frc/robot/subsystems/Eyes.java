/*
 * This subsytem controls robot vision. It uses the LimelightHelpers class
 * and contains methods that are used to track april tags and gather
 * positional data from them. This data is then used in the swerve
 * subsystem to update the robot's odometry using the pose estimator.
 * 
 * parameters:
 * none
 */


package frc.robot.subsystems;


import frc.robot.Constants;
import java.util.List;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Swerve;



public class Eyes extends SubsystemBase {

    // Swerve subsystem for pose estimator
    Swerve s_Swerve;

    // create objects and variables
    public LimelightHelpers limelight;
    public double tx;
    public double ty;
    public double ta;
    public double tID;
  
    // constuctor
    public Eyes(Swerve swerve) {

        s_Swerve = swerve;
    }

 
    /*
     * This method will gather all of the positional data of the limelight target.
     * 
     * parameters:
     * none
     * 
     * returns;
     * none
     */
    public void updateData() {

        /* 
         * get data from limelight target
         * tx = x position in degrees in limelight FOV
         * ty = y position in degrees in limelight FOV
         * ta = pitch in degrees in limelight FOV
         * tID = target ID number
         */
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        tID = LimelightHelpers.getFiducialID("");

        // log target data
        SmartDashboard.putNumber("AprilTagX", tx);
        SmartDashboard.putNumber("AprilTagY", ty);
        SmartDashboard.putNumber("AprilTagA", ta);
        SmartDashboard.putNumber("AprilTagID", tID);

    }

    /*
     * This method will wrap all target data into an array for easy access.
     * 
     * Array indexes:
     * [0] = x
     * [1] = y
     * [2] = a (pitch)
     * [3] = ID
     */
    public double[] getDataPackage() {

        double[] data = {
            tx,
            ty,
            ta,
            tID
        };

        return data;
    }

    /*
     * This method will return the pose of the robot based upon the pose of a detected apriltag
     * 
     * parameters:
     * none
     * 
     * returns:
     * robot pose      (Pose2d)
     */
    public Pose2d getRobotPose() {

        Pose2d pose;

        pose = LimelightHelpers.getBotPose2d_wpiBlue("");

        return pose;

    }
    

    /*
     * This method will return the known pose of the desired target.
     * 
     * parameters:
     * none
     * 
     * returns:
     * target pose      (Pose2d)
     */
    public Pose3d getTargetPose() {

        Pose3d pose;

        // if robot is on blue alliance
        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            // get pose of blue speaker
            pose = new Pose3d(Constants.Positions.speakerBlueX, Constants.Positions.speakerBlueY, 0, new Rotation3d(0,0,Constants.Positions.speakerBlueR));

        // if robot is on red alliance
        } else {

            // get pose of red speaker
            pose = new Pose3d(Constants.Positions.speakerRedX, Constants.Positions.speakerRedY, 0, new Rotation3d(0,0,Constants.Positions.speakerRedR));

        }
        
        return pose;

    }

    public double getTargetRotation() {

        Pose2d robotPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle + 180;
    }

    public double getDistanceFromTarget() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            double xDistanceToSpeaker = Constants.Positions.speakerBlueX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerBlueY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        } else {

            double xDistanceToSpeaker = Constants.Positions.speakerRedX - s_Swerve.m_poseEstimator.getEstimatedPosition().getX();
            double yDistanceToSpeaker = Constants.Positions.speakerRedY - s_Swerve.m_poseEstimator.getEstimatedPosition().getY();
            distance = Math.sqrt(Math.pow(xDistanceToSpeaker, 2) + Math.pow(yDistanceToSpeaker, 2));

        }

        return distance;

    }


    @Override
    public void periodic() {
        s_Swerve.m_poseEstimator.update(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());

        if (LimelightHelpers.getTV("") == true) {
            s_Swerve.m_poseEstimator.addVisionMeasurement(
                getRobotPose(), 
                Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("")/1000.0) - (LimelightHelpers.getLatency_Capture("")/1000.0)
            );
        }

        SmartDashboard.putNumber("Pose estimator rotations", s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Pose Estimator X", s_Swerve.m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pose Estimator Y", s_Swerve.m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("target X", getTargetPose().getX());
        SmartDashboard.putNumber("target Y", getTargetPose().getY());
        SmartDashboard.putNumber("Distance to Target", getDistanceFromTarget());

    }
}