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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;


public class Eyes extends SubsystemBase {

    // create objects and variables
    public LimelightHelpers limelight;
    public double tx;
    public double ty;
    public double ta;
    public double tID;
  
    // constuctor
    public Eyes() {}

 
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


    @Override
    public void periodic() {

    }
}