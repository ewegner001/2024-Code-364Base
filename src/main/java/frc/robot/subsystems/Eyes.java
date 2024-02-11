package frc.robot.subsystems;


import frc.robot.Constants;

import java.util.List;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class Eyes extends SubsystemBase {

    public LimelightHelpers limelight;
    public double tx;
    public double ty;
    public double ta;
    public double tID;
  
    public Eyes() {}


    public void updateData() {

        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        tID = LimelightHelpers.getFiducialID("");

        SmartDashboard.putNumber("AprilTagX", tx);
        SmartDashboard.putNumber("AprilTagY", ty);
        SmartDashboard.putNumber("AprilTagA", ta);
        SmartDashboard.putNumber("AprilTagID", tID);

    }

    public double[] getDataPackage() {

        double[] data = {
            tx,
            ty,
            ta,
            tID
        };

        return data;
    }

    @Override
    public void periodic(){
        updateData();
    }
}