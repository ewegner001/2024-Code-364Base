package frc.robot.subsystems;


import frc.robot.Constants;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Eyes extends SubsystemBase {

    public PhotonCamera camera;
    public PhotonTrackedTarget target;
    public int targetID;
    public double poseAmbiguity;
    public PhotonPipelineResult result;
  
    public Eyes() {

        // photonvision camera object
        

    }

    public void initCamera() {
        camera = new PhotonCamera("more_camera");
    }

    public void look() {

        result = camera.getLatestResult();
        List<PhotonTrackedTarget> target = result.getTargets();
        
    }

    public void readAprilTag() {

        // identify april tag target
        // look();

        if (camera != null) {
            System.out.println(camera.isConnected());
        }
        // if target found
        // if (result.hasTargets()) {

        //     // read data from target
        //     targetID = target.getFiducialId();
        //     poseAmbiguity = target.getPoseAmbiguity();

        // }
        
    }


    @Override
    public void periodic(){

        readAprilTag();
        //SmartDashboard.putNumber("Target ID", targetID);

    }
}