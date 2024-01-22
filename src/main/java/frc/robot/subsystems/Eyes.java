package frc.robot.subsystems;


import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Eyes extends SubsystemBase {

    public PhotonCamera camera;
    public PhotonTrackedTarget target;
    public int targetID;
    public double poseAmbiguity;
  
    public Eyes() {

        // photonvision camera object
        camera = new PhotonCamera("photonvision");

    }

    public void readAprilTag() {

        // identify april tag target
        var result = camera.getLatestResult();
        target = result.getBestTarget();

        // if target found
        if (result.hasTargets()) {

            System.out.println("Lock");

            // read data from target
            targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();

        } else {

            System.out.println("Blind");

        }
        
    }


    @Override
    public void periodic(){

    }
}