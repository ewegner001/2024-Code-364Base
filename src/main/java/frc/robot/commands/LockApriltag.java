package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Eyes;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class LockApriltag extends Command {    
    
    private Eyes eyes;

    public LockApriltag(Eyes eyes) {

        this.eyes = eyes;
        addRequirements(eyes);

    }

    @Override
    public void initialize() {

        double[] aprilTagLock = eyes.getDataPackage();

        SmartDashboard.putNumber("AprilTagX", aprilTagLock[0]);
        SmartDashboard.putNumber("AprilTagY", aprilTagLock[1]);
        SmartDashboard.putNumber("AprilTagA", aprilTagLock[2]);
        
    }

    @Override
    public void execute() {

    
    }
}