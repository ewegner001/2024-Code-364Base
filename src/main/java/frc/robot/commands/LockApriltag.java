package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class LockApriltag extends Command {    
    
    private Vision vision;

    public LockApriltag(Vision vision) {

        this.vision = vision;
        addRequirements(vision);

    }

    @Override
    public void initialize() {

        double[] aprilTagLock = vision.getDataPackage();

        SmartDashboard.putNumber("AprilTagX", aprilTagLock[0]);
        SmartDashboard.putNumber("AprilTagY", aprilTagLock[1]);
        SmartDashboard.putNumber("AprilTagA", aprilTagLock[2]);
        
    }

    @Override
    public void execute() {

    
    }
}