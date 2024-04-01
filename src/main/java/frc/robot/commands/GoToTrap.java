package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Eyes;

public class GoToTrap extends Command {
    
    Eyes s_Eyes;
    Swerve s_Swerve;

    private StructPublisher<Translation2d> translationPublisher;

    public double robotX = 0.0;
    public double robotY = 0.0;
    public double robotR = 0.0;

    public double targetX = 0.0;
    public double targetY = 0.0;
    public double targetR = 0.0;

    public double positionTolerance = 0.1;
    public double rotationTolerance =  2.0;
    public double distanceLimit = 3.0;
    public double closestDistance = distanceLimit; 

    // blue trap positions
    public double blueTrapLeftX = 4.3;
    public double blueTrapLeftY = 5.0;
    public double blueTrapLeftR = -60.0;
    public double blueTrapLeftDistance = 0.0; // Leave as 0.0

    public double blueTrapRightX = 4.3;
    public double blueTrapRightY = 3.0;
    public double blueTrapRightR = 60.0;
    public double blueTrapRightDistance = 0.0; // Leave as 0.0

    public double blueTrapCenterX = 6.1;
    public double blueTrapCenterY = 4.1;
    public double blueTrapCenterR = 180.0;
    public double blueTrapCenterDistance = 0.0; // Leave as 0.0

    // red trap positions
    public double redTrapLeftX = 12.3;
    public double redTrapLeftY = 3.0;
    public double redTrapLeftR = 121.64;
    public double redTrapLeftDistance = 0.0; // Leave as 0.0

    public double redTrapRightX = 12.3;
    public double redTrapRightY = 5.0;
    public double redTrapRightR = -121.64;
    public double redTrapRightDistance = 0.0; // Leave as 0.0

    public double redTrapCenterX = 10.4;
    public double redTrapCenterY = 4.1;
    public double redTrapCenterR = 0.0;
    public double redTrapCenterDistance = 0.0; // Leave as 0.0

    public GoToTrap(Eyes s_Eyes, Swerve s_Swerve) {

        this.s_Eyes = s_Eyes;
        this.s_Swerve = s_Swerve;

        addRequirements(s_Eyes, s_Swerve);
    }

    @Override
    public void initialize() {

        translationPublisher = NetworkTableInstance.getDefault().getStructTopic("/Closest Trap", Translation2d.struct).publish();

        robotX = s_Eyes.getRobotPose().getX();
        robotY = s_Eyes.getRobotPose().getY();
        robotR = s_Eyes.getRobotPose().getRotation().getDegrees();

        blueTrapLeftDistance = s_Eyes.getDistanceFromTargetTrap(blueTrapLeftX, blueTrapLeftY);
        blueTrapRightDistance = s_Eyes.getDistanceFromTargetTrap(blueTrapRightX, blueTrapRightY);
        blueTrapCenterDistance = s_Eyes.getDistanceFromTargetTrap(blueTrapCenterX, blueTrapCenterY);

        redTrapLeftDistance = s_Eyes.getDistanceFromTargetTrap(redTrapLeftX, redTrapRightY);
        redTrapRightDistance = s_Eyes.getDistanceFromTargetTrap(redTrapRightX, redTrapRightY);
        redTrapCenterDistance = s_Eyes.getDistanceFromTargetTrap(redTrapCenterX, redTrapCenterY);

        double[] distances = {
            blueTrapLeftDistance,
            blueTrapRightDistance,
            blueTrapCenterDistance,
            redTrapLeftDistance,
            redTrapRightDistance,
            redTrapCenterDistance
        };

        for(int i = 0; i < distances.length; i++) {
            if (closestDistance < distances[i]) {
                closestDistance = distances[i];
            }
        }
        
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (closestDistance == blueTrapLeftDistance) {
                targetX = blueTrapLeftX;
                targetY = blueTrapLeftY;
                targetR = blueTrapLeftR;
            } else if (closestDistance == blueTrapRightDistance) {
                targetX = blueTrapRightX;
                targetY = blueTrapRightY;
                targetR = blueTrapRightR;
            } else if (closestDistance == blueTrapCenterDistance) {
                targetX = blueTrapCenterX;
                targetY = blueTrapCenterY;
                targetR = blueTrapCenterR;
            }
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            if (closestDistance == redTrapLeftDistance) {
                targetX = redTrapLeftX;
                targetY = redTrapLeftY;
                targetR = redTrapLeftR;
            } else if (closestDistance == redTrapRightDistance) {
                targetX = redTrapRightX;
                targetY = redTrapRightY;
                targetR = redTrapRightR;
            } else if (closestDistance == redTrapCenterDistance) {
                targetX = redTrapCenterX;
                targetY = redTrapCenterY;
                targetR = redTrapCenterR;
            }
        } else {
            closestDistance = distanceLimit;
        }

        //Log Target Trap Location
        translationPublisher.set(new Translation2d(targetX, targetY));

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(

            new Pose2d(s_Eyes.getRobotPose().getX(), s_Eyes.getRobotPose().getY(), Rotation2d.fromDegrees(0)),
            new Pose2d(targetX, targetY, Rotation2d.fromDegrees(0))

        );


        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(targetR))
        );

        // Prevent the path from being flipped as all coordinates are blue origin
        path.preventFlipping = true;

        if (closestDistance < distanceLimit) { //DO NOT INCLUDE DISTANCE LIMIT
            AutoBuilder.followPath(path);
            SmartDashboard.putString("Trap Status", "Going to Trap");
        } else{
            SmartDashboard.putString("Trap Status", "Too Far Away To Trap");
        }

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {

        double xError = Math.abs(targetX + s_Swerve.m_poseEstimator.getEstimatedPosition().getX());
        double yError = Math.abs(targetY + s_Swerve.m_poseEstimator.getEstimatedPosition().getY());
        double rError = Math.abs(targetR + s_Swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees() % 360);

        if (xError <= positionTolerance && yError <= positionTolerance && rError <= rotationTolerance) {
            return true;
        } else {
            return false;
        }

    }
}
