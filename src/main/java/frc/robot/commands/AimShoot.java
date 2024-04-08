/*
 * This command will use linear interpolation to adjust the
 * shooter angle and shooter power based upon the distance from the target,
 * which is determined using the robot pose estimator. This command assumes
 * that there is a note in the loader and not touching the shooter wheels.
 * 
 * Parameters:
 * 
 * Swerve           (subsystem)
 * ShooterPivot     (subsystem)
 * Shooter          (subsystem)
 * 
 */

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

import java.sql.Driver;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class AimShoot extends Command {

    // required subystems
    private Eyes eyes;
    private ShooterPivot shooterPivot;  
    private Shooter shooter;
    private Elevator elevator;
    private RobotContainer robotContainer;

    private double distance;
    private double manualDistance = 0.0;

    // required WPILib class objects
    private InterpolatingDoubleTreeMap shooterAngleInterpolation;
    private InterpolatingDoubleTreeMap shooterAngleInterpolationAuto;
    private InterpolatingDoubleTreeMap shooterAngleInterpolationElevator;
    private InterpolatingDoubleTreeMap shooterSpeedInterpolation;

    // local variables
    private double shooterAngle;
    private double shooterSpeed;

    public boolean isElevatorShot = false;
    public boolean onMove = false;
    public boolean feed = false;

    // positions 

    //Higher note shot is lower angle!!!
    private final double subWooferDistance = 1.31; //1.21 at 930, 1.25 at comp, 1.31 at marquette
    private final double subWooferAngle = 115.0; //115
    private final double subWooferSpeed = 35.0; //50
    
    private final double xSpotDistance = 2.45; //2.45 at marquette
    private final double xSpotAngle = 131.0;
    private final double xSpotSpeed = 45.0;

    private final double podiumDistance = 3.02; //3.17 at comp, 3.02 at marquette
    private final double podiumAngle = 137;
    private final double podiumSpeed = 60.0;

    private final double chainDistance = 4.33; //4.33 at marquette (NOT ACCCURATE)
    private final double chainAngle = 145.0;
    private final double chainSpeed = 80.0;

    private final double wingerDistance = 5.44;
    private final double wingerAngle = 147;
    private final double wingerSpeed = 40.25;


 
    private final double feedDistance = 2.4;
    private final double feedAngle = 117.0;
    private final double feedSpeed = 65.0;

    private final double elevatorShotDistance = 2.65;
    private final double elevatorShotAngle = 146;
    private final double elevatorShotShooterSpeed = 30.0;


    // constructor
    public AimShoot(Eyes eyes, ShooterPivot shooterPivot, Shooter shooter, boolean isElevatorShot, boolean onMove, boolean feed) {
        this.eyes = eyes;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.isElevatorShot = isElevatorShot;
        this.onMove = onMove;
        this.feed = feed;

        addRequirements(eyes, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();
        shooterAngleInterpolationElevator = new InterpolatingDoubleTreeMap();
        shooterSpeedInterpolation = new InterpolatingDoubleTreeMap();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(subWooferDistance, subWooferAngle);
        shooterAngleInterpolation.put(podiumDistance, podiumAngle);
        shooterAngleInterpolation.put(xSpotDistance, xSpotAngle);
        shooterAngleInterpolation.put(chainDistance, chainAngle);
        shooterAngleInterpolation.put(wingerDistance, wingerAngle);


        shooterAngleInterpolationElevator.put(elevatorShotDistance, elevatorShotAngle);

        // create points in shooter linear interpolation line
        // TODO tune these values
        shooterSpeedInterpolation.put(podiumDistance, podiumSpeed);
        shooterSpeedInterpolation.put(chainDistance, chainSpeed);
        shooterSpeedInterpolation.put(xSpotDistance, xSpotSpeed);
        shooterSpeedInterpolation.put(wingerDistance, wingerSpeed);
        shooterSpeedInterpolation.put(subWooferDistance, subWooferSpeed);

    }

    public AimShoot(Eyes eyes, ShooterPivot shooterPivot, Shooter shooter, double manualDistance) {
        this.eyes = eyes;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.manualDistance = manualDistance;

        addRequirements(eyes, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();
        shooterAngleInterpolationElevator = new InterpolatingDoubleTreeMap();
        shooterSpeedInterpolation = new InterpolatingDoubleTreeMap();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(subWooferDistance, subWooferAngle);
        shooterAngleInterpolation.put(podiumDistance, podiumAngle);
        shooterAngleInterpolation.put(xSpotDistance, xSpotAngle);
        shooterAngleInterpolation.put(chainDistance, chainAngle);
        shooterAngleInterpolation.put(wingerDistance, wingerAngle);

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterSpeedInterpolation.put(podiumDistance, podiumSpeed);
        shooterSpeedInterpolation.put(chainDistance, chainSpeed);
        shooterSpeedInterpolation.put(xSpotDistance, xSpotSpeed);
        shooterSpeedInterpolation.put(wingerDistance, wingerSpeed);
        shooterSpeedInterpolation.put(subWooferDistance, subWooferSpeed);

        shooterAngleInterpolationElevator.put(elevatorShotDistance, elevatorShotAngle);
    }

    @Override
    public void execute() {

        // assign target distance as variable

        if (isElevatorShot) {
            distance = elevatorShotDistance;
            shooterAngle = elevatorShotAngle;
        } else if(feed) {
            distance = feedDistance;
            shooterAngle = feedAngle;
        } else if(manualDistance == 0) {
            if (onMove == true) {
                distance = eyes.getDistanceFromMovingTarget();
                shooterAngle = shooterAngleInterpolation.get(distance);
            } else {
                distance = eyes.getDistanceFromTarget();
                shooterAngle = shooterAngleInterpolation.get(distance);
            }

        } else {
            distance = manualDistance;
            shooterAngle = shooterAngleInterpolation.get(distance);
        }
        
        // move shooter to calculated angle
        shooterPivot.moveShooterPivot(shooterAngle);

        
        // get desired shooter power using the linear interpolation
        // x (input) = distance
        // y (output) = shooter power
        if(isElevatorShot) {
            shooterSpeed = elevatorShotShooterSpeed;
        } else if (feed){
            shooterSpeed = feedSpeed;
        } else {
            shooterSpeed = shooterSpeedInterpolation.get(distance);
        }
        
        // set shooter speed power to calculated value
        shooter.shootingMotorsSetControl(shooterSpeed, shooterSpeed);

        if(shooterPivot.atPosition() == true && eyes.swerveAtPosition(onMove) == true && shooter.isUpToSpeed() == true) {
            eyes.controllerRumble = true;
        } else {
            eyes.controllerRumble = false;
        }
        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setLoaderVoltage(shooter.stopLoaderVoltage);
        shooter.setShooterVoltage(shooter.stopShooterVoltage, shooter.stopShooterVoltage);
        if (DriverStation.isAutonomous()) {
            shooter.shootingMotorsSetControl(subWooferSpeed, subWooferSpeed);
        }
        eyes.controllerRumble = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}