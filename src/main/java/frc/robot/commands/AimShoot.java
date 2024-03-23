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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

    // local variables
    private double shooterAngle;
    private double leftShooterSpeed;
    private double rightShooterSpeed;

    public boolean isElevatorShot = false;

    // positions 

    //Higher note shot is lower angle!!!
    private final double subWooferDistance = 1.21; //1.21 at 930, 1.25 at comp
    private final double subWooferAngle = 115.0;
    private final double subWooferLeftShooterSpeed = 90.0;
    private final double subWooferRightShooterSpeed = subWooferLeftShooterSpeed;

    private final double d2Distance = 2.15;
    private final double d2Angle = 128.0;
    private final double d2LeftShooterSpeed = 90.0;
    private final double d2RightShooterSpeed = d2LeftShooterSpeed;

    private final double podiumDistance = 2.98; //2.98 at 930, 3.17 at comp
    private final double podiumAngle = 138;
    private final double podiumLeftShooterSpeed = 90.0;
    private final double podiumRightShooterSpeed = podiumLeftShooterSpeed;

    private final double d3Distance = 4.1;
    private final double d3Angle = 140.0;
    private final double d3LeftShooterSpeed = 95.0;
    private final double d3RightShooterSpeed = d3LeftShooterSpeed;


    private final double subWooferDistanceAuto = 1.25;
    private final double subWooferAngleAuto = 115.0;
    private final double subWooferLeftShooterSpeedAuto = 90.0;
    private final double subWooferRightShooterSpeedAuto = subWooferLeftShooterSpeed;

    private final double d2DistanceAuto = 2.15;
    private final double d2AngleAuto = 128.0;
    private final double d2LeftShooterSpeedAuto = 90.0;
    private final double d2RightShooterSpeedAuto = d2LeftShooterSpeed;

    private final double xSpotDistance = 2.4;
    private final double xSpotAngle = 131.0;
    private final double xSpotLeftShooterSpeed = 92.5;
    private final double xSpotRightShooterSpeed = xSpotLeftShooterSpeed;

    private final double podiumDistanceAuto = 3.17;
    private final double podiumAngleAuto = 138;
    private final double podiumLeftShooterSpeedAuto = 90.0;
    private final double podiumRightShooterSpeedAuto = podiumLeftShooterSpeed;

    private final double d3DistanceAuto = 5.27;
    private final double d3AngleAuto = 136.0;
    private final double d3LeftShooterSpeedAuto = 95.0;
    private final double d3RightShooterSpeedAuto = d3LeftShooterSpeed;

    private final double xSpotDistanceAuto = 2.4;
    private final double xSpotAngleAuto = 130.0;
    private final double xSpotLeftShooterSpeedAuto = 90.0;
    private final double xSpotRightShooterSpeedAuto = xSpotLeftShooterSpeed;


    private final double elevatorShotDistance = 2.65;
    private final double elevatorShotAngle = 144;
    private final double elevatorShotLeftShooterSpeed = 90;
    private final double elevatorShotRightShooterSpeed = elevatorShotLeftShooterSpeed;


    // constructor
    public AimShoot(Eyes eyes, ShooterPivot shooterPivot, Shooter shooter, boolean isElevatorShot) {
        this.eyes = eyes;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.isElevatorShot = isElevatorShot;

        addRequirements(eyes, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();
        shooterAngleInterpolationAuto = new InterpolatingDoubleTreeMap();
        shooterAngleInterpolationElevator = new InterpolatingDoubleTreeMap();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(subWooferDistance, subWooferAngle);
        //shooterAngleInterpolation.put(d2Distance, d2Angle);
        shooterAngleInterpolation.put(podiumDistance, podiumAngle);
        shooterAngleInterpolation.put(d3Distance, d3Angle);
        shooterAngleInterpolation.put(xSpotDistance, xSpotAngle);


        shooterAngleInterpolationAuto.put(subWooferDistanceAuto, subWooferAngleAuto);
        shooterAngleInterpolationAuto.put(d2DistanceAuto, d2AngleAuto);
        shooterAngleInterpolationAuto.put(podiumDistanceAuto, podiumAngleAuto);
        shooterAngleInterpolationAuto.put(d3DistanceAuto, d3AngleAuto);
        shooterAngleInterpolationAuto.put(xSpotDistanceAuto, xSpotAngleAuto);

        shooterAngleInterpolationElevator.put(elevatorShotDistance, elevatorShotAngle);

    }

    public AimShoot(Eyes eyes, ShooterPivot shooterPivot, Shooter shooter, double manualDistance) {
        this.eyes = eyes;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;
        this.manualDistance = manualDistance;

        addRequirements(eyes, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();
        shooterAngleInterpolationAuto = new InterpolatingDoubleTreeMap();
        shooterAngleInterpolationElevator = new InterpolatingDoubleTreeMap();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(subWooferDistance, subWooferAngle);
        shooterAngleInterpolation.put(d2Distance, d2Angle);
        shooterAngleInterpolation.put(podiumDistance, podiumAngle);
        shooterAngleInterpolation.put(d3Distance, d3Angle);
        shooterAngleInterpolation.put(xSpotDistance, xSpotAngle);

        shooterAngleInterpolationAuto.put(subWooferDistanceAuto, subWooferAngleAuto);
        shooterAngleInterpolationAuto.put(d2DistanceAuto, d2AngleAuto);
        shooterAngleInterpolationAuto.put(podiumDistanceAuto, podiumAngleAuto);
        shooterAngleInterpolationAuto.put(d3DistanceAuto, d3AngleAuto);
        shooterAngleInterpolationAuto.put(xSpotDistanceAuto, xSpotAngleAuto);

        shooterAngleInterpolationElevator.put(elevatorShotDistance, elevatorShotAngle);
    }

    @Override
    public void execute() {

        // assign target distance as variable

        if (isElevatorShot == true) {
            distance = elevatorShotDistance;
            shooterAngle = elevatorShotAngle;
        } else if(manualDistance == 0) {
            distance = eyes.getDistanceFromTarget();
            shooterAngle = shooterAngleInterpolation.get(distance);
        } else {
            distance = manualDistance;
            shooterAngle = shooterAngleInterpolation.get(distance);
        }
        
        // get desired shooter angle using the linear interpolation
        // x (input) = distance
        // y (output) = shooter angle
        


        // get desired shooter power using the linear interpolation
        // x (input) = distance
        // y (output) = shooter power
        leftShooterSpeed = shooter.shotSpeedRPS;
        rightShooterSpeed = shooter.shotSpeedRPS * shooter.shooterSpinReduction;
        


        // move shooter to calculated angle
        shooterPivot.moveShooterPivot(shooterAngle);

        // set shooter speed power to calculated value
        shooter.shootingMotorsSetControl(rightShooterSpeed, leftShooterSpeed);

        if(shooterPivot.atPosition() == true && eyes.swerveAtPosition() == true && shooter.isUpToSpeed() == true) {
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
        eyes.controllerRumble = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}