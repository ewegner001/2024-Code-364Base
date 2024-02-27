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
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class AimShoot extends Command {

    // required subystems
    private Eyes eyes;
    private ShooterPivot shooterPivot;  
    private Shooter shooter;
    private double distance;

    // required WPILib class objects
    private InterpolatingDoubleTreeMap shooterAngleInterpolation;
    private InterpolatingDoubleTreeMap shooterLeftSpeedInterpolation;
    private InterpolatingDoubleTreeMap shooterRightSpeedInterpolation;

    // local variables
    private double shooterAngle;
    private double leftShooterSpeed;
    private double rightShooterSpeed;

    // positions
    private final double subWooferDistance = 1.25;
    private final double subWooferAngle = 115.0;
    private final double subWooferLeftShooterSpeed = 90.0;
    private final double subWooferRightShooterSpeed = subWooferLeftShooterSpeed;

    private final double d2Distance = 2.15;
    private final double d2Angle = 130.0;
    private final double d2LeftShooterSpeed = 90.0;
    private final double d2RightShooterSpeed = d2LeftShooterSpeed;

    private final double podiumDistance = 3.17;
    private final double podiumAngle = 137.0;
    private final double podiumLeftShooterSpeed = 90.0;
    private final double podiumRightShooterSpeed = d2LeftShooterSpeed;

    private final double d3Distance = 4.0;
    private final double d3Angle = 137.0;
    private final double d3LeftShooterSpeed = 90.0;
    private final double d3RightShooterSpeed = d2LeftShooterSpeed;

    // constructor
    public AimShoot(Eyes eyes, ShooterPivot shooterPivot, Shooter shooter) {
        this.eyes = eyes;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;

        addRequirements(eyes, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();
        shooterLeftSpeedInterpolation = new InterpolatingDoubleTreeMap();
        shooterRightSpeedInterpolation = new InterpolatingDoubleTreeMap();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(subWooferDistance, subWooferAngle);
        shooterAngleInterpolation.put(d2Distance, d2Angle);
        shooterAngleInterpolation.put(podiumDistance, podiumAngle);
        shooterAngleInterpolation.put(d3Distance, d3Angle);

        // create points in shooter power linear interpolation line
        // TODO tune these values
        shooterLeftSpeedInterpolation.put(subWooferDistance, subWooferLeftShooterSpeed);
        shooterLeftSpeedInterpolation.put(d2Distance, d2LeftShooterSpeed);
        shooterLeftSpeedInterpolation.put(podiumDistance, podiumLeftShooterSpeed);
        shooterLeftSpeedInterpolation.put(d3Distance, d3LeftShooterSpeed);

        shooterRightSpeedInterpolation.put(subWooferDistance, subWooferRightShooterSpeed);
        shooterRightSpeedInterpolation.put(d2Distance, d2RightShooterSpeed);
        shooterRightSpeedInterpolation.put(podiumDistance, podiumRightShooterSpeed);
        shooterRightSpeedInterpolation.put(d3Distance, d3RightShooterSpeed);

    }

    @Override
    public void execute() {

        // assign target distance as variable
        distance = eyes.getDistanceFromTarget();

    
        // get desired shooter angle using the linear interpolation
        // x (input) = distance
        // y (output) = shooter angle
        shooterAngle = shooterAngleInterpolation.get(distance);

        // get desired shooter power using the linear interpolation
        // x (input) = distance
        // y (output) = shooter power
        leftShooterSpeed = shooterLeftSpeedInterpolation.get(distance);
        rightShooterSpeed = shooterRightSpeedInterpolation.get(distance);
        


        // move shooter to calculated angle
        shooterPivot.moveShooterPivot(shooterAngle);

        // set shooter speed power to calculated value
        shooter.shootingMotorsSetControl(rightShooterSpeed, leftShooterSpeed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setLoaderVoltage(shooter.stopLoaderVoltage);
        shooter.setShooterVoltage(shooter.stopShooterVoltage, shooter.stopShooterVoltage);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}