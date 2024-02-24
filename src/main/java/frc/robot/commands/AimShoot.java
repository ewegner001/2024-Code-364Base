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
    private final double d1Distance = 1.25;
    private final double d1Angle = 115.0;
    private final double d1LeftShooterSpeed = 90.0;
    private final double d1RightShooterSpeed = d1LeftShooterSpeed;

    private final double d2Distance = 2.0;
    private final double d2Angle = 128.0;
    private final double d2LeftShooterSpeed = 90.0;
    private final double d2RightShooterSpeed = d2LeftShooterSpeed;

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

    }

    @Override
    public void execute() {

        // assign target distance as variable
        distance = eyes.getDistanceFromTarget();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(d1Distance, d1Angle);
        shooterAngleInterpolation.put(d2Distance, d2Angle);

        // create points in shooter power linear interpolation line
        // TODO tune these values
        shooterLeftSpeedInterpolation.put(d1Distance, d1LeftShooterSpeed);
        shooterLeftSpeedInterpolation.put(d2Distance, d2LeftShooterSpeed);

        shooterRightSpeedInterpolation.put(d1Distance, d1RightShooterSpeed);
        shooterRightSpeedInterpolation.put(d2Distance, d2RightShooterSpeed);
    
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