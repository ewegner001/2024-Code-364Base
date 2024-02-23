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
    private Swerve swerve;
    private Eyes eyes;
    private ShooterPivot shooterPivot;  
    private Shooter shooter;
    private double distance;

    // required WPILib class objects
    private InterpolatingDoubleTreeMap shooterAngleInterpolation;
    private InterpolatingDoubleTreeMap shooterSpeedInterpolation;

    // local variables
    private double shooterAngle;
    private double shooterSpeed;

    // constructor
    public AimShoot(Swerve swerve, ShooterPivot shooterPivot, Shooter shooter) {

        this.swerve = swerve;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;

        addRequirements(swerve, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();

    }

    @Override
    public void execute() {

        // assign target distance as variable
        distance = swerve.getDistanceFromTarget();

        // create points in angle linear interpolation line
        // TODO tune these values
        shooterAngleInterpolation.put(0.0, 0.0);
        shooterAngleInterpolation.put(1.0, 1.0);

        // create points in shooter power linear interpolation line
        // TODO tune these values
        shooterSpeedInterpolation.put(0.0, 0.0);
        shooterSpeedInterpolation.put(1.0, 1.0);
    
        // get desired shooter angle using the linear interpolation
        // x (input) = distance
        // y (output) = shooter angle
        shooterAngle = shooterAngleInterpolation.get(distance);

        // get desired shooter power using the linear interpolation
        // x (input) = distance
        // y (output) = shooter power
        shooterSpeed = shooterSpeedInterpolation.get(distance);

        // move shooter to calculated angle
        shooterPivot.moveShooterPivot(shooterAngle);

        // set shooter speed power to calculated value
        shooter.shootingMotorsSetControl(shooterSpeed, shooterSpeed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setLoaderVoltage(shooter.stopLoaderVoltage);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}