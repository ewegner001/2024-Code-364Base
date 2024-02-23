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
    private InterpolatingDoubleTreeMap shooterLeftSpeedInterpolation;
    private InterpolatingDoubleTreeMap shooterRightSpeedInterpolation;

    // local variables
    private double shooterAngle;
    private double leftShooterSpeed;
    private double rightShooterSpeed;

    // positions
    private final double d1Distance = 0.0;
    private final double d1Angle = 0.0;
    private final double d1LeftShooterSpeed = 0.0;
    private final double d1RightShooterSpeed = 0.0;

    private final double d2Distance = 1.0;
    private final double d2Angle = 1.0;
    private final double d2LeftShooterSpeed = 1.0;
    private final double d2RightShooterSpeed = 1.0;

    // constructor
    public AimShoot(Swerve swerve, ShooterPivot shooterPivot, Shooter shooter) {

        this.swerve = swerve;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;

        addRequirements(swerve, shooterPivot, shooter);

        // instantiate objects
        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();
        shooterLeftSpeedInterpolation = new InterpolatingDoubleTreeMap();
        shooterRightSpeedInterpolation = new InterpolatingDoubleTreeMap();

    }

    @Override
    public void execute() {

        // assign target distance as variable
        distance = swerve.getDistanceFromTarget();

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
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}