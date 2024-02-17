package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class AimShoot extends Command {

    private Swerve swerve;
    private ShooterPivot shooterPivot;  
    private Shooter shooter;
    private double distance;
    private InterpolatingDoubleTreeMap shooterAngleInterpolation;
    private InterpolatingDoubleTreeMap shooterSpeedInterpolation;
    private double shooterAngle;
    private double shooterSpeed;

    public AimShoot(Swerve swerve, ShooterPivot shooterPivot, Shooter shooter) {

        this.swerve = swerve;
        this.shooterPivot = shooterPivot;
        this.shooter = shooter;

        addRequirements(swerve, shooterPivot, shooter);

        shooterAngleInterpolation = new InterpolatingDoubleTreeMap();

    }

    @Override
    public void execute() {

        distance = swerve.getDistanceFromTarget();

        //TODO tune
        shooterAngleInterpolation.put(0.0, 0.0);
        shooterAngleInterpolation.put(1.0, 1.0);

        shooterSpeedInterpolation.put(0.0, 0.0);
        shooterSpeedInterpolation.put(1.0, 1.0);
    
        shooterAngle = shooterAngleInterpolation.get(distance);
        shooterSpeed = shooterSpeedInterpolation.get(distance);
;
        shooterPivot.moveShooterPivot(shooterAngle);
        shooter.shootingMotorsSetControl(shooterSpeed, shooterSpeed);
        Timer.delay(0.5);
        shooter.frontShooterIntake();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.frontRollersStop();
    }

        // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; 
    }
}