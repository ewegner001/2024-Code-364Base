package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class LockApriltag extends Command {    
    private Swerve s_Swerve;    
    private Eyes eyes;

    public LockApriltag(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        double xPos = eyes.getDataPackage()[0];

        /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            xPos * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }
}