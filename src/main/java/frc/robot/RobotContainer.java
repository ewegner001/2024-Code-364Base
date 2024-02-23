package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    double rotationSpeed = 1.0;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int driverLeftY = XboxController.Axis.kLeftY.value;
    private final int driverLeftX = XboxController.Axis.kLeftX.value;
    private final int driverRightX = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driverRStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final POVButton driverDpadUp = new POVButton(driver, 0);
    private final POVButton driverDpadRight = new POVButton(driver, 90);
    private final POVButton driverDpadDown = new POVButton(driver, 180);
    private final POVButton driverDpadLeft = new POVButton(driver, 270);

    /* driver axis */
    private final int driverLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int driverRightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    
    /* driver triggers */
    final Trigger driverLeftTrigger = new Trigger(() -> driver.getRawAxis(driverLeftTriggerAxis) > 0.1);
    final Trigger driverRightTrigger = new Trigger(() -> driver.getRawAxis(driverRightTriggerAxis) > 0.1);
 
    /* Operator Buttons */
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorLB = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRB = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorLStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorRStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton operatorUpStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorDownStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);

    private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final POVButton operatorDpadUp = new POVButton(operator, 0);
    private final POVButton operatorDpadRight = new POVButton(operator, 90);
    private final POVButton operatorDpadDown = new POVButton(operator, 180);
    private final POVButton operatorDpadLeft = new POVButton(operator, 270);

    /* operator axis */
    private final int operatorLeftYAxis = XboxController.Axis.kLeftY.value;
    private final int operatorRightYAxis = XboxController.Axis.kRightY.value;
    private final int operatorLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int operatorRightTriggerAxis = XboxController.Axis.kRightTrigger.value;

    /* operator triggers */
    final Trigger operatorLeftTrigger = new Trigger(() -> operator.getRawAxis(operatorLeftTriggerAxis) > 0.1);
    final Trigger operatorRightTrigger = new Trigger(() -> operator.getRawAxis(operatorRightTriggerAxis) > 0.1);
    
    
    /* Subsystems */
    private final ShooterPivot s_ShooterPivot = new ShooterPivot();
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();
    private final Eyes s_Eyes = new Eyes();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(driverLeftY), 
                () -> -driver.getRawAxis(driverLeftX), 
                () -> -driver.getRawAxis(driverRightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                false
            )
        );

     
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driverB.toggleOnTrue(new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(driverLeftY), 
                () -> -driver.getRawAxis(driverLeftX), 
                () -> -driver.getRawAxis(driverRightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Swerve.getTargetRotation(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            )//.until(() -> Math.abs(s_Swerve.getGyroYaw().getDegrees() % 360) < s_Swerve.getTargetRotation() + Constants.AUTO_ROTATE_DEADBAND && 
            //Math.abs(s_Swerve.getGyroYaw().getDegrees() % 360) > s_Swerve.getTargetRotation() - Constants.AUTO_ROTATE_DEADBAND)
        );

        // Go To Intake Position and back again
        driverX.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeGroundPosition)), // Move Intake Pivot to intaking position
            new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.runIntakeVoltage)), // Run Intake
            new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotIntakePosition)), // Move Shooter Pivot to intaking position
            new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage)), // Run the Shooter Loader
            new InstantCommand(() -> s_Shooter.setShooterVoltage(-s_Shooter.reverseShooterVoltage, s_Shooter.reverseShooterVoltage)) // Run the Shooter backwards slightly to avoid the notes touching the shooter wheels
        ))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeSafePosition)), // Move Intake Pivot to safe position
            new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.stopIntakeVoltage)), // Stop Intake
            new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotStowPosition)), // Move Shooter Pivot to stow position
            new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage)), // Stop the Shooter Loader
            new InstantCommand(() -> s_Shooter.setShooterVoltage(s_Shooter.stopShooterVoltage, s_Shooter.stopShooterVoltage)) // Stop the Shooter
        ));
        
        /* Operator Buttons */
        // Reverse Shooter and Shooter Loader
        operatorLB.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.reverseLoaderVoltage)), // Reverse the Shooter Loader
            new InstantCommand(() -> s_Shooter.setShooterVoltage(-s_Shooter.runShooterVoltage, s_Shooter.runShooterVoltage)) // Reverse the Shooter
        ))
        .onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.stopLoaderVoltage)), // Stop the Shooter Loader
            new InstantCommand(() -> s_Shooter.setShooterVoltage(s_Shooter.stopShooterVoltage, s_Shooter.stopShooterVoltage)) // Stop the Shooter
        ));
        
        // Run the Shooter
        operatorLeftTrigger.whileTrue(
            new InstantCommand(() -> s_Shooter.setShooterVoltage(s_Shooter.runShooterVoltage, -s_Shooter.runShooterVoltage)) 
        ).onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> s_Shooter.setShooterVoltage(s_Shooter.stopShooterVoltage, s_Shooter.stopShooterVoltage))
        ));

        // aim and shoot
        operatorRightTrigger.onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.reverseLoaderVoltage)),
                new AimShoot(s_Swerve, s_ShooterPivot, s_Shooter)
            ).withTimeout(1.5),
            new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage))
        ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto("Example Auto");
        
    }
}
