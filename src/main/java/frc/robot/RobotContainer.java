package frc.robot;

import java.time.Instant;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private final int leftY = XboxController.Axis.kLeftY.value;
    private final int leftX = XboxController.Axis.kLeftX.value;
    private final int rightX = XboxController.Axis.kRightX.value;
    /*Operator Buttons */
    private final JoystickButton operatorButtonY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorButtonA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorButtonB = new JoystickButton(operator, XboxController.Button.kB.value);
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
    private final Elevator s_Elevator = new Elevator();
    private final Intake s_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();
    private final Eyes s_Eyes = new Eyes(s_Swerve);



    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */

    public RobotContainer() {

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(leftY), 
                () -> -driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                false
            )
        );
        
        } else {
        s_Swerve.setPose(new Pose2d(16.54, 0, new Rotation2d(Math.PI)));
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(leftY), 
                () -> driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Swerve.getGyroYaw().getDegrees(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                false
            )
        );

        }
        // Configure the button bindings
        configureButtonBindings();

        //Command ElevatorAtPosition = new s_Elevator.ElevatorAtPosition();

        Command AimThenShoot = new ParallelRaceGroup(
            new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter), 
            new SequentialCommandGroup(
                new WaitCommand(1.0), 
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage)), 
                new WaitCommand(1.0))
                );

        NamedCommands.registerCommand("Intake", new RunIntake(s_Intake, s_ShooterPivot, s_Shooter, s_Eyes).until(() -> !s_Shooter.getBreakBeamOutput()));
        NamedCommands.registerCommand("Score", AimThenShoot);
        NamedCommands.registerCommand("Aim", new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter));
        NamedCommands.registerCommand("Fire", new InstantCommand(() -> s_Shooter.setLoaderVoltage(s_Shooter.runLoaderVoltage)));
        
        autoChooser = AutoBuilder.buildAutoChooser();

        //autoChooser.addOption("Duluth Auto", new PathPlannerAuto("Duluth Auto"));

        //autoChooser.addOption("Example Auto", new PathPlannerAuto("Example Auto"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /*Operator Buttons */
        /*operatorButtonY.onTrue(new InstantCommand(()-> s_Elevator.lift()));
        operatorButtonB.onTrue(new InstantCommand(()-> s_Elevator.stop()));
        operatorButtonA.onTrue(new InstantCommand(()-> s_Elevator.down()));
        if(operatorButtonY.getAsBoolean() == false && operatorButtonA.getAsBoolean() == false){
            
        }*/

        // zero gyro
        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // aim speaker
        driverLeftTrigger.whileTrue(new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(leftY), 
                () -> driver.getRawAxis(leftX), 
                () -> driver.getRawAxis(rightX),
                () -> driverDpadUp.getAsBoolean(),
                () -> s_Eyes.getTargetRotation(),
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            ).alongWith(new AimShoot(s_Eyes, s_ShooterPivot, s_Shooter))//.until(() -> Math.abs(s_Swerve.getGyroYaw().getDegrees() % 360) < s_Swerve.getTargetRotation() + Constants.AUTO_ROTATE_DEADBAND && 
            //Math.abs(s_Swerve.getGyroYaw().getDegrees() % 360) > s_Swerve.getTargetRotation() - Constants.AUTO_ROTATE_DEADBAND)
        );

        // shoot speaker
        driverRightTrigger.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(12))
            )
        ).onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(0))
            )
        );

        // intake
        driverX.whileTrue(new RunIntake(s_Intake, s_ShooterPivot, s_Shooter, s_Eyes).until(() -> !s_Shooter.getBreakBeamOutput()).andThen(new InstantCommand(() -> s_Eyes.limelight.setLEDMode_ForceBlink("")))).onFalse(new InstantCommand(() -> s_Eyes.limelight.setLEDMode_ForceOff("")));

        // outake
        driverA.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> s_Intake.setIntakePivotPosition(s_Intake.intakeGroundPosition)),
            new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotIntakePosition)),
            new InstantCommand(() -> s_Intake.setIntakeVoltage(s_Intake.reverseIntakeVoltage))
        ));

        // climb reach
        driverLB.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(15)),
                new InstantCommand(() -> s_ShooterPivot.moveShooterPivot(s_ShooterPivot.shooterPivotClimbPosition))
            )
        );


        // climb pull up
        driverRB.onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Elevator.SetElevatorPosition(3))
                
            )
        );


        /* Operator Buttons */
        
        // aim amp
        operatorLeftTrigger.onTrue(

            new SequentialCommandGroup(
                new AmpElevator(s_Elevator),
                s_Elevator.ElevatorAtPosition(),
                new AmpShooterPivot(s_ShooterPivot)
            )
        ).onFalse(
                new SequentialCommandGroup(
                    new AmpShooterPivotRetract(s_ShooterPivot),
                    s_ShooterPivot.ShooterPivotAtPosition(),
                    new AmpElevatorRetract(s_Elevator)
                )
        );

        // shoot amp
        operatorRightTrigger.onTrue(
            new ParallelCommandGroup(
                //new InstantCommand(() -> s_Shooter.setShooterVoltage(3, -3)),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(-3))
            )
        ).onFalse(
            new ParallelCommandGroup(
                //new InstantCommand(() -> s_Shooter.setShooterVoltage(0, 0)),
                new InstantCommand(() -> s_Shooter.setLoaderVoltage(0))
            )
        );

        // move elevator down (emergency use)
        operatorA.onTrue(

            new InstantCommand(() -> s_Elevator.SetElevatorPosition(0))

        );
        

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //s_Swerve.swerveOdometry.update()
        return autoChooser.getSelected();
        
    }
}
