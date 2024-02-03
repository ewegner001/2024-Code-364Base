// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterFrontRollerRun extends Command {
  private Shooter shooter;
  /** Creates a new ShooterFrontRollerRun. */
  public ShooterFrontRollerRun(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    //shooter = new Shooter();
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.frontShooterIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.frontRollersStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.sensorValue() == true) {
      return false;
    } else {
      return true;
    }
  }
}
