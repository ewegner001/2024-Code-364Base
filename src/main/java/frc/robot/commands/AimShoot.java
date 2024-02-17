package frc.robot.commands;

import frc.robot.subsystems.ShooterPivot;

import edu.wpi.first.wpilibj2.command.Command;


public class AimShoot extends Command {

    private ShooterPivot shooterPivot;    

    public AimShoot(ShooterPivot shooterPivot) {

        this.shooterPivot = shooterPivot;

        addRequirements(shooterPivot);

    }

    @Override
    public void execute() {

        shooterPivot.moveShooterPivot(shooterPivot.getShooterAngle());
    }

      // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false; 
  }
}