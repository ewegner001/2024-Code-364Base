// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private int ShooterPiviotMotorID = 0;
  private int ShooterPiviotCANCoderID = 0;
  private double ShooterPiviotPGains = 0;
  private double ShooterPiviotIGains = 0;
  private double ShooterPiviotDGains = 0;
  private double ShooterPiviotGGains = 0;
  private double shooterPivotGearRatio = 54.545;

  private CANSparkMax motor;
  private CANcoder cancoder;
  private double m_setPoint;
  private PIDController pid;
  private ArmFeedforward shooterPivotFeedforward;
  private RelativeEncoder shooterPivotMotorEncoder;
  private Swerve swerve;


  /** Creates a new ShooterPiviot. */
  public ShooterPivot() {
    swerve = new Swerve();
    motor = new CANSparkMax(ShooterPiviotMotorID, MotorType.kBrushless);
    cancoder = new CANcoder(ShooterPiviotCANCoderID);


    shooterPivotMotorEncoder = motor.getEncoder();
    shooterPivotMotorEncoder.setPositionConversionFactor(360 / shooterPivotGearRatio);
    shooterPivotMotorEncoder.setPosition(cancoder.getPosition().getValue());
    shooterPivotMotorEncoder.setVelocityConversionFactor(360 / shooterPivotGearRatio);


    m_setPoint = 0;
    pid = new PIDController(ShooterPiviotPGains, ShooterPiviotIGains, ShooterPiviotDGains);
    shooterPivotFeedforward = new ArmFeedforward(0, ShooterPiviotGGains, 0, 0);

  }

  public void moveShooterPivot(double setPoint) {
    m_setPoint = setPoint;
  }



 
  @Override
  public void periodic() {
    double feedForward = shooterPivotFeedforward.calculate(Units.degreesToRadians(shooterPivotMotorEncoder.getPosition()), Units.degreesToRadians(shooterPivotMotorEncoder.getVelocity()));
    motor.setVoltage(pid.calculate(shooterPivotMotorEncoder.getPosition(), m_setPoint) + feedForward);
    // This method will be called once per scheduler run
  }
}
