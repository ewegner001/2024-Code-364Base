// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private int ShooterPivotMotorID = 12;
  private int ShooterPivotCANCoderID = 15;
  private double ShooterPivotPGains = 0.3;
  private double ShooterPivotIGains = 0;
  private double ShooterPivotDGains = 0;
  private double ShooterPivotGGains = 0;
  private double shooterPivotGearRatio = 54.545;
  private final double magnetOffset = 0.0;

  private CANSparkMax m_ShooterPivot;
  private CANcoder cancoder;
  private double m_setPoint;
  private PIDController pid;
  private ArmFeedforward shooterPivotFeedforward;
  private RelativeEncoder shooterPivotMotorEncoder;
  private double m_ShooterPivotVoltage;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    
    m_ShooterPivot = new CANSparkMax(ShooterPivotMotorID, MotorType.kBrushless);
    cancoder = new CANcoder(ShooterPivotCANCoderID);

    CANcoderConfigurator cancoderConfigurator = cancoder.getConfigurator();
    cancoderConfigurator.apply(
      new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf).withMagnetOffset(magnetOffset)
    );
    shooterPivotMotorEncoder = m_ShooterPivot.getEncoder();
    shooterPivotMotorEncoder.setPositionConversionFactor(360 / shooterPivotGearRatio);
    shooterPivotMotorEncoder.setPosition(cancoderInDegrees());
    shooterPivotMotorEncoder.setVelocityConversionFactor(360 / shooterPivotGearRatio);


    m_setPoint = 115;
    pid = new PIDController(ShooterPivotPGains, ShooterPivotIGains, ShooterPivotDGains);
    shooterPivotFeedforward = new ArmFeedforward(0, ShooterPivotGGains, 0, 0);

  }

  public void moveShooterPivot(double setPoint) {
    m_setPoint = setPoint;
  }

  private double cancoderInDegrees() {
    return cancoder.getPosition().getValue() * 360;
  }



 
  @Override
  public void periodic() {
    //double feedForward = shooterPivotFeedforward.calculate(Units.degreesToRadians(shooterPivotMotorEncoder.getPosition()), Units.degreesToRadians(shooterPivotMotorEncoder.getVelocity()));
    m_ShooterPivotVoltage = pid.calculate(cancoderInDegrees(), m_setPoint); //m_IntakePivotVoltage = pid.calculate(intakePivotMotorEncoder.getPosition(), m_setPoint) + feedForward;
    m_ShooterPivot.setVoltage(m_ShooterPivotVoltage);

    SmartDashboard.putNumber("Shooter Voltage", m_ShooterPivotVoltage);
    SmartDashboard.putNumber("Shooter CANcoder", cancoderInDegrees());
    SmartDashboard.putNumber("Shooter Pivot Motor Position", shooterPivotMotorEncoder.getPosition());
    SmartDashboard.putNumber("Shooter setpoint", m_setPoint);
  }
}
