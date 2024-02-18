// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final double intakePivotMotorGearRatio = 100.0;
  private final int intakeMotorID = 10;
  private final int intakePivotID = 11;
  private final int intakePivotEncoderID = 9;
  private final double intakePValue = 0.2;
  private final double intakeIValue = 0.0;
  private final double intakeDValue = 0.0;
  private final double intakeSValue = 0.0;
  private final double intakeGValue = 0.0;
  private final double intakeVValue = 0.0;
  private final double magnetOffSet = 0.0;

  private CANSparkMax intakeMotor;
  private CANSparkMax m_IntakePiviot;
  private CANcoder intakePivotEncoder;
  private RelativeEncoder intakePivotMotorEncoder;
  private PIDController pid;
  private ArmFeedforward intakePivotFeedforward;
  private double m_setPoint;
  private double m_IntakePiviotVoltage = 0.0;


  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    m_IntakePiviot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    intakePivotEncoder = new CANcoder(intakePivotEncoderID);
    CANcoderConfigurator configuator = intakePivotEncoder.getConfigurator();

    configuator.apply(
      new MagnetSensorConfigs()
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withMagnetOffset(magnetOffSet)
    );

    intakePivotMotorEncoder = m_IntakePiviot.getEncoder();
    intakePivotMotorEncoder.setPositionConversionFactor(360 / intakePivotMotorGearRatio);
    intakePivotMotorEncoder.setPosition(intakePivotEncoder.getPosition().getValue());
    intakePivotMotorEncoder.setVelocityConversionFactor(360 / intakePivotMotorGearRatio);

    pid = new PIDController(intakePValue, intakeIValue, intakeDValue);
    intakePivotFeedforward = new ArmFeedforward(intakeSValue, intakeGValue, intakeVValue);
    m_setPoint = 70;

    intakeMotor.setVoltage(0);

  }

  public void intakePivotRun(double setpoint) {
    m_setPoint = setpoint;
  }

  public void intakeMotorRun() {
    intakeMotor.setVoltage(12);
    }

  public void intakeMotorStop() {
    intakeMotor.setVoltage(0);
  }

  public void intakeMotorRunOut() {
    intakeMotor.setVoltage(-12);
  }

  public double cancoderInDegrees() {
    return intakePivotEncoder.getPosition().getValue() * 360;
  }


  

   @Override
   public void periodic() {
    //double feedForward = intakePivotFeedforward.calculate(Units.degreesToRadians(cancoderInDegrees()), Units.degreesToRadians(cancoder.getVelocity()));
    m_IntakePiviotVoltage = pid.calculate(cancoderInDegrees(), m_setPoint); //m_IntakePiviotVoltage = pid.calculate(intakePivotMotorEncoder.getPosition(), m_setPoint) + feedForward;
    m_IntakePiviot.setVoltage(m_IntakePiviotVoltage);

    SmartDashboard.putNumber("Intake Voltage", m_IntakePiviotVoltage);
    SmartDashboard.putNumber("Intake CANcoder", cancoderInDegrees());
    SmartDashboard.putNumber("Intake Pivot Motor Position", intakePivotMotorEncoder.getPosition());
    SmartDashboard.putNumber("Intake setpoint", m_setPoint);
   }
}
