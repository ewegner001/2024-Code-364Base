// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final double intakePivotMotorGearRatio = 100.0;
  private final int intakeMotorID = 12;
  private final int intakePivotID = 11;
  private final int intakePivotEncoderID = 1;

  private CANSparkMax intakeMotor;
  private CANSparkMax m_IntakePiviot;
  private CANcoder intakePivotEncoder;
  private RelativeEncoder intakePivotMotorEncoder;
  private PIDController pid;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    m_IntakePiviot = new CANSparkMax(intakePivotID, MotorType.kBrushless);
    intakePivotEncoder = new CANcoder(intakePivotEncoderID);
    intakePivotMotorEncoder = m_IntakePiviot.getEncoder();
    intakePivotMotorEncoder.setPositionConversionFactor(360 / intakePivotMotorGearRatio);
    intakePivotMotorEncoder.setPosition(intakePivotEncoder.getPosition().getValue());
    pid = new PIDController(intakePivotID, intakePivotEncoderID, intakeMotorID);
  }

  private void intakePivotMotorPID() {
    
  }





  

  

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
