// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  /** Creates a new Intake. */
  public Intake() {
    motor = new CANSparkMax(1, MotorType.kBrushless);
  }

public void run(double speed) {
  motor.set(speed);
}

public double intakeMotorTempotorVolt() {
  return motor.getBusVoltage();
}

public double intakeMotorTemp(){
  return motor.getMotorTemperature();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
