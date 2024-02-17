// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private CANSparkMax motor;

  /** Creates a new Wrist. */
  public Wrist() {
    motor = new CANSparkMax(20, MotorType.kBrushless); //Change later q1
  }

  public void run(double speed) {
    motor.set(speed);
  }
  public double WristMotorTemp() {
    return motor.getMotorTemperature();
  }

  //@Override
  //public void periodic() {
    // This method will be called once per scheduler run
 // }
}
