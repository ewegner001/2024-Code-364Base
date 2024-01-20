// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class intake extends SubsystemBase {
  private CANSparkMax motor;
  
  /** Creates a new intake. */
  public intake() {
    motor = new CANSparkMax(deviceId:1, MotorType.kBrushless);
  }

  public void run() {
    motor.set(speed:1.0);
  }

  public double intakeMotorTemp() {
    return motor.getMotorTemperature();
  }



  //@Override
  //public void periodic() {
    // This method will be called once per scheduler run
  //}
}
