// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFX frontShooterMotor;
  private DigitalInput inputSensor;
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new TalonFX(0);
    rightShooterMotor = new TalonFX(0);
    frontShooterMotor = new TalonFX(0);
    inputSensor = new DigitalInput(0);
  }

  public void frontShooterIntake() {
    frontShooterMotor.setVoltage(12);  
  }

  // public void shooterIntakeStop() {
  //   if (inputSensor.get() == true) {
  //     frontShooterMotor.stopMotor();
  //   }
  // }

  public void frontRollersStop() {
    frontShooterMotor.stopMotor();
  }

  public boolean sensorValue() {
    return inputSensor.get();
  }

  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
