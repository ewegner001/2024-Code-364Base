// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFX frontShooterMotor;
  private DigitalInput inputSensor;
  private Slot0Configs slotConfigs;
  private TrapezoidProfile motorProfile;
  private TrapezoidProfile.State motorGoal;
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new TalonFX(0);
    rightShooterMotor = new TalonFX(0);
    frontShooterMotor = new TalonFX(0);
    inputSensor = new DigitalInput(0);
    slotConfigs = new Slot0Configs();
    slotConfigs.kS = 0.05;
    slotConfigs.kV = 0.12; 
    slotConfigs.kA = 0.01;
    slotConfigs.kP = 0.11; 
    slotConfigs.kI = 0;
    slotConfigs.kD = 0;
    motorProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(400, 4000)
    );
    
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

  public void shootingMotorsConfig() {
    rightShooterMotor.getConfigurator().apply(slotConfigs);
    rightShooterMotor.setControl()
  }

  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
