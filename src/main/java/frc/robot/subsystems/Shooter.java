// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFX frontShooterMotor;
  private DigitalInput inputSensor;
  private Slot0Configs slotConfigsR;
  private Slot0Configs slotConfigsL;
  private VelocityVoltage rm_request;
  private VelocityVoltage lm_request;
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new TalonFX(0);
    rightShooterMotor = new TalonFX(0);
    frontShooterMotor = new TalonFX(0);
    inputSensor = new DigitalInput(0);
    slotConfigsR = new Slot0Configs();
    slotConfigsR.kS = 0.05;
    slotConfigsR.kV = 0.12; 
    slotConfigsR.kP = 0.11; 
    slotConfigsR.kI = 0;
    slotConfigsR.kD = 0;
    rm_request = new VelocityVoltage(0).withSlot(0);
    slotConfigsL = new Slot0Configs();
    slotConfigsL.kS = 0;
    slotConfigsL.kV = 0;
    slotConfigsL.kP = 0;
    slotConfigsL.kI = 0;
    slotConfigsL.kD = 0;
    lm_request = new VelocityVoltage(0);
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
    rightShooterMotor.getConfigurator().apply(slotConfigsR);
    leftShooterMotor.getConfigurator().apply(slotConfigsL);
  }

  public void shootingMotorsSetControl() {
    rightShooterMotor.setControl(rm_request.withVelocity(0));
    leftShooterMotor.setControl(lm_request.withVelocity(0));
  }

  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
