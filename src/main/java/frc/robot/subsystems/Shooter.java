// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    leftShooterMotor = new TalonFX(10);
    rightShooterMotor = new TalonFX(11);
    frontShooterMotor = new TalonFX(17);
    inputSensor = new DigitalInput(0);
    slotConfigsR = new Slot0Configs();
    slotConfigsR.kS = 0.05; //0.05
    slotConfigsR.kV = 0.12; // 0.12 
    slotConfigsR.kP = 0.12; 
    slotConfigsR.kI = 0;
    slotConfigsR.kD = 0;
    rm_request = new VelocityVoltage(0).withSlot(0);
    slotConfigsL = new Slot0Configs();
    slotConfigsL.kS = 0.05;
    slotConfigsL.kV = 0.12;
    slotConfigsL.kP = 0.12;
    slotConfigsL.kI = 0;
    slotConfigsL.kD = 0;
    lm_request = new VelocityVoltage(0);

    SmartDashboard.putNumber("RShooter kP", slotConfigsR.kP);
    SmartDashboard.putNumber("RShooter kV", slotConfigsR.kV);
    SmartDashboard.putNumber("RShooter Speed", 0);
    SmartDashboard.putNumber("LShooter kP", slotConfigsL.kP);
    SmartDashboard.putNumber("LShooter kV", slotConfigsL.kV);
    SmartDashboard.putNumber("LShooter Speed", 0);
    shootingMotorsConfig();
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
    slotConfigsR.kP = SmartDashboard.getNumber("RShooter kP", 0);
    slotConfigsR.kV = SmartDashboard.getNumber("RShooter kV", 0);
    slotConfigsL.kP = SmartDashboard.getNumber("LShooter kP", 0);
    slotConfigsL.kV = SmartDashboard.getNumber("LShooter kV", 0);
    rightShooterMotor.getConfigurator().apply(slotConfigsR);
    leftShooterMotor.getConfigurator().apply(slotConfigsL);
  }

  public void shootingMotorsSetControl() {
    double RShooterSpeed = SmartDashboard.getNumber("RShooter Speed", 0);
    double LShooterSpeed = SmartDashboard.getNumber("LShooter Speed", 0);
    rightShooterMotor.setControl(rm_request.withVelocity(RShooterSpeed));
    leftShooterMotor.setControl(lm_request.withVelocity(LShooterSpeed));

  }

  public void stop() {
    rightShooterMotor.setControl(rm_request.withVelocity(0));
    leftShooterMotor.setControl(rm_request.withVelocity(0));
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
    SmartDashboard.putNumber("Right Shooter Speed", rightShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Speed", leftShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Temp", rightShooterMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Temp", leftShooterMotor.getDeviceTemp().getValueAsDouble());
  }
}
