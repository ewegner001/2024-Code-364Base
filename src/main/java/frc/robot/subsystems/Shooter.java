// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double lShooterMotorSGains = 0.0;
  private final double lShooterMotorVGains = 0.0;
  private final double lShooterMotorPGains = 0.0;
  private final double lShooterMotorIGains = 0.0;
  private final double lShooterMotorDGains = 0.0;
  private final double rShooterMotorSGains = 0.0;
  private final double rShooterMotorVGains = 0.12;
  private final double rShooterMotorPGains = 0.05;
  private final double rShooterMotorIGains = 0.0;
  private final double rShooterMotorDGains = 0.0;
  private final int m_CurrentLimit = 40;
  



  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFX frontShooterMotor;
  private DigitalInput inputSensor;
  private Slot0Configs slotConfigsR;
  private Slot0Configs slotConfigsL;
  private VelocityVoltage rm_request;
  private VelocityVoltage lm_request;
  private TalonFXConfigurator configL;
  private TalonFXConfigurator configR;
  private TalonFXConfigurator configF;

  
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new TalonFX(13);
    rightShooterMotor = new TalonFX(14);
    frontShooterMotor = new TalonFX(16);
    inputSensor = new DigitalInput(3);
    configL = leftShooterMotor.getConfigurator();

    configL.apply(
      new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(m_CurrentLimit)
    );

    configR.apply(
      new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(m_CurrentLimit)
    );

    configF.apply(
      new CurrentLimitsConfigs()
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(m_CurrentLimit)
    );

    slotConfigsR = new Slot0Configs();
    slotConfigsR.kS = rShooterMotorSGains;
    slotConfigsR.kV = rShooterMotorVGains;
    slotConfigsR.kP = rShooterMotorPGains; 
    slotConfigsR.kI = rShooterMotorIGains;
    slotConfigsR.kD = rShooterMotorDGains;
    rm_request = new VelocityVoltage(0).withSlot(0);
    slotConfigsL = new Slot0Configs();
    slotConfigsL.kS = lShooterMotorSGains;
    slotConfigsL.kV = lShooterMotorVGains;
    slotConfigsL.kP = lShooterMotorPGains;
    slotConfigsL.kI = lShooterMotorIGains;
    slotConfigsL.kD = lShooterMotorDGains;
    lm_request = new VelocityVoltage(0);

    leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);

    SmartDashboard.putNumber("RShooter kP", slotConfigsR.kP);
    SmartDashboard.putNumber("RShooter kV", slotConfigsR.kV);
    SmartDashboard.putNumber("RShooter Speed", 0);
    SmartDashboard.putNumber("LShooter kP", slotConfigsL.kP);
    SmartDashboard.putNumber("LShooter kV", slotConfigsL.kV);
    SmartDashboard.putNumber("LShooter Speed", 0);
    shootingMotorsConfig();
  }

  public void frontShooterIntake() {
    frontShooterMotor.setVoltage(12.0);  
  }

    public void frontShooterOutake() {
    frontShooterMotor.setVoltage(-3.0);  
  }

  // public void shooterIntakeStop() {
  //   if (inputSensor.get() == true) {
  //     frontShooterMotor.stopMotor();
  //   }
  // }

  public void frontRollersStop() {
    frontShooterMotor.setVoltage(0);
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

  public void shootingMotorsSetControl(double rightShooterSpeed, double leftShooterSpeed) {

    rightShooterMotor.setControl(rm_request.withVelocity(rightShooterSpeed));
    leftShooterMotor.setControl(lm_request.withVelocity(leftShooterSpeed));

  }

  public void setShooterVoltage(double rightVoltage, double leftVoltage) {
    rightShooterMotor.setVoltage(rightVoltage);
    leftShooterMotor.setVoltage(leftVoltage);
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
    SmartDashboard.putBoolean("Break Beam Sensor", sensorValue());
  }
}
