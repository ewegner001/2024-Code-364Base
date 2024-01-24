// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPiviot extends SubsystemBase {
  private CANSparkMax motor;
  private CANcoder cancoder;
  /** Creates a new ShooterPiviot. */
  public ShooterPiviot() {
    motor = new CANSparkMax(1, MotorType.kBrushless);
    cancoder = new CANcoder(0);
    }

    public double getValue() {
      return cancoder.getPosition().;
    }
public void run() {
PIDController pid = new PIDController(0, 0, 0);
motor.set(pid.calculate(cancoder.getPosition(), 10));
}
 
 
 
 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
