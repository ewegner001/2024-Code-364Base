// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private int ShooterPiviotMotorID = 0;
  private int ShooterPiviotCANCoderID = 0;
  private double ShooterPiviotPGains = 0;
  private double ShooterPiviotIGains = 0;
  private double ShooterPiviotDGains = 0;
  private double ShooterPiviotGGains = 0;

  private CANSparkMax motor;
  private CANcoder cancoder;
  private double m_setPoint;
  private PIDController pid;
  private ArmFeedforward ff;
  /** Creates a new ShooterPiviot. */
  public ShooterPivot() {
    motor = new CANSparkMax(ShooterPiviotMotorID, MotorType.kBrushless);
    cancoder = new CANcoder(ShooterPiviotCANCoderID);
    m_setPoint = 0;
    pid = new PIDController(ShooterPiviotPGains, ShooterPiviotIGains, ShooterPiviotDGains);
    this.ff = new ArmFeedforward(0, ShooterPiviotGGains, 0, 0);

  }

  private double encoderAngle() {
    StatusSignal<Double> position = cancoder.getPosition();
    Double positionValue = position.getValue();
    return positionValue.doubleValue();
  }

  public void moveShooterPivot(double setPoint) {
    m_setPoint = setPoint;
  }

  private double encoderPosition() {
    StatusSignal<Double> aPosition = cancoder.getAbsolutePosition();
    Double aPositionValue = aPosition.getValue();
    return aPositionValue.doubleValue();
  }

  private double encoderVelocity() {
    StatusSignal<Double> velocity = cancoder.getVelocity();
    Double velocityValue = velocity.getValue();
    return velocityValue.doubleValue();
  }
 
 
 
  @Override
  public void periodic() {
    double feedForward = ff.calculate(Units.degreesToRadians(encoderPosition()), Units.degreesToRadians(encoderVelocity()));
    motor.setVoltage(pid.calculate(encoderAngle(), m_setPoint) + feedForward);
    // This method will be called once per scheduler run
  }
}
