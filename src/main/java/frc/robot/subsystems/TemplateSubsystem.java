// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TemplateSubsystem extends SubsystemBase {
  /* Instance Variables
   * This section should contain any variable that need to be accessed by your entire subsystem/class.
   * This usually includes any phyical motors or sensors that are a part of your subsystem,
   * as well as any data that needs to be accessed by your entire subsystem/class 
   * E.G. the height of the elevator or angle of the wrist.
   */

  /* Declaration
   * This is a variable declaration. It's type and name are specified but it is not given a value yet.
   * It should be given a value in the constructor.
   */
  CANSparkMax motor;
  
  /* Constructor
   * The Constructor is a special type of method that gets called when this subsystem/class is created,
   * usually when the robot turns on. You should give a value to most instance variables created above
   * in this method. You can also do anything else that you want to happen right away.
   */
  public TemplateSubsystem() {
    motor = new CANSparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
