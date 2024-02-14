// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  /* Instance Variables
   * This section should contain any variables that need to 
   * be accessed by your entire subsystem/class. This usually includes any phyical motors or sensors that are a part of your subsystem,
   * as well as any data that needs to be accessed by your 
   * entire subsystem/class E.G. the height of the elevator 
   * or angle of the wrist.
   */

  /* These are variable declarations. Their access modifiers,
   * types, and names are specified but they are not given a 
   * value yet. They should be given a value in the 
   * constructor. In this example, they is private, which 
   * means that nothing outside of this class can access it.
   */
  private CANSparkMax motor;
  private SparkPIDController pidController;
  private double internalData;

  private float heightlimit = 10;
  public double elevatorspeed = 1.0;
  public float restingposition = 0;
  public double shootingPosition = 10;
  
  /* Constructor
   * The Constructor is a special type of method that gets 
   * called when this subsystem/class is created, usually 
   * when the robot turns on. You should give a value to most
   * instance variables created above in this method. You can
   * also do anything else that you want to happen right away.
   */
  public Elevator() {
    /* These are variable initializations, where a variable 
     * is given a value. In this case, the `new` keyword is 
     * used to create a new CANSparkMax object and give it 
     * to `motor` as it's value. 
     */
    motor = new CANSparkMax(0, MotorType.kBrushless);
    internalData = 0.4;
    motor.restoreFactoryDefaults();
    pidController = motor.getPIDController();
    motor.setSoftLimit(SoftLimitDirection.kForward, heightlimit);
    motor.setSoftLimit(SoftLimitDirection.kReverse,restingposition);

    double kP = 0.1; 
    double kI = 1e-4;
    double kD = 1; 
    double kIz = 0; 
    double kFF = 0; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /*This method tells the robot how many rotations are needed for moving the elevator. SetReference is the 
  number of, in this case, rotations needed to be executed by the robot
  */
   
  public void SetReference(double rotations){
  
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

  /* Methods
   * The below section should contain any methods you want 
   * to use in your class.
   */

   /* This method makes the elevator go up until it hits the soft limit*/
  public void lift() {
  motor.set(elevatorspeed);
  }

  /* The below method is public, which means that it can be 
   * accessed outside of this class. It also takes in a 
   * single double value as a parameter. It then takes that 
   * value (called `number`) and multiplies it by 
   * `internalData` to change it's value.
   */
  public void stop() {
  motor.set(0);
  }
/*This method makes the elevator go down until */
public void down() {
  motor.set(elevatorspeed);
}
/*This method sets the elevator to a shooting position */
private void goToShootingPos(){
if (motor.getEncoder().getPosition() == shootingPosition){
  stop();
}else if(motor.getEncoder().getPosition() < shootingPosition){
  lift();
}else{
  down();
}
}




  /* The below method is included in every Subsystem. You can
   * think of it as an infinite loop that runs constantly 
   * while the robot is on.
   * 
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
