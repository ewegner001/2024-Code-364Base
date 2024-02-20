// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
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
  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private SparkPIDController pidController; 

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
    motor1 = new CANSparkMax(0, MotorType.kBrushless);
    motor2 = new CANSparkMax(1, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    pidController = motor1.getPIDController();
    motor1.setSoftLimit(SoftLimitDirection.kForward, heightlimit);
    motor1.setSoftLimit(SoftLimitDirection.kReverse,restingposition);
    
    motor2.restoreFactoryDefaults();
    pidController = motor2.getPIDController();
    motor2.setSoftLimit(SoftLimitDirection.kForward, heightlimit);
    motor2.setSoftLimit(SoftLimitDirection.kReverse,restingposition);
    motor2.follow(motor1, false);

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
  motor1.set(elevatorspeed);
  }

    /* Sets the Target Elevator Position in inches.*/
    public void setTargetElevatorPosition(double inches){
      targetElevatorPosition = inches;
  }

  /* Sets the Target Elevator Position in inches.*/
  public double getTargetElevatorPosition(){
      return targetElevatorPosition;
  }


    private double motorRotationsToInches(double rotations) {
        return rotations * Constants.ELEVATOR_ROTATIONS_TO_IN;
    }

    private double inchesToMotorRotations(double inches) {
        return inches / Constants.ELEVATOR_ROTATIONS_TO_IN;
    }

    public boolean atPosition() {

        double error = Math.abs(elevatorEncoder.getPosition() - targetElevatorPosition);

        if (Constants.ELEVATOR_TOLERANCE >= error) {
            return true;

        } else {
            return false;
        }

        }

        public Boolean isHigh(){
          return getTargetElevatorPosition() == Constants.ELEVATOR_HIGH_LEVEL;
      }

  /* The below method is public, which means that it can be 
   * accessed outside of this class. It also takes in a 
   * single double value as a parameter. It then takes that 
   * value (called `number`) and multiplies it by 
   * `internalData` to change it's value.
   */
  public void stop() {
  motor1.set(0);
  }
/*This method makes the elevator go down until */
public void down() {
  motor1.set(elevatorspeed);
}
/*This method sets the elevator to a shooting position */
private void goToShootingPos(){
if (motor1.getEncoder().getPosition() == shootingPosition){
  stop();
}else if(motor1.getEncoder().getPosition() < shootingPosition){
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
    if (DriverStation.isEnabled()){
            // This method will be called once per scheduler run
            // TODO: Test that .getPosition() gives us the elevator position in inches
            double voltage = controller.calculate(elevatorEncoder.getPosition(), targetElevatorPosition);
            // double feedforward = ff.calculate(/*double */elevatorEncoder.getVelocity());
            MathUtil.clamp(voltage, -12, 12);

            elevatorMotor.setVoltage(voltage);

            
            SmartDashboard.putNumber("ELEVATOR PID VOLTAGE", voltage);
        }
        logData();
        SmartDashboard.putNumber("ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("Elevator Encoder Value: ", getEncoderPosition());
        
        logData();
      
  }
   
public Command SetElevatorPosition (double inches){
        return new InstantCommand(() -> setTargetElevatorPosition(inches), this);
    }

    public Command ElevatorAtPosition(){
        return Commands.waitUntil(() -> atPosition());
    }

    private void logData() {
        /* Elevator Motor */
        elevatorMotorTemperature.append(elevatorMotor.getMotorTemperature());
        elevatorMotorAppliedOutput.append(elevatorMotor.getAppliedOutput());
       // elevatorMotorBusVoltage.append(elevatorMotor.getBusVoltage());
        elevatorMotorOutputCurrent.append(elevatorMotor.getOutputCurrent());
      //  elevatorMotorClosedLoopRampRate.append(elevatorMotor.getClosedLoopRampRate());
    //    elevatorMotorOpenLoopRampRate.append(elevatorMotor.getOpenLoopRampRate());
        elevatorMotorFaults.append(elevatorMotor.getFaults());
    //    elevatorMotorIdleMode.append(elevatorMotor.getIdleMode().toString());
    //    elevatorMotorInverted.append(elevatorMotor.getInverted());
     //   elevatorMotorLastError.append(elevatorMotor.getLastError().toString());

        // /* Elevator Encoder */
        // elevatorEncoderPosition.append(getEncoderPosition());
        // elevatorEncoderVelocity.append(elevatorEncoder.getVelocity());
      }
    }