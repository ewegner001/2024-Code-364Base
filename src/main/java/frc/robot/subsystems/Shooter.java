/*
 * This method controls the shooting mechanism. It
 * manages both the shooter and the loader.
 * 
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  // local constants
  
  //IDs
  private final int leftShooterMotorID = 13;
  private final int rightShooterMotorID = 14;
  private final int loaderMotorID = 16;

  // left shooter motor PID
  private final double lShooterMotorPGains = 0.05;
  private final double lShooterMotorIGains = 0.0;
  private final double lShooterMotorDGains = 0.0;
  private final double lShooterMotorSGains = 0.0;
  private final double lShooterMotorVGains = 0.12;

  // right shooter motor PID
  private final double rShooterMotorPGains = 0.05;
  private final double rShooterMotorIGains = 0.0;
  private final double rShooterMotorDGains = 0.0;
  private final double rShooterMotorSGains = 0.0;
  private final double rShooterMotorVGains = 0.12;

  // WPILib class objects
  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFX loaderMotor;

  private Slot0Configs slotConfigsR;
  private Slot0Configs slotConfigsL;

  private VelocityVoltage rm_request;
  private VelocityVoltage lm_request;

  private DigitalInput inputSensor;


  // constructor
  public Shooter() {

    // instantiate objects
    leftShooterMotor = new TalonFX(leftShooterMotorID);
    rightShooterMotor = new TalonFX(rightShooterMotorID);
    loaderMotor = new TalonFX(loaderMotorID);

    inputSensor = new DigitalInput(3);

    slotConfigsR = new Slot0Configs();
    slotConfigsR.kS = rShooterMotorSGains;
    slotConfigsR.kV = rShooterMotorVGains;
    slotConfigsR.kP = rShooterMotorPGains; 
    slotConfigsR.kI = rShooterMotorIGains;
    slotConfigsR.kD = rShooterMotorDGains;


    slotConfigsL = new Slot0Configs();
    slotConfigsL.kS = lShooterMotorSGains;
    slotConfigsL.kV = lShooterMotorVGains;
    slotConfigsL.kP = lShooterMotorPGains;
    slotConfigsL.kI = lShooterMotorIGains;
    slotConfigsL.kD = lShooterMotorDGains;

    rm_request = new VelocityVoltage(0).withSlot(0);
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

  public void runLoader() {
    loaderMotor.setVoltage(12.0);  
  }

    public void reverseLoader() {
    loaderMotor.setVoltage(-3.0);  
  }

  public void stopLoader() {
    loaderMotor.setVoltage(0);
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
