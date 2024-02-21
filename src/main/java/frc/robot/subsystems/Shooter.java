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
  private final int breakBeamID = 3;

  // loader speeds
  public final double runLoaderVoltage = 12.0;
  public final double reverseLoaderVoltage = -3.0;
  public final double stopLoaderVoltage = 0.0;

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
  private TalonFX m_leftShooter;
  private TalonFX m_rightShooter;
  private TalonFX m_loader;

  private Slot0Configs slotConfigsR;
  private Slot0Configs slotConfigsL;

  private VelocityVoltage rm_request;
  private VelocityVoltage lm_request;

  private DigitalInput breakBeam;


  // constructor
  public Shooter() {

    // instantiate objects

    // motors
    m_leftShooter = new TalonFX(leftShooterMotorID);
    m_rightShooter = new TalonFX(rightShooterMotorID);
    m_loader = new TalonFX(loaderMotorID);

    // break beam sensor
    breakBeam = new DigitalInput(breakBeamID);

    // right shooter motor configuration
    slotConfigsR = new Slot0Configs();
    slotConfigsR.kS = rShooterMotorSGains;
    slotConfigsR.kV = rShooterMotorVGains;
    slotConfigsR.kP = rShooterMotorPGains; 
    slotConfigsR.kI = rShooterMotorIGains;
    slotConfigsR.kD = rShooterMotorDGains;

    // left shooter motor configuration
    slotConfigsL = new Slot0Configs();
    slotConfigsL.kS = lShooterMotorSGains;
    slotConfigsL.kV = lShooterMotorVGains;
    slotConfigsL.kP = lShooterMotorPGains;
    slotConfigsL.kI = lShooterMotorIGains;
    slotConfigsL.kD = lShooterMotorDGains;

    // set shooter motors to coast mode
    m_leftShooter.setNeutralMode(NeutralModeValue.Coast);
    m_rightShooter.setNeutralMode(NeutralModeValue.Coast);

    // log data
    SmartDashboard.putNumber("RShooter kP", slotConfigsR.kP);
    SmartDashboard.putNumber("RShooter kV", slotConfigsR.kV);
    SmartDashboard.putNumber("RShooter Speed", 0);
    SmartDashboard.putNumber("LShooter kP", slotConfigsL.kP);
    SmartDashboard.putNumber("LShooter kV", slotConfigsL.kV);
    SmartDashboard.putNumber("LShooter Speed", 0);

    // configure shooter motors
    shootingMotorsConfig();

    // PID Velocity target
    rm_request = new VelocityVoltage(0);
    lm_request = new VelocityVoltage(0);
  }

  /*
   * This method will set the voltage of the loader.
   * It can be used to start and stop the loader.
   * 
   * parameters:
   * desired voltage      (double)
   * 
   * returns:
   * none
   */
  public void setLoaderVoltage(double loaderSpeedVoltage) {
    m_loader.setVoltage(loaderSpeedVoltage);  
  }

  /*
   * This method will get the output of the break beam sensor
   * as a boolean
   * 
   * parameters:
   * none
   * 
   * returns:
   * none
   */
  public boolean getBreakBeamOutput() {
    return breakBeam.get();
  }

  /*
   * This method will configure the left and right shooting motors.
   * 
   * parameters:
   * none
   * 
   * returns:
   * none
   */
  public void shootingMotorsConfig() {

    // get configurations
    slotConfigsR.kP = SmartDashboard.getNumber("RShooter kP", 0);
    slotConfigsR.kV = SmartDashboard.getNumber("RShooter kV", 0);
    slotConfigsL.kP = SmartDashboard.getNumber("LShooter kP", 0);
    slotConfigsL.kV = SmartDashboard.getNumber("LShooter kV", 0);

    // set configurations to motors
    m_rightShooter.getConfigurator().apply(slotConfigsR);
    m_leftShooter.getConfigurator().apply(slotConfigsL);
  }

  public void shootingMotorsSetControl(double rightShooterSpeed, double leftShooterSpeed) {

    m_rightShooter.setControl(rm_request.withVelocity(rightShooterSpeed));
    m_leftShooter.setControl(lm_request.withVelocity(leftShooterSpeed));

  }

  public void setShooterVoltage(double rightVoltage, double leftVoltage) {
    m_rightShooter.setVoltage(rightVoltage);
    m_leftShooter.setVoltage(leftVoltage);
  }

  public void stop() {
    m_rightShooter.setControl(rm_request.withVelocity(0));
    m_leftShooter.setControl(rm_request.withVelocity(0));
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
    SmartDashboard.putNumber("Right Shooter Speed", m_rightShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Speed", m_leftShooter.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Shooter Temp", m_rightShooter.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Left Shooter Temp", m_leftShooter.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putBoolean("Break Beam Sensor", getBreakBeamOutput());
  }
}
