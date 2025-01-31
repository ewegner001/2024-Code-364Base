/*
 * This subsytem controls the shooter angle.
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {

  // local constants

  // IDs
  private final int shooterPivotMotorID = 12;
  private final int shooterPivotCANCoderID = 15;

  // positions
  public final double shooterPivotStowPosition = 115.0;
  public final double shooterPivotIntakePosition = 136.75;

  // pivot motor PID
  private final double shooterPivotPGains = 0.3;
  private final double shooterPivotIGains = 0;
  private final double shooterPivotDGains = 0;
  private final double shooterPivotGGains = 0;

  private final double shooterPivotGearRatio = 54.545;
  private final double magnetOffset = 0.0;

  // WPILib class objects
  private CANSparkMax m_ShooterPivot;
  private CANcoder e_ShooterPivot;
  private double m_setPoint;
  private PIDController shooterPivotPID;
  private ArmFeedforward shooterPivotFeedForward;
  private RelativeEncoder e_ShooterPivotIntegrated;
  private double m_ShooterPivotVoltage;

  // constructor
  public ShooterPivot() {
    
    // instantiate objects

    m_ShooterPivot = new CANSparkMax(shooterPivotMotorID, MotorType.kBrushless);
    e_ShooterPivot = new CANcoder(shooterPivotCANCoderID);

    CANcoderConfigurator cancoderConfigurator = e_ShooterPivot.getConfigurator();
    cancoderConfigurator.apply(
      new MagnetSensorConfigs()
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withMagnetOffset(magnetOffset)
    );

    shooterPivotPID = new PIDController(shooterPivotPGains, shooterPivotIGains, shooterPivotDGains);
    shooterPivotFeedForward = new ArmFeedforward(0, shooterPivotGGains, 0, 0);


    // set integrated encoder to cancoder position
    e_ShooterPivotIntegrated = m_ShooterPivot.getEncoder();

    // configure integrated ercoder
    e_ShooterPivotIntegrated.setPositionConversionFactor(360 / shooterPivotGearRatio);
    e_ShooterPivotIntegrated.setPosition(cancoderInDegrees());
    e_ShooterPivotIntegrated.setVelocityConversionFactor(360 / shooterPivotGearRatio);

    // set shooter angle to safe position on startup
    moveShooterPivot(shooterPivotStowPosition);
  }

  /*
   * This method will move the move the shooter pivot motor 
   * to a point provided as a parameter.
   * 
   * parameters:
   * setpoint         (double)
   * 
   * returns:
   * none
   */
  public void moveShooterPivot(double setPoint) {
    m_setPoint = setPoint;
  }

  /*
   * This method will get the postition of the cancoder and
   * return it in degrees
   * 
   * parameters:
   * none
   * 
   * returns:
   * cancoder degrees
   */
  private double cancoderInDegrees() {
    return e_ShooterPivot.getPosition().getValue() * 360;
  }



 
  @Override
  public void periodic() {
    //double feedForward = shooterPivotFeedforward.calculate(Units.degreesToRadians(shooterPivotMotorEncoder.getPosition()), Units.degreesToRadians(shooterPivotMotorEncoder.getVelocity()));

    // constantly move shooter pivot to the setpoint
    m_ShooterPivotVoltage = shooterPivotPID.calculate(cancoderInDegrees(), m_setPoint); //m_IntakePivotVoltage = pid.calculate(intakePivotMotorEncoder.getPosition(), m_setPoint) + feedForward;
    m_ShooterPivot.setVoltage(m_ShooterPivotVoltage);

    // log data
    SmartDashboard.putNumber("Shooter Voltage", m_ShooterPivotVoltage);
    SmartDashboard.putNumber("Shooter CANcoder", cancoderInDegrees());
    SmartDashboard.putNumber("Shooter Pivot Motor Position", e_ShooterPivotIntegrated.getPosition());
    SmartDashboard.putNumber("Shooter setpoint", m_setPoint);
  }
}
