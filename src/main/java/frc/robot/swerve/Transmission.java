// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import java.util.Map;
import java.util.TreeMap;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Transmission extends TrapezoidProfileSubsystem {
  private static final boolean MOTOR_INVERTED = false;
  private final CANSparkMax m_motor;
  private final SparkPIDController m_controller;
  private final RelativeEncoder m_motorEncoder;

  private static final double PID_PROPORTIONAL = 1.0;
  private static final double CM_PER_REVOLUTION = 1.0;
  // we will need to find this it is the distance that the belt will move with one
  // turn of the motor
  private static final double CURRENT_LIMIT = 20.0;
  private static final double MAX_VEL_PER_SEC = 1;
  private static final double MAX_ACCEL_PER_SEC = 1;

  static final TreeMap<Double, Double> CSVGearRatiosToMotorPose = new TreeMap<>(Map.ofEntries(
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null)));
  
  //this is the inverse of the one above 
  static final TreeMap<Double, Double> MotorPoseToCSVGearRatios = new TreeMap<>(Map.ofEntries(
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null),
      Map.entry(null, null)));

  

  static void checkNeoError(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
      // System.out.println(String.format("%s: %s", message, error.toString()));
    }
  }

  public Transmission(int canId, int canCoderId) {
    super(new TrapezoidProfile.Constraints(MAX_VEL_PER_SEC, MAX_ACCEL_PER_SEC));

    m_motor = new CANSparkMax(canId, CANSparkLowLevel.MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motorEncoder = m_motor.getEncoder();

    m_motorEncoder.setPositionConversionFactor(CM_PER_REVOLUTION);

    // adjust the CANbus update periods and alert on any errors
    checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100),
        "Failed to set periodic status frame 0 rate");
    checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate");
    checkNeoError(m_motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20),
        "Failed to set periodic status frame 2 rate");

    // set turn moter to brake mode
    checkNeoError(m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
    m_motor.setInverted(!MOTOR_INVERTED);

    // enable voltage compensation and current limit
    checkNeoError(m_motor.enableVoltageCompensation(Constants.MAX_VOLTAGE), "Failed to enable voltage compensation");
    checkNeoError(m_motor.setSmartCurrentLimit((int) Math.round(CURRENT_LIMIT)),
        "Failed to set NEO current limits");
    
    m_controller = m_motor.getPIDController();
    checkNeoError(m_controller.setP(PID_PROPORTIONAL), "Failed to set NEO PID proportional constant");

  }

  public void setTransmissionMotorPose(double distance) {
    super.setGoal(limitMotorDistance(distance));
  }

  public static double limitMotorDistance(double distance) {
    return MathUtil.clamp(distance, 0, 1);// 1 is a guess
  }

  public void setTransmissionGearRatio(double transmissionGearRatio) {
    Map.Entry<Double, Double> before = CSVGearRatiosToMotorPose.floorEntry(transmissionGearRatio);
    Map.Entry<Double, Double> after = CSVGearRatiosToMotorPose.ceilingEntry(transmissionGearRatio);
    if (before == null) {
      if (after == null) {
        return; // this should never happen b/c shooterSpeeds should have at least 1 element
      }
      setTransmissionMotorPose(after.getValue());
      return;
    }
    if (after == null) {
      setTransmissionMotorPose(before.getValue());
      return;
    }

    double denom = after.getKey() - before.getKey();
    if (Math.abs(denom) < 0.1) {
      // distance must have exactly matched a key
      setTransmissionMotorPose(before.getValue());
      return;
    }

    double ratio = (transmissionGearRatio - before.getKey()) / denom;
    double motorPose = MathUtil.interpolate(before.getValue(), after.getValue(), ratio);
    setTransmissionMotorPose(motorPose);
  }

  public double getTransmisionGearRatio() {

    final double motorPose = m_motorEncoder.getPosition();



    return 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useState(State state) {
    // TODO Auto-generated method stub
    m_controller.setReference(setPoint.position, CANSparkMax.ControlType.kPosition);
  }
}
