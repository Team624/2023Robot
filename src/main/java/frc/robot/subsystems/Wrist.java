// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wristMotor;

  private RelativeEncoder wristEncoder;

  private SparkMaxPIDController wristPidController;

  private double WristP;
  private double WristI;
  private double WristD;

  private DutyCycleEncoder WristboreEncoder;

  private Rotation2d rotationReference;

  private ArmFeedforward wristfeedforward =
      new ArmFeedforward(Constants.Wrist.kS, Constants.Arm.kG, Constants.Arm.kV);

  private ProfiledPIDController wristController;

  public Wrist() {

    wristMotor = new CANSparkMax(Constants.Wrist.WristMotor, MotorType.kBrushless);

    wristMotor.restoreFactoryDefaults();

    wristEncoder = wristMotor.getEncoder();

    wristMotor.setIdleMode(IdleMode.kBrake);

    wristPidController = wristMotor.getPIDController();

    WristP = frc.robot.Constants.Wrist.P;
    WristI = frc.robot.Constants.Wrist.I;
    WristD = frc.robot.Constants.Wrist.D;

    wristPidController.setP(WristP);
    wristPidController.setI(WristI);
    wristPidController.setD(WristD);

    wristMotor.setCANTimeout(500);

    wristPidController.setOutputRange(-1, 1);

    WristboreEncoder = new DutyCycleEncoder(1);

    wristController =
        new ProfiledPIDController(
            Constants.Wrist.P,
            Constants.Wrist.I,
            Constants.Wrist.D,
            Constants.Wrist.wristCONSTRAINTS);

    // wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // wristMotor.setSoftLimit(SoftLimitDirection.kForward, 0.01f);
    // wristMotor.setSoftLimit(SoftLimitDirection.kReverse, -228);

    rotationReference = getAbsoluteRotation();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/Wrist/Encoder", getWristEncoder());
    SmartDashboard.putNumber("/Arm/BoreEncoder/get", WristboreEncoder.get());
    SmartDashboard.putNumber("/Wrist/BoreEncoder/Absolute", WristboreEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("/Wrist/BoreEncoder/radians", getAbsoluteRotation().getRadians());
  }

  public void resetController() {
    wristController.reset(getAbsoluteRotation().getRadians());
  }

  public void setReference(double rotation) {
    wristController.setTolerance(Units.degreesToRadians(3));

    // 3.85 radians = mid
    // 4.24 raidans = high

    // 5.54 raidans ground intake
    double voltage = wristController.calculate(getAbsoluteRotation().getRadians(), rotation);

    System.out.println("Voltage: " + voltage);

    wristMotor.setVoltage(-voltage);
    System.out.println("Bore: " + getAbsoluteRotation().getRadians());
    System.out.println("Setpoint: " + rotation);
    if (wristController.atGoal()) {
      this.stopWrist();
    }
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBoreEncoder();

    return new Rotation2d(radians);
  }

  public double getBoreEncoder() {
    return (WristboreEncoder.getAbsolutePosition() + 0.02);
  }

  public void zeroBoreEncoder() {
    WristboreEncoder.reset();
  }

  public void moveWrist(double speed) {
    wristMotor.set(speed);
  }

  public void zeroWrist() {
    wristEncoder.setPosition(0.0);
  }

  public void stopWrist() {
    // rotationReference = getAbsoluteRotation();
    wristMotor.stopMotor();
  }

  public void setWristCommand(double setpoint) {

    Rotation2d angle = new Rotation2d(setpoint);

    wristPidController.setReference(setpoint, ControlType.kPosition, 0);
  }

  public double getWristEncoder() {
    return wristEncoder.getPosition();
  }
}
