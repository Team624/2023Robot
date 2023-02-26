// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax armMotorRight;

  private CANSparkMax armMotorLeft;

  private SparkMaxAbsoluteEncoder alternateEncoder;

  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;

  private SparkMaxPIDController armSparkmaxPIDLeft;
  private SparkMaxPIDController armSparkmaxPIDRight;

  ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV);

  private double RkP;
  private double RkI;
  private double RkD;

  private double LkP;
  private double LkI;
  private double LkD;

  private MotorControllerGroup Armgroup;

  // private DutyCycleEncoder ArmboreEncoder;

  private ProfiledPIDController ArmProfiledPIDController =
      new ProfiledPIDController(0.05, 0.0, 0.0, new TrapezoidProfile.Constraints(0.8, 0.8));

  public Arm() {

    armMotorRight = new CANSparkMax(frc.robot.Constants.Arm.armMotorRight, MotorType.kBrushless);
    armMotorRight.restoreFactoryDefaults();
    armEncoderRight = armMotorRight.getEncoder();
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armSparkmaxPIDRight = armMotorRight.getPIDController();
    armMotorRight.setCANTimeout(500);

    RkP = frc.robot.Constants.Arm.RkP;
    RkI = frc.robot.Constants.Arm.RkI;
    RkD = frc.robot.Constants.Arm.RkD;

    armSparkmaxPIDRight.setP(RkP);
    armSparkmaxPIDRight.setI(RkI);
    armSparkmaxPIDRight.setD(RkD);

    armSparkmaxPIDRight.setOutputRange(-1, 1);

    armMotorLeft = new CANSparkMax(frc.robot.Constants.Arm.armMotorLeft, MotorType.kBrushless);
    armMotorLeft.restoreFactoryDefaults();
    armEncoderLeft = armMotorLeft.getEncoder();
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setInverted(true);
    armSparkmaxPIDLeft = armMotorLeft.getPIDController();
    armMotorLeft.setCANTimeout(500);

    

    LkP = frc.robot.Constants.Arm.LkP;
    LkI = frc.robot.Constants.Arm.LkI;
    LkD = frc.robot.Constants.Arm.LkD;

    armSparkmaxPIDLeft.setP(LkP);
    armSparkmaxPIDLeft.setI(LkI);
    armSparkmaxPIDLeft.setD(LkD);

    armSparkmaxPIDLeft.setOutputRange(-1, 1);

    Armgroup = new MotorControllerGroup(armMotorRight, armMotorLeft);

    // ArmboreEncoder = new DutyCycleEncoder(0);

    alternateEncoder = armMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
    armSparkmaxPIDRight.setFeedbackDevice(alternateEncoder);

    double maxVel = 0.0;
    double maxAcc = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("/Arm/Encoder/Right", getArmEncoderRight());
    SmartDashboard.putNumber("/Arm/Encoder/Left", getArmEncoderLeft());
    // SmartDashboard.putNumber("/Arm/BoreEncoder/get", ArmboreEncoder.get());
    // SmartDashboard.putNumber("/Arm/BoreEncoder/Absolute", ArmboreEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("/Arm/BoreEncoder/Distance", ArmboreEncoder.getDistance());
    SmartDashboard.putNumber("/Arm/alternate/", alternateEncoder.getPosition());
    // ArmboreEncoder.setPositionOffset(0.23);
  }

  public void controlArmRight(double speed) {
    armMotorRight.set(speed);
  }

  public double getBoreEncoder() {
    return alternateEncoder.getPosition();
    
  }

  // public void zeroBoreEncoder() {
  //   ArmboreEncoder.reset();
  // }

  public void controlArmLeft(double speed) {
    armMotorLeft.set(speed);
  }

  public void controlArm(double speed) {

    armMotorRight.set(speed);
    armMotorLeft.set(speed);
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public void stopArm() {
    armMotorRight.stopMotor();
    armMotorLeft.stopMotor();
  }

  public void setArmCommand(double setpoint) {
    System.out.println("setpoint in command: " + setpoint);
    Rotation2d angle = new Rotation2d(setpoint);

    // armMotorLeft.setVoltage(setpoint + feedforward.calculate(angle.getRadians(), 0));
    // armMotorRight.setVoltage(setpoint + feedforward.calculate(angle.getRadians(), 0));

    armSparkmaxPIDRight.setReference(
        setpoint, ControlType.kPosition, 0, feedforward.calculate(angle.getRadians(), 0));

    armSparkmaxPIDLeft.setReference(
        setpoint, ControlType.kPosition, 0, feedforward.calculate(angle.getRadians(), 0));

  }

  public void ArmProfile(TrapezoidProfile.State setpoint) {

    System.out.println(setpoint.position);
    Rotation2d setpoint2d = new Rotation2d(setpoint.position);

    armSparkmaxPIDLeft.setReference(1 - setpoint.position, ControlType.kPosition, 0);

    armSparkmaxPIDRight.setReference(1 - setpoint.position, ControlType.kPosition, 0);
  }

  public void resetEncoder() {
    armEncoderRight.setPosition(0.0);
    armEncoderLeft.setPosition(0.0);
  }

  public void goToSetpoint(double setpoint) {

    double pidVal = ArmProfiledPIDController.calculate(getBoreEncoder(), setpoint);
    Rotation2d rot = new Rotation2d(ArmProfiledPIDController.getSetpoint().position);
    // armMotorLeft.setVoltage(pidVal + feedforward.calculate(rot.getRadians(), 0));

    // armMotorRight.setVoltage(pidVal + feedforward.calculate(rot.getRadians(), 0));
  }
}
