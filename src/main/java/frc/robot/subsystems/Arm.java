// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Arm extends ProfiledPIDSubsystem {
  /** Creates a new Arm. */
  private CANSparkMax armMotor;

  private DutyCycleEncoder boreEncoder;

  private ArmFeedforward armFeedForward;

  private ShuffleboardTab armTab;

  private double voltage = 0;

  private GenericEntry positionEntry;
  private GenericEntry enabledEntry;
  private GenericEntry setpointEntry;
  private GenericEntry voltageEntry;

  public Arm() {
    super(new ProfiledPIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD, new TrapezoidProfile.Constraints(Constants.Arm.kMaxVelocityRadiansPerSecond, Constants.Arm.kMaxAccelerationRadiansPerSecondSquared)));

    armMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);

    boreEncoder = new DutyCycleEncoder(Constants.Arm.BORE_ENCODER_PORT);

    armFeedForward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV, Constants.Arm.kA);

    armTab = Shuffleboard.getTab("Arm");

    positionEntry = armTab.add("Position (Bore)", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    enabledEntry = armTab.add("Enabled", m_enabled).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    setpointEntry = armTab.add("Setpoint", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    voltageEntry = armTab.add("Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).getEntry();
  }

  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
    positionEntry.setDouble(getAbsoluteRotation().getDegrees());
    enabledEntry.setBoolean(m_enabled);
    setpointEntry.setDouble(getController().getGoal().position * (180 / Math.PI));
  }

  public double getBore() {
    return MathUtil.inputModulus(boreEncoder.getAbsolutePosition() - Constants.Arm.BORE_ENCODER_OFFSET, 0.0, 1.0);
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBore();

    return new Rotation2d(radians);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    System.out.println("RUNNING!!!!!!!!!!!!!!!!\n");
    if (this.m_enabled) {
      // voltage = armFeedForward.calculate(1.5 * Math.PI - setpoint.position, setpoint.velocity);

      voltage = output + armFeedForward.calculate(0.5 * Math.PI - setpoint.position, setpoint.velocity);
      System.out.println("Voltage: " + voltage);
      voltageEntry.setDouble(voltage);
      armMotor.setVoltage(voltage);
    }
  }

  @Override
  protected double getMeasurement() {
    return getAbsoluteRotation().getRadians();
  }

  public void setGoal(Rotation2d rotation) {
    this.setGoal(rotation.getRadians());
  }

  public double getArmEncoderRight() {
    return armEncoderRight.getPosition();
  }

  public double getArmEncoderLeft() {
    return armEncoderLeft.getPosition();
  }

  public void stopArm() {
    rotationReference = getAbsoluteRotation();
    armMotorRight.stopMotor();
    armMotorLeft.stopMotor();
  }

  public void setArmCommand(double setpoint) {
    System.out.println("setpoint in command: " + setpoint);
    Rotation2d angle = new Rotation2d(setpoint);
    // feedforward.calculate(angle.getRadians(), 0)

    armMotorLeft.setVoltage(setpoint);
    armMotorRight.setVoltage(setpoint);
  }

  public void ArmProfile(TrapezoidProfile.State setpoint) {

    System.out.println(setpoint.position);
    Rotation2d setpoint2d = new Rotation2d(setpoint.position);

    armSparkmaxPIDLeft.setReference(setpoint.position, ControlType.kPosition, 0);

    armSparkmaxPIDRight.setReference(setpoint.position, ControlType.kPosition, 0);
  }

  public void resetEncoder() {
    armEncoderRight.setPosition(0.0);
    armEncoderLeft.setPosition(0.0);
  public void setSpeed(double speed) {
    disable();
    armMotor.set(speed);
  }

  public void setReference(Rotation2d rotation) {
    this.rotationReference = rotation;
  }
}
