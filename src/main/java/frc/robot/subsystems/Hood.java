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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Hood extends ProfiledPIDSubsystem {
  /** Creates a new Hood. */
  private final CANSparkMax hoodMotor;

  private DutyCycleEncoder boreEncoder;
  private ArmFeedforward hoodFeedForward;

  private double voltage = 0.0;

  private ShuffleboardTab hoodTab;

  private GenericEntry enabledEntry;
  private GenericEntry voltageEntry;

  private GenericEntry positionEntry;
  private GenericEntry setpointEntry;
  private GenericEntry goalEntry;

  public Hood() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            Constants.Hood.kP,
            Constants.Hood.kI,
            Constants.Hood.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(4, 4)));
    getController().setTolerance(Units.degreesToRadians(3));
    hoodMotor = new CANSparkMax(Constants.Hood.hoodMotor, MotorType.kBrushless);
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setCANTimeout(500);

    boreEncoder = new DutyCycleEncoder(2);
    hoodFeedForward =
        new ArmFeedforward(
            Constants.Hood.kS, Constants.Hood.kG, Constants.Hood.kV, Constants.Hood.kA);

    enabledEntry =
        hoodTab.add("Enabled", m_enabled).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    positionEntry = hoodTab.add("Position", getAbsoluteRotation().getDegrees()).getEntry();

    voltageEntry = hoodTab.add("Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).getEntry();
    setpointEntry = hoodTab.add("Setpoint", getController().getSetpoint().position).getEntry();
    goalEntry = hoodTab.add("Goal", getController().getGoal().position).getEntry();
  }

  @Override
  public void periodic() {
    super.periodic();

    enabledEntry.setBoolean(m_enabled);
    positionEntry.setDouble(getBore());
    setpointEntry.setDouble(getController().getGoal().position);
    goalEntry.setDouble(getController().getGoal().position);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    if (this.m_enabled) {
      // voltage = armFeedForward.calculate(1.5 * Math.PI - setpoint.position, setpoint.velocity);

      voltage = output;
      // + hoodFeedForward.calculate(0.5 * Math.PI - setpoint.position, setpoint.velocity);

      voltage = MathUtil.clamp(voltage, -9.0, 9.0);

      voltageEntry.setDouble(voltage);

      hoodMotor.setVoltage(voltage);
    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAbsoluteRotation().getRadians();
  }

  public double getBore() {
    return MathUtil.inputModulus(
        boreEncoder.getAbsolutePosition() + Constants.Hood.BORE_ENCODER_OFFSET, 0.0, 1.0);
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBore();

    return new Rotation2d(radians);
  }

  public void setGoal(Rotation2d rotation) {
    this.setGoal(rotation.getRadians());
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  public void set(double speed) {
    disable();
    hoodMotor.set(speed);
  }
}
