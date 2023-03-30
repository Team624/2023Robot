// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Wrist extends ProfiledPIDSubsystem {
  /** Creates a new Wrist2. */
  private CANSparkMax wristMotor;

  private RelativeEncoder wristEncoder;

  private DutyCycleEncoder WristboreEncoder;

  private ShuffleboardTab wristTab;

  private GenericEntry voltageEntry;
  private GenericEntry positionEntry;
  private GenericEntry setpointEntry;
  private GenericEntry goalEntry;
  private GenericEntry velocityEntry;
  private GenericEntry velocitySetpointEntry;

  private Rotation2d prevPosition;
  private double prevTime;

  private int rotations;
  private double prevRadians;


  /** If 0 Can go -180 to 180 */

  /** If 90 180 - Counter clockwise -180 - Clock wise 90 0 - counter clockwise */

  /** If 180 0 - Clockwise 90 - clockwise 180 -90 - ClockWise -180 - Clockwise */

  /**
   * If -90 0 - Counter clock wise 90 - Counter clock wise 180 - Counter clock wise -90 - -180 -
   * Clockwise
   */

  /** If -180 0 - clock wise 90 - clock wise 180 - clock wise -90 - clock wise -180 - */
  private double voltage = 0;

  public Wrist() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            Constants.Wrist.P,
            Constants.Wrist.I,
            Constants.Wrist.D,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(7.5, 12.5)));
    getController().setTolerance(Units.degreesToRadians(3));
    wristMotor = new CANSparkMax(Constants.Wrist.WristMotor, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();

    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.setCANTimeout(500);

    // wristMotor.setSmartCurrentLimit(5);

    WristboreEncoder = new DutyCycleEncoder(1);

    prevPosition = getAbsoluteRotation();
    prevTime = Timer.getFPGATimestamp();

    wristTab = Shuffleboard.getTab("Wrist");
    positionEntry = wristTab.add("Positionning degrees", getBoreEncoder()).getEntry();

    voltageEntry = wristTab.add("Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).getEntry();
    setpointEntry = wristTab.add("Setpoint", getController().getSetpoint().position).getEntry();
    goalEntry = wristTab.add("Goal", getController().getGoal().position).getEntry();
    velocityEntry = wristTab.add("Velocity", 0).getEntry();
    velocitySetpointEntry = wristTab.add("Velocity Setpoint", 0.0).getEntry();

  }

  @Override
  public void periodic() {
    super.periodic();

    positionEntry.setDouble(getAbsoluteRotation().getDegrees());
    setpointEntry.setDouble(getBoreEncoder());
    goalEntry.setDouble(getController().getGoal().position);
    velocitySetpointEntry.setDouble(getController().getSetpoint().velocity);

    Rotation2d deltaPosition = getAbsoluteRotation().minus(prevPosition);
    double deltaTime = Timer.getFPGATimestamp() - prevTime;

    double degreesPerSecond = deltaPosition.getRadians() / deltaTime;
    velocityEntry.setDouble(degreesPerSecond);

    prevPosition = getAbsoluteRotation();
    prevTime = Timer.getFPGATimestamp();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here

    if (this.m_enabled) {
      voltage = output;

      voltage = MathUtil.clamp(voltage, -9.0, 9.0);

      // if (voltage < 0 && getAbsoluteRotation().getRadians() < 0) {
      //   voltage = 0;
      // }

      wristMotor.setVoltage(-voltage);
      voltageEntry.setDouble(-voltage);
    }
  }

  public double getBoreEncoder() {
    return (MathUtil.inputModulus(
        (1 - WristboreEncoder.getAbsolutePosition()) + Constants.Wrist.boreEncoderOffset, 0, 1));
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBoreEncoder();

    if (prevRadians >= 1.5 * Math.PI && radians <= 0.5 * Math.PI) {
      rotations += 1;
    } else if (prevRadians <= 0.5 * Math.PI && radians >= 1.5 * Math.PI) {
      rotations -= 1;
    }

    prevRadians = radians;

    return new Rotation2d(radians + rotations * 2 * Math.PI);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAbsoluteRotation().getRadians();
  }

  public void setGoal(Rotation2d rotation) {
    this.setGoal(rotation.getRadians());
  }

  public void set(double speed) {
    disable();
    // if (!softLimit(speed)) {
    //   wristMotor.set(speed);
    // }
    wristMotor.set(speed);
  }

  public double AbsoluteRadians() {
    double radians = 2 * Math.PI * getBoreEncoder();
    return radians;
  }

  public boolean softLimit(double value) {

    if (value > 0 && (getAbsoluteRotation().getDegrees() > 340)) {

      wristMotor.stopMotor();
      return true;
    }

    return false;
  }

  public void stopWrist() {
    wristMotor.stopMotor();
  }
}
