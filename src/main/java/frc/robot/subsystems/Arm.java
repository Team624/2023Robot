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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Arm extends ProfiledPIDSubsystem {
  /** Creates a new Arm. */
  private CANSparkMax armMotorRight;

  private CANSparkMax armMotorLeft;

  private DutyCycleEncoder boreEncoder;

  private ArmFeedforward armFeedForward;

  private ShuffleboardTab armTab;

  private double voltage = 0;
  public boolean recentFunnel = false;

  private GenericEntry enabledEntry;
  private GenericEntry voltageEntry;
  private GenericEntry coneEntry;
  private GenericEntry positionEntry;
  private GenericEntry setpointEntry;
  private GenericEntry goalEntry;

  public boolean cone = false;

  public Arm() {

    super(
        new ProfiledPIDController(
            Constants.Arm.kP,
            Constants.Arm.kI,
            Constants.Arm.kD,
            new TrapezoidProfile.Constraints(
                Constants.Arm.kMaxVelocityRadiansPerSecond,
                Constants.Arm.kMaxAccelerationRadiansPerSecondSquared)));

    getController().setTolerance(Units.degreesToRadians(3));

    armMotorRight = new CANSparkMax(Constants.Arm.armMotorRight, MotorType.kBrushless);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armMotorRight.setCANTimeout(500);
    armMotorRight.setSmartCurrentLimit(9);

    armMotorLeft = new CANSparkMax(Constants.Arm.armMotorLeft, MotorType.kBrushless);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorLeft.setCANTimeout(500);
    armMotorLeft.setSmartCurrentLimit(9);

    armMotorLeft.setInverted(true);

    boreEncoder = new DutyCycleEncoder(Constants.Arm.BORE_ENCODER_PORT);

    armFeedForward =
        new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV, Constants.Arm.kA);

    armTab = Shuffleboard.getTab("Arm");

    enabledEntry =
        armTab.add("Enabled", m_enabled).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    // setpointEntry = armTab.add("Setpoint", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

    positionEntry = armTab.add("Position", getAbsoluteRotation().getDegrees()).getEntry();

    voltageEntry = armTab.add("Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).getEntry();
    coneEntry = armTab.add("Cone", false).withPosition(9, 0).getEntry();
    setpointEntry = armTab.add("Setpoint", getController().getSetpoint().position).getEntry();
    goalEntry = armTab.add("Goal", getController().getGoal().position).getEntry();
  }

  @Override
  public void periodic() {
    super.periodic();

    enabledEntry.setBoolean(m_enabled);
    positionEntry.setDouble(getBore());
    setpointEntry.setDouble(getController().getGoal().position);



    // if (getController().getSet() > Constants.Arm.ESTOP_TOLERANCE.getRadians()) {
    //   armMotorLeft.stopMotor();
    //   armMotorRight.stopMotor();
    //   new DisabledArm(this).schedule();
    //   return;
    // }

    // setpointEntry.setDouble(getController().getGoal().position * (180 / Math.PI));

    coneEntry.setBoolean(cone);
  }

  public double getBore() {
    return MathUtil.inputModulus(
        (1 - boreEncoder.getAbsolutePosition()) + Constants.Arm.BORE_ENCODER_OFFSET, 0.0, 1.0);
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBore();
    if (radians > 1.5 * Math.PI) {
      double excess = radians - 1.5 * Math.PI;
      radians = -(0.5 * Math.PI - excess);
    }

    // Handle encoder looping around
    if (radians > 1.5 * Math.PI) {
      double excess = radians - 1.5 * Math.PI;

      radians = -(0.5 * Math.PI - excess);
    }

    return new Rotation2d(radians);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    if (this.m_enabled) {
      // voltage = armFeedForward.calculate(1.5 * Math.PI - setpoint.position, setpoint.velocity);

      voltage =
          output + armFeedForward.calculate(0.5 * Math.PI - setpoint.position, setpoint.velocity);

      voltage = MathUtil.clamp(voltage, -8.5, 8.5);

      if (voltage < 0 && getAbsoluteRotation().getRadians() < 0) {
        voltage = 0;
      }

      voltageEntry.setDouble(voltage);

      armMotorLeft.setVoltage(voltage);
      armMotorRight.setVoltage(voltage);
    } else {
      armMotorLeft.stopMotor();
      armMotorRight.stopMotor();

      voltage = 0;
    }
  }

  @Override
  protected double getMeasurement() {
    return getAbsoluteRotation().getRadians();
  }

  public void setGoal(Rotation2d rotation) {
    this.setGoal(rotation.getRadians());
  }

  public void stopArm() {
    armMotorRight.stopMotor();
    armMotorLeft.stopMotor();
  }

  public void setArmCommand(double setpoint) {
    Rotation2d angle = new Rotation2d(setpoint);

    armMotorLeft.setVoltage(setpoint);
    armMotorRight.setVoltage(setpoint);
  }

  public void setSpeed(double speed) {
    disable();
    armMotorLeft.set(speed);
    armMotorRight.set(speed);
  }

  public void pieceChange() {
    cone = !cone;
  }
}
