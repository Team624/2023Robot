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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wristMotor;

  private RelativeEncoder wristEncoder;

  private DutyCycleEncoder WristboreEncoder;

  private ProfiledPIDController wristController;

  private boolean enabledFeedback = true;

  private ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist0");
  private GenericEntry setpointEntry = wristTab.add("Setpoint", 0).getEntry();
  private GenericEntry positionEntry = wristTab.add("Position", 0).getEntry();
  private GenericEntry enabledEntry =
      wristTab.add("Enabled Feedback", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

  public Wrist() {

    wristMotor = new CANSparkMax(Constants.Wrist.WristMotor, MotorType.kBrushless);

    wristEncoder = wristMotor.getEncoder();

    wristMotor.setIdleMode(IdleMode.kBrake);

    wristMotor.setCANTimeout(500);

    wristMotor.setSmartCurrentLimit(5);

    WristboreEncoder = new DutyCycleEncoder(1);

    wristController =
        new ProfiledPIDController(
            Constants.Wrist.P,
            Constants.Wrist.I,
            Constants.Wrist.D,
            Constants.Wrist.wristCONSTRAINTS);

    wristController.setGoal(getAbsoluteRotation().getRadians());
    wristController.setTolerance(Units.degreesToRadians(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/Wrist/Encoder", getWristEncoder());
    SmartDashboard.putNumber("/Arm/BoreEncoder/get", WristboreEncoder.get());
    SmartDashboard.putNumber("/Wrist/BoreEncoder/Absolute", getBoreEncoder());
    SmartDashboard.putNumber("/Wrist/BoreEncoder/radians", getAbsoluteRotation().getRadians());

    setpointEntry.setDouble(wristController.getGoal().position);
    enabledEntry.setBoolean(enabledFeedback);
    positionEntry.setDouble(getAbsoluteRotation().getRadians());

    if (enabledFeedback) {
      double voltage = wristController.calculate(getAbsoluteRotation().getRadians());

      if (!softLimit(-voltage)) {
        wristMotor.setVoltage(-voltage);
      }
    }
  }

  public void resetController() {
    wristController.reset(getAbsoluteRotation().getRadians());
  }

  public ProfiledPIDController getContoller() {
    return wristController;
  }

  public void setReference(double rotation) {

    // 3.85 radians = mid
    // 4.24 raidans = high

    // 5.54 raidans ground intake

    // Prevent setpoints at unreachable points
    if (rotation > 4.9) {
      rotation = 0.15;
    }

    enabledFeedback = true;

    wristController.setGoal(rotation);
  }

  public Rotation2d getAbsoluteRotation() {
    double radians = 2 * Math.PI * getBoreEncoder();

    return new Rotation2d(radians);
  }

  public boolean softLimit(double value) {
    if (value > 0
        && (getAbsoluteRotation().getRadians() <= 0.15
            || getAbsoluteRotation().getRadians() > 4.9)) {
      System.out.println("SOFT LIMIT!");
      wristMotor.stopMotor();
      return true;
    }

    // if (value > 0 && getAbsoluteRotation().getRadians() >= 10.0) {
    //   return true;
    // }

    return false;
  }

  public boolean slowZone(double value) {
    if (value > 0 && getAbsoluteRotation().getRadians() <= 0.25) {
      System.out.println("SLOW ZONE!");
      return true;
    }

    return false;
  }

  public double getBoreEncoder() {
    return (MathUtil.inputModulus(
        WristboreEncoder.getAbsolutePosition() - Constants.Wrist.boreEncoderOffset, 0, 1));
  }

  public void zeroBoreEncoder() {
    WristboreEncoder.reset();
  }

  public void moveWrist(double speed) {
    this.enabledFeedback = false;

    if (!softLimit(speed)) {
      if (slowZone(speed)) {
        speed = MathUtil.clamp(speed, -0.5, 0.5);
      }
      wristMotor.set(speed);
    }
  }

  public void zeroWrist() {
    wristEncoder.setPosition(0.0);
  }

  public void stopWrist() {
    // rotationReference = getAbsoluteRotation();
    wristMotor.stopMotor();
  }

  public double getWristEncoder() {
    return wristEncoder.getPosition();
  }
}
