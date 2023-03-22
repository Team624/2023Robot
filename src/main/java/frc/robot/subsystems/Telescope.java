// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Telescope.ResetEncoder;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  private CANSparkMax telescopeMotor;

  private RelativeEncoder telescopeEncoder;

  private SparkMaxAnalogSensor stringPot;

  private SparkMaxPIDController telescopePID;

  private double P;
  private double I;
  private double D;

  private GenericEntry positionEntry;

  private boolean softLimited;

  public Telescope() {
    telescopeMotor = new CANSparkMax(Constants.Telescope.telescopemotor, MotorType.kBrushless);
    telescopeEncoder = telescopeMotor.getEncoder();
    telescopePID = telescopeMotor.getPIDController();
    telescopeMotor.setIdleMode(IdleMode.kBrake);
    telescopeMotor.setCANTimeout(500);
    telescopeMotor.setInverted(true);

    P = frc.robot.Constants.Telescope.P;
    I = frc.robot.Constants.Telescope.I;
    D = frc.robot.Constants.Telescope.D;

    telescopePID.setP(P);
    telescopePID.setI(I);
    telescopePID.setD(D);

    telescopePID.setOutputRange(-1, 1);

    // telescopeMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // telescopeMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // telescopeMotor.setSoftLimit(SoftLimitDirection.kForward, 45);
    // telescopeMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.01f);

    stringPot = telescopeMotor.getAnalog(Mode.kAbsolute);

    telescopePID.setFeedbackDevice(stringPot);

    Shuffleboard.getTab("Telescope").add("Reset Encoder", new ResetEncoder(this));
    positionEntry =
        Shuffleboard.getTab("Telescope").add("Position (String Pot)", getStringPot()).getEntry();

    softLimited = getSoftLimit();
    if (softLimited) telescopeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("/Telescope/Encoder", getTelescopeEncoder());
    positionEntry.setDouble(getStringPot());
  }

  public void controlTelescope(double speed) {
    if (softLimited) return;
    telescopeMotor.set(speed);
  }

  public void stopTelescope() {
    telescopeMotor.stopMotor();
  }

  public void setTelescope(double position) {
    if (softLimited) return;
    telescopePID.setReference(position, ControlType.kPosition);
  }

  public void resetEncoder() {
    telescopeEncoder.setPosition(0.0);
  }

  public double getTelescopeEncoder() {
    return telescopeEncoder.getPosition();
  }

  public double getStringPot() {
    return stringPot.getPosition();
  }

  public boolean getSoftLimit() {
    return (stringPot.getPosition() <= 0.2 && telescopeMotor.getAppliedOutput() < 0)
        || (stringPot.getPosition() >= 1.12 && telescopeMotor.getAppliedOutput() > 0);
  }
}
