// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */
  private CANSparkMax telescopeMotor;

  private ElevatorFeedforward telescopFeedforward;

  private RelativeEncoder telescopeEncoder;

  private SparkMaxPIDController telescopePID;

  private double P;
  private double I;
  private double D;

  public Telescope() {
    telescopeMotor = new CANSparkMax(Constants.Telescope.telescopemotor, MotorType.kBrushless);
    telescopeMotor.restoreFactoryDefaults();
    telescopeEncoder = telescopeMotor.getEncoder();
    telescopePID = telescopeMotor.getPIDController();
    telescopeMotor.setIdleMode(IdleMode.kBrake);
    telescopeMotor.setCANTimeout(500);

    P = frc.robot.Constants.Telescope.P;
    I = frc.robot.Constants.Telescope.I;
    D = frc.robot.Constants.Telescope.D;

    telescopePID.setP(P);
    telescopePID.setI(I);
    telescopePID.setD(D);

    telescopePID.setOutputRange(-1, 1);

    telescopeMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    telescopeMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    telescopeMotor.setSoftLimit(SoftLimitDirection.kForward, 30);
    telescopeMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.01f);

    telescopFeedforward =
        new ElevatorFeedforward(
            Constants.Telescope.kS, Constants.Telescope.kG, Constants.Telescope.kV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("telescope encoder: " + telescopeEncoder.getPosition());
    // System.out.println("telescope velocity: " + telescopeEncoder.getVelocity());

    SmartDashboard.putNumber("/Telescope/Encoder", getTelescopeEncoder());
  }

  public void controlTelescope(double speed) {
    telescopeMotor.set(speed);
  }

  public void stopTelescope() {
    telescopeMotor.stopMotor();
  }

  public void setTelescope(double position) {
    telescopePID.setReference(position, ControlType.kPosition);
    // telescopePID.setReference(
    //     position, ControlType.kPosition, 0, telescopFeedforward.calculate(0.0));
  }

  public void resetEncoder() {
    telescopeEncoder.setPosition(0.0);
  }

  public double getTelescopeEncoder() {
    return telescopeEncoder.getPosition();
  }
}
