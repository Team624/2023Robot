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
import edu.wpi.first.math.geometry.Rotation2d;
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

  ArmFeedforward wristfeedforward =
      new ArmFeedforward(Constants.Wrist.kS, Constants.Wrist.kG, Constants.Wrist.kV);

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

    wristPidController.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/Wrist/Encoder", getWristEncoder());
  }

  public void moveWrist(double speed) {
    wristMotor.set(speed);
  }

  public void zeroWrist() {
    wristEncoder.setPosition(0.0);
  }

  public void stopWrist() {
    wristMotor.stopMotor();
  }

  public void setWristCommand(double setpoint, Rotation2d angle) {

    wristPidController.setReference(
        angle.getRadians(),
        ControlType.kPosition,
        0,
        wristfeedforward.calculate(angle.getRadians(), 0));
  }

  public double getWristEncoder() {
    return wristEncoder.getPosition();
  }
}
