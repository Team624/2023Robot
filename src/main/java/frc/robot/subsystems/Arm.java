// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor = new CANSparkMax(Constants.Arm.ARM_MOTOR, MotorType.kBrushless);
  private RelativeEncoder armEncoder;

  private SparkMaxPIDController armPID;
  private double armP;
  private double armI;
  private double armD;
  private double armIz;
  private double armFF;
  private double armPnew;
  private double armInew;
  private double armDnew;
  private double armIznew;
  private double armFFnew;

  /** Creates a new Arm. */
  public Arm() {

    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder = armMotor.getEncoder();
    armPID = armMotor.getPIDController();

    armP = frc.robot.Constants.Arm.P;
    armI = frc.robot.Constants.Arm.I;
    armD = frc.robot.Constants.Arm.D;

    armPID.setP(armP);
    armPID.setI(armI);
    armPID.setD(armD);

    armPID.setOutputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend(double speed) {
    armMotor.set(speed);
  }

  public void extendTo(double setPoint) {
    armPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void resetArmEncoder() {
    armEncoder.setPosition(0);
  }

  public double getEncoder() {
    return armEncoder.getPosition();
  }
}
