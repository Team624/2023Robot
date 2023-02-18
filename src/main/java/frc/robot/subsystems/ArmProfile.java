// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class ArmProfile extends TrapezoidProfileSubsystem {
  /** Creates a new ArmProfile. */
  private CANSparkMax armMotorRight;

  private CANSparkMax armMotorLeft;

  private RelativeEncoder armEncoderLeft;
  private RelativeEncoder armEncoderRight;

  private SparkMaxPIDController armSparkmaxPIDLeft;
  private SparkMaxPIDController armSparkmaxPIDRight;

  ArmFeedforward feedforward =
      new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, Constants.Arm.kV,Constants.Arm.kA);

  private double RkP;
  private double RkI;
  private double RkD;

  private double LkP;
  private double LkI;
  private double LkD;

  private MotorControllerGroup Armgroup;

  private DutyCycleEncoder boreEncoder;
  public ArmProfile() {
    
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(1, 3),
        // The initial position of the mechanism
        0.0);

       

    armMotorRight = new CANSparkMax(frc.robot.Constants.Arm.armMotorRight, MotorType.kBrushless);
    armMotorRight.restoreFactoryDefaults();
    armEncoderRight = armMotorRight.getEncoder();
    armMotorRight.setIdleMode(IdleMode.kBrake);
    armSparkmaxPIDRight = armMotorRight.getPIDController();

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

    LkP = frc.robot.Constants.Arm.LkP;
    LkI = frc.robot.Constants.Arm.LkI;
    LkD = frc.robot.Constants.Arm.LkD;
    

    armSparkmaxPIDLeft.setP(LkP);
    armSparkmaxPIDLeft.setI(LkI);
    armSparkmaxPIDLeft.setD(LkD);

    armSparkmaxPIDLeft.setOutputRange(-1, 1);

    Armgroup = new MotorControllerGroup(armMotorRight, armMotorLeft);

    boreEncoder = new DutyCycleEncoder(0);
  }

  @Override
  protected void useState(TrapezoidProfile.State setpoint) {
    // Use the computed profile state here.
    double ArmFeedforward = feedforward.calculate(setpoint.position, 0);
    armSparkmaxPIDLeft.setReference(setpoint.position, ControlType.kPosition,0,ArmFeedforward / 12.0);
    armSparkmaxPIDRight.setReference(setpoint.position, ControlType.kPosition,0,ArmFeedforward / 12.0);
  }

  public Command setArmCommand(double setpoint) {
    return Commands.runOnce(() -> setGoal(setpoint), this);
  }

  public void controlArmRight(double speed) {
    armMotorRight.set(speed);
  }
  public double getBoreEncoder(){
    return boreEncoder.get();
  }

  public void zeroBoreEncoder() {
    boreEncoder.reset();
  }

  public void controlArmLeft(double speed) {
    armMotorLeft.set(speed);
  }

  public void controlArm(double speed) {
    Armgroup.set(speed);
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

  public void setArmCommand(double velocity, Rotation2d angle) {
  
    armMotorLeft.setVoltage(velocity+feedforward.calculate(angle.getRadians(), 0));
    armMotorRight.setVoltage(velocity+feedforward.calculate(angle.getRadians(), 0));

  

    // armSparkmaxPIDRight.setReference(
    //   velocity, ControlType.kVelocity, 0, feedforward.calculate(angle.getRadians(), 0));

    // armSparkmaxPIDLeft.setReference(
    //   velocity, ControlType.kVelocity, 0, feedforward.calculate(angle.getRadians(), 0));
  }

  public void resetEncoder() {
    armEncoderRight.setPosition(0.0);
    armEncoderLeft.setPosition(0.0);
  }
}
