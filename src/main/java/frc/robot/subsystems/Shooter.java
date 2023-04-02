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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax shooterMotorLeft;

  private final CANSparkMax shooterMotorRight;

  public double addedPercentOutput = 0;

  private double goalRPM;

  private final RelativeEncoder shooterEncoderLeft;
  private final SparkMaxPIDController shooterPidControllerLeft;

  private final RelativeEncoder shooterEncoderRight;
  private final SparkMaxPIDController shooterPidControllerRight;

  private ShuffleboardTab shootTab = Shuffleboard.getTab("Shooter");
  private GenericEntry dashBoardGoalRPM =
      shootTab.add("Goal RPM:", 0).withPosition(0, 1).getEntry();
  private GenericEntry dashBoardCurrentRPMLeft =
      shootTab.add("Current RPM LEFT:", 0).withPosition(1, 1).getEntry();
  private GenericEntry dashBoardCurrentRPMRight =
      shootTab.add("Current RPM RIGHT:", 0).withPosition(1, 1).getEntry();
  private GenericEntry dashBoardinputRPM =
      shootTab
          .add("Input RPM: ", 0)
          .withPosition(0, 3)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();
  private GenericEntry addedPercent =
      shootTab
          .add("Added Percent: ", 0)
          .withPosition(0, 3)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry();

  public Shooter() {
    shooterMotorLeft = new CANSparkMax(Constants.Shooter.shooterMotorLeft, MotorType.kBrushless);
    shooterMotorRight = new CANSparkMax(Constants.Shooter.shooterMotorRight, MotorType.kBrushless);

    shooterEncoderLeft = shooterMotorLeft.getEncoder();
    shooterPidControllerLeft = shooterMotorLeft.getPIDController();
    shooterMotorLeft.setIdleMode(IdleMode.kCoast);
    shooterMotorLeft.setCANTimeout(500);

    shooterMotorLeft.setInverted(true);

    shooterEncoderRight = shooterMotorRight.getEncoder();
    shooterPidControllerRight = shooterMotorRight.getPIDController();
    shooterMotorRight.setIdleMode(IdleMode.kCoast);
    shooterMotorRight.setCANTimeout(500);
    shooterMotorRight.follow(shooterMotorLeft, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateDashBoard();
  }

  private void updateDashBoard() {
    dashBoardGoalRPM.setDouble(goalRPM);
    dashBoardCurrentRPMLeft.setDouble(shooterEncoderLeft.getVelocity());
    dashBoardCurrentRPMRight.setDouble(shooterEncoderRight.getVelocity());
    addedPercent.setDouble(addedPercentOutput);
  }

  public void setDashBoardRPM() {
    goalRPM = dashBoardinputRPM.getDouble(0);
    shooterPidControllerLeft.setReference(goalRPM, ControlType.kVelocity);
    shooterPidControllerRight.setReference(goalRPM, ControlType.kVelocity);
    dashBoardGoalRPM.setDouble(goalRPM);
  }

  public void setRPM(double RPM) {
    goalRPM = RPM;
    shooterPidControllerLeft.setReference(goalRPM, ControlType.kVelocity);
    shooterPidControllerRight.setReference(goalRPM, ControlType.kVelocity);
    dashBoardGoalRPM.setDouble(goalRPM);
  }

  public void setPercentOutput(double speed) {
    shooterMotorLeft.set(speed);
  }

  public void setShooterVoltage(double voltage){
    shooterMotorLeft.setVoltage(voltage);
    // shooterMotorRight.setVoltage(voltage);
  }

  public void stopShooter() {
    shooterMotorLeft.stopMotor();
  }

  public void addPercentOutput() {
    addedPercentOutput += 100;
  }

  public void lostPercentOutput() {
    addedPercentOutput += 100;
  }
}
