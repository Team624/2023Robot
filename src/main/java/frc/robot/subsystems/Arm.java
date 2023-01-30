// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor = new CANSparkMax(0, null);
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

  private CANSparkMax pivotMotor = new CANSparkMax(0, null);
  private RelativeEncoder pivotEncoder;

  private SparkMaxPIDController pivotPID;
  private double pivotP;
  private double pivotI;
  private double pivotD;
  private double pivotIz;
  private double pivotFF;
  private double pivotPnew;
  private double pivotInew;
  private double pivotDnew;
  private double pivotIznew;
  private double pivotFFnew;
  

  /** Creates a new Arm. */
  public Arm() {
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotEncoder=pivotMotor.getEncoder();
    
    armMotor.setIdleMode(IdleMode.kBrake);
    armEncoder=pivotMotor.getEncoder();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void extend(double speed){armMotor.set(speed);}
  public void extendTo(double setPoint){armPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);}
  public void movePivot(double speed){pivotMotor.set(speed);}
  public void pivotTo(double setPoint){pivotPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);}

  public void stop(){
    armMotor.stopMotor();
    pivotMotor.stopMotor();
  }
  public void resetArmEncoder(){
    armEncoder.setPosition(0);
  }
  public void resetPivotEncoder(){
    pivotEncoder.setPosition(0);
  }
  
  public void resetEncoders(){
    resetArmEncoder();
    resetPivotEncoder();
  }

}
