// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private Solenoid grabSolenoid;
  private boolean isGrabbin;

  public Grabber() {
    this.grabSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean getGrabbin(){
    return isGrabbin;
  }
  public void close(){
    grabSolenoid.set(true);
    isGrabbin=true;
  }
  public void open(){
    grabSolenoid.set(false);
    isGrabbin=false;
  }
}
