// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  public enum Animation {
    RED_CHASE(0),
    BLUE_CHASE(1),
    PURPLE_CHASE(2),
    PURPLE_SOLID(3),
    YELLOW_CHASE(4),
    YELLOW_SOLID(5),
    CRYPTONITE(6);

    public final int index;

    private Animation(int index) {
      this.index = index;
    }
  }

  private I2C leds;

  /** Creates a new LEDs. */
  public LEDs() {
    // NavX uses id 0x32
    leds = new I2C(Constants.LEDs.LEDS_PORT, Constants.LEDs.LEDS_ID);

    setAnimationCommand(Animation.CRYPTONITE).schedule();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAnimation(Animation animation) {
    leds.write(0x62, animation.index);
  }

  public CommandBase setAnimationCommand(Animation animation) {
    return this.runOnce(
        () -> {
          setAnimation(animation);
        });
  }
}
