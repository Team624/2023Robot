package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
  private final Drivetrain m_drivetrain;

  private double prevPitch;
  private double prevRoll;

  private double prevTimestamp;

  private double angleDegrees;

  private boolean offGround;

  private boolean reversed;

  public Balance(Drivetrain drivetrain, boolean reversed) {
    this.m_drivetrain = drivetrain;

    this.reversed = reversed;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    prevPitch = m_drivetrain.getPitch();
    prevRoll = m_drivetrain.getRoll();
    prevTimestamp = Timer.getFPGATimestamp();

    offGround = false;
  }

  @Override
  public void execute() {
    Rotation2d currentYaw = m_drivetrain.getPose().getRotation();
    Rotation2d currentPitch = Rotation2d.fromDegrees(m_drivetrain.getPitch());
    Rotation2d currentRoll = Rotation2d.fromDegrees(m_drivetrain.getRoll());

    // Get deltas for velocity calculations
    double pitchDelta = currentPitch.getDegrees() - prevPitch;
    double rollDelta = currentRoll.getDegrees() - prevRoll;
    double timeDelta = Timer.getFPGATimestamp() - prevTimestamp;

    double pitchDegreesPerSec = pitchDelta / timeDelta;
    double rollDegreesPerSec = rollDelta / timeDelta;

    // Calculate angle and velocity of the charger
    angleDegrees = currentYaw.getCos() * m_drivetrain.getPitch()
            + currentYaw.getSin() * m_drivetrain.getRoll();

    double angleVelocityDegreesPerSec =
    currentYaw.getCos() * pitchDegreesPerSec
            + currentYaw.getSin() * rollDegreesPerSec;

    System.out.println(angleVelocityDegreesPerSec);

    prevPitch = currentPitch.getDegrees();
    prevRoll = currentRoll.getDegrees();
    prevTimestamp = Timer.getFPGATimestamp();

    if (Math.abs(angleDegrees) >= Constants.Autonomous.AUTO_BALANCE_GROUND_ANGLE_THRESHOLD) {
      offGround = true;
      // System.out.println("off ground" + angleDegrees);
    }

    if (!offGround) {
      m_drivetrain.drive(new ChassisSpeeds(Constants.Autonomous.AUTO_BALANCE_GROUND_SPEED * (reversed ? -1 : 1), 0, 0), true, false);
      return;
    }

    // Check thresholds
    boolean shouldStop =
        (angleDegrees < 0.0 && angleVelocityDegreesPerSec > Constants.Autonomous.AUTO_BALANCE_VELOCITY_THRESHOLD)
            || (angleDegrees > 0.0
                && angleVelocityDegreesPerSec < -Constants.Autonomous.AUTO_BALANCE_VELOCITY_THRESHOLD);

    if (shouldStop) {
      m_drivetrain.stop();
    } else {
      m_drivetrain.drive(new ChassisSpeeds(Constants.Autonomous.AUTO_BALANCE_SPEED * (angleDegrees > 0.0 ? -1 : 1), 0.0, 0.0), true, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopWithX();
    setNTState(true);
  }

  @Override
  public boolean isFinished() {
    return offGround && Math.abs(angleDegrees) < Constants.Autonomous.AUTO_BALANCE_POSITION_THRESHOLD;
  }

  private void setNTState(boolean state) {
    SmartDashboard.getEntry("/auto/balance/state").setBoolean(state);
  }
}
