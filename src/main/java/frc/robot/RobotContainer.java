// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drivetrain.AutonomousDrive;
import frc.robot.commands.Drivetrain.BlankDrive;
import frc.robot.commands.Drivetrain.DisabledSwerve;
import frc.robot.commands.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final XboxController d_controller = new XboxController(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(d_controller, XboxController.Button.kA.value);

  /* Subsystems */
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final JoystickButton robotCentric =
      new JoystickButton(d_controller, XboxController.Button.kLeftBumper.value);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () ->
                -modifyAxis(
                    d_controller.getRawAxis(translationAxis)
                        * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND),
            () ->
                -modifyAxis(
                    d_controller.getRawAxis(strafeAxis)
                        * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND),
            () ->
                -modifyAxis(
                    d_controller.getRawAxis(rotationAxis)
                        * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
            () -> robotCentric.getAsBoolean()));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    zeroGyro.onTrue(new InstantCommand(() -> m_drivetrain.zeroGyroscope()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

  public Command getAutonomousDriveCommand(Auton auton) {
    return new AutonomousDrive(m_drivetrain, auton);
  }

  public void ghostSwerve() {
    new DisabledSwerve(m_drivetrain);
  }

  public void setBlankDrivetrainCommand() {
    m_drivetrain.setDefaultCommand(new BlankDrive(m_drivetrain));
  }

  public void setDrivetrainDefaultCommand() {
    Command c =
        new SwerveDrive(
            m_drivetrain,
            () ->
                -modifyAxis(
                    d_controller.getRawAxis(translationAxis)
                        * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND),
            () ->
                -modifyAxis(
                    d_controller.getRawAxis(strafeAxis)
                        * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND),
            () ->
                -modifyAxis(
                    d_controller.getRawAxis(rotationAxis)
                        * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
            () -> robotCentric.getAsBoolean());

    m_drivetrain.setDefaultCommand(c);
    c.schedule();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.Swerve.DRIVETRAIN_INPUT_DEADBAND);

    return value;
  }
}
