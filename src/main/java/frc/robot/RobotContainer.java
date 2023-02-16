// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ControlArm;
import frc.robot.commands.Arm.IdleArm;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.Drivetrain.BlankDrive;
import frc.robot.commands.Drivetrain.ConeAlign;
import frc.robot.commands.Drivetrain.DisabledSwerve;
import frc.robot.commands.Drivetrain.GoalPose;
import frc.robot.commands.Drivetrain.SwerveDrive;
import frc.robot.commands.Drivetrain.UpdatePose;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Telescope.ControlTelescope;
import frc.robot.commands.Telescope.IdleTelescope;
import frc.robot.commands.Wrist.ControlWrist;
import frc.robot.commands.Wrist.IdleWrist;
import frc.robot.commands.auton.AutonManager;
import frc.robot.commands.auton.AutonSelection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final XboxController d_controller = new XboxController(0);
  public final XboxController m_controller = new XboxController(1);

  CommandXboxController m_controllerCommand = new CommandXboxController(1);

  /* Operator Controls */

  private final int armAxis = XboxController.Axis.kLeftY.value;
  private final int telescopeAxis = XboxController.Axis.kRightX.value;
  private final int wristAxis = XboxController.Axis.kRightY.value;

  // private final JoystickButton setTelescope =
  //     new JoystickButton(m_controller, XboxController.Button.kY.value);

  private final POVButton setArmTop = new POVButton(m_controller, 0);

  // private final JoystickButton setArmTop =
  //     new JoystickButton(m_controller, XboxController.Button.kA.value);

  private final POVButton setArmMid = new POVButton(m_controller, 90);

  private final JoystickButton runIntake =
      new JoystickButton(m_controller, XboxController.Button.kB.value);

  private final POVButton setArmBot = new POVButton(m_controller, 180);

  // private final JoystickButton resetArmEncoder =
  //     new JoystickButton(m_controller, XboxController.Button.kX.value);

  private final JoystickButton reverseIntake =
      new JoystickButton(m_controller, XboxController.Button.kX.value);

  private final POVButton setArmZero = new POVButton(m_controller, 270);

  private final JoystickButton resetTelescopeEncoder =
      new JoystickButton(m_controller, XboxController.Button.kA.value);

  private final JoystickButton resetWristEncoder =
      new JoystickButton(m_controller, XboxController.Button.kY.value);

  private final Trigger armMove = m_controllerCommand.axisLessThan(armAxis, -0.05);
  private final Trigger armMove2 = m_controllerCommand.axisGreaterThan(armAxis, 0.05);

  private final Trigger telescopeMove = m_controllerCommand.axisLessThan(telescopeAxis, -0.05);
  private final Trigger telescopeMove2 = m_controllerCommand.axisGreaterThan(telescopeAxis, 0.05);

  private final Trigger wristMove = m_controllerCommand.axisLessThan(wristAxis, -0.05);
  private final Trigger wristMove2 = m_controllerCommand.axisGreaterThan(wristAxis, 0.05);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro =
      new JoystickButton(d_controller, XboxController.Button.kA.value);

  private final JoystickButton alignTag =
      new JoystickButton(d_controller, XboxController.Button.kX.value);

  private final JoystickButton alignTag2 =
      new JoystickButton(d_controller, XboxController.Button.kY.value);

  private final JoystickButton alignTag3 =
      new JoystickButton(d_controller, XboxController.Button.kB.value);

  private final POVButton left = new POVButton(d_controller, 270);

  private final POVButton right = new POVButton(d_controller, 90);

  private final JoystickButton resetpose =
      new JoystickButton(d_controller, XboxController.Button.kLeftBumper.value);

  private final JoystickButton creepMode =
      new JoystickButton(d_controller, XboxController.Button.kRightBumper.value);

  // private final JoystickButton balance =
  //     new JoystickButton(d_controller, XboxController.Button.kX.value);

  /* Subsystems */
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Limelight m_limelight = new Limelight();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Wrist m_wrist = new Wrist();
  private final Telescope m_telescope = new Telescope();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () -> -modifyAxis(d_controller.getRawAxis(translationAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(strafeAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(rotationAxis))));

    m_arm.setDefaultCommand(new IdleArm(m_arm));
    m_telescope.setDefaultCommand(new IdleTelescope(m_telescope));
    // m_intake.setDefaultCommand(new IdleIntake(m_intake));
    m_wrist.setDefaultCommand(new IdleWrist(m_wrist));
    m_telescope.setDefaultCommand(new IdleTelescope(m_telescope));

    // m_limelight.setDefaultCommand(new UpdatePose(m_limelight, m_drivetrain));

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

    // balance.onTrue(new Balance(m_drivetrain));

    alignTag.whileTrue(new GoalPose(m_drivetrain, m_limelight, 0, 3));

    alignTag2.whileTrue(new GoalPose(m_drivetrain, m_limelight, 1, 3));

    alignTag3.whileTrue(new GoalPose(m_drivetrain, m_limelight, 2, 3));

    left.whileTrue(new ConeAlign(m_drivetrain, m_limelight, false));

    right.whileTrue(new ConeAlign(m_drivetrain, m_limelight, true));

    resetpose.whileTrue(new UpdatePose(m_limelight, m_drivetrain));

    creepMode.whileTrue(new InstantCommand(() -> m_drivetrain.yesCreepMode()));
    creepMode.whileFalse(new InstantCommand(() -> m_drivetrain.noCreepMode()));

    /** Intake */
    // runIntake.whileTrue(new RunIntake(m_intake));
    // reverseIntake.whileTrue(new ReverseIntake(m_intake));

    /** Arm */
    armMove.whileTrue(new ControlArm(m_arm, m_controller));

    armMove2.whileTrue(new ControlArm(m_arm, m_controller));

    setArmTop.onTrue(new SetArm(m_arm, 69));

    setArmMid.onTrue(new SetArm(m_arm, 62));

    setArmBot.onTrue(new SetArm(m_arm, 40));

    // resetArmEncoder.onTrue(new InstantCommand(() -> m_arm.resetEncoder()));

    setArmZero.onTrue(new SetArm(m_arm, 0.0));

    /** Telescope */
    telescopeMove.whileTrue(new ControlTelescope(m_telescope, m_controller));
    telescopeMove2.whileTrue(new ControlTelescope(m_telescope, m_controller));
    resetTelescopeEncoder.onTrue(new InstantCommand(() -> m_telescope.resetEncoder()));

    /** Wrist */
    wristMove.whileTrue(new ControlWrist(m_wrist, m_controller));
    wristMove2.whileTrue(new ControlWrist(m_wrist, m_controller));
    resetWristEncoder.onTrue(new InstantCommand(() -> m_wrist.zeroWrist()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

  public Command getAutonManager() {
    return new AutonManager(m_drivetrain);
  }

  public Command getAutonSelectionCommand() {
    return new AutonSelection();
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
            () -> -modifyAxis(d_controller.getRawAxis(translationAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(strafeAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(rotationAxis)));

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

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
