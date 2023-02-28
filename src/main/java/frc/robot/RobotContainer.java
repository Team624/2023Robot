// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ControlArm;
import frc.robot.commands.Arm.IdleArm;
import frc.robot.commands.Drivetrain.BlankDrive;
import frc.robot.commands.Arm.SetArm;
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
import frc.robot.commands.Wrist.SetWrist;
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
  private final GenericHID driverPOV = new GenericHID(0);
  public final XboxController m_controller = new XboxController(1);

  CommandXboxController m_controllerCommand = new CommandXboxController(1);

  /* Operator Controls */

  private final JoystickButton toggleLED =
      new JoystickButton(m_controller, XboxController.Button.kY.value);

  private final int armAxis = XboxController.Axis.kLeftY.value;
  private final int telescopeAxis = XboxController.Axis.kRightY.value;
  private final int wristAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton manual =
      new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);

  /* Intake */

  private final JoystickButton runIntake =
      new JoystickButton(m_controller, XboxController.Button.kX.value);

  private final JoystickButton reverseIntake =
      new JoystickButton(m_controller, XboxController.Button.kB.value);

  /* Arm */

  private final Trigger armMove = m_controllerCommand.axisLessThan(armAxis, -0.08);
  private final Trigger armMove2 = m_controllerCommand.axisGreaterThan(armAxis, 0.08);

  private final POVButton setBotTop = new POVButton(m_controller, 0);

  private final POVButton setBotMid = new POVButton(m_controller, 90);

  private final POVButton setBotLower = new POVButton(m_controller, 180);

  private final POVButton setBot0 = new POVButton(m_controller, 270);

  // private final POVButton setArmZero = new POVButton(m_controller, 270);

  /* Telescope */

  private final Trigger telescopeMove = m_controllerCommand.axisLessThan(telescopeAxis, -0.08);
  private final Trigger telescopeMove2 = m_controllerCommand.axisGreaterThan(telescopeAxis, 0.08);

  private final JoystickButton resetTelescopeEncoder =
      new JoystickButton(m_controller, XboxController.Button.kA.value);
  private final JoystickButton runIntake =
      new JoystickButton(m_controller, XboxController.Button.kX.value);

  private final JoystickButton reverseIntake =
      new JoystickButton(m_controller, XboxController.Button.kB.value);

  private final Trigger wristMove = m_controllerCommand.axisLessThan(wristAxis, -0.08);
  private final Trigger wristMove2 = m_controllerCommand.axisGreaterThan(wristAxis, 0.08);

  private final JoystickButton resetWristEncoder =
      new JoystickButton(m_controller, XboxController.Button.kY.value);

  private final Trigger armMove = m_controllerCommand.axisLessThan(armAxis, -0.08);
  private final Trigger armMove2 = m_controllerCommand.axisGreaterThan(armAxis, 0.08);

  private final Trigger telescopeMove = m_controllerCommand.axisLessThan(telescopeAxis, -0.08);
  private final Trigger telescopeMove2 = m_controllerCommand.axisGreaterThan(telescopeAxis, 0.08);

  private final Trigger wristMove = m_controllerCommand.axisLessThan(wristAxis, -0.08);
  private final Trigger wristMove2 = m_controllerCommand.axisGreaterThan(wristAxis, 0.08);

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

  private final JoystickButton substationButton =
      new JoystickButton(d_controller, XboxController.Button.kLeftBumper.value);

  private final JoystickButton creepMode =
      new JoystickButton(d_controller, XboxController.Button.kRightBumper.value);

  private final JoystickButton resetpose =
      new JoystickButton(d_controller, XboxController.Button.kLeftBumper.value);

  private final JoystickButton balance =
      new JoystickButton(d_controller, XboxController.Button.kX.value);

  /* Subsystems */
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Limelight m_limelight = new Limelight();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Wrist m_wrist = new Wrist();
  private final Telescope m_telescope = new Telescope();
  // private final ledControl m_LedControl =
  //     new ledControl(
  //         new TrobotAddressableLED(Constants.LED.LEDPort, Constants.LED.LENGTH),
  //         DriverStation.getAlliance() == Alliance.Red);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () -> -modifyAxis(d_controller.getRawAxis(translationAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(strafeAxis)),
            () -> -modifyAxis((d_controller.getRawAxis(rotationAxis)))));

    m_arm.setDefaultCommand(new IdleArm(m_arm));

    m_intake.setDefaultCommand(new IdleIntake(m_intake));
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
    // uncomment this
    // alignTag.onTrue(new SubstationAlign(m_drivetrain,
    // DriverStation.getAlliance()==Alliance.Red));
    alignTag2.whileTrue(new GoalPose(m_drivetrain, m_limelight, 1, 3));

    alignTag3.whileTrue(new GoalPose(m_drivetrain, m_limelight, 2, 3));

    left.whileTrue(new ConeAlign(m_drivetrain, m_limelight, false));

    right.whileTrue(new ConeAlign(m_drivetrain, m_limelight, true));

    substationButton.whileTrue(
        new SubstationAlign(m_drivetrain, DriverStation.getAlliance() == Alliance.Red));

    manual.and(armMove).whileTrue(new ControlArm(m_arm, m_controller));

    /** Arm */
    manual.and(armMove).whileTrue(new ControlArm(m_arm, m_controller));
    manual.and(armMove2).whileTrue(new ControlArm(m_arm, m_controller));
    manual.and(armMove2).whileTrue(new ControlArm(m_arm, m_controller));

    setArmTop.whileTrue(new SetArm(m_arm, Constants.Arm.SETPOINT_HIGH));

    setArmMid.whileTrue(new SetArm(m_arm, Constants.Arm.SETPOINT_MID));

    setArmBot.whileTrue(new SetArm(m_arm, Constants.Arm.SETPOINT_INTAKE));

    // resetArmEncoder.onTrue(new InstantCommand(() -> m_arm.resetEncoder()));

    setArmZero.whileTrue(new SetArm(m_arm, Constants.Arm.SETPOINT_RETRACT));

    /** Telescope */
    manual.and(telescopeMove).whileTrue(new ControlTelescope(m_telescope, m_controller));
    manual.and(telescopeMove2).whileTrue(new ControlTelescope(m_telescope, m_controller));

    resetTelescopeEncoder.onTrue(new InstantCommand(() -> m_telescope.resetEncoder()));

    /** Wrist */
    manual.and(wristMove).whileTrue(new ControlWrist(m_wrist, m_controller));
    manual.and(wristMove2).whileTrue(new ControlWrist(m_wrist, m_controller));

    resetWristEncoder.onTrue(new InstantCommand(() -> m_wrist.zeroWrist()));

    // Setpoints

    // Funnel = 0
    // IntakeCONE = 1
    // IntakeCUBE = 2
    // mid = 3
    // top = 4

    setBotLower.onTrue(new SetWrist(m_wrist, Constants.Wrist.WRIST_SETPOINT_CONE_INTAKE));

    // setBotTop.onTrue(new TelescopeWrist(m_telescope, m_wrist, 4));
    // setBotMid.onTrue(new TelescopeWrist(m_telescope, m_wrist, 3));

    // setBotTop.onTrue(new SetTelescope(m_telescope, Constants.Telescope.TELESCOPE_SETPOINT_HIGH));
    // setBotMid.onTrue(new SetTelescope(m_telescope, Constants.Telescope.TELESCOPE_SETPOINT_MID));
    setBot0.onTrue(new SetTelescope(m_telescope, Constants.Telescope.TELESCOPE_SETPOINT_FUNNEL));
    // setBotLower.onTrue(new SetArm(m_arm, Constants.Arm.SETPOINT_MID));

    // toggleLED.onTrue(new InstantCommand(() -> m_LedControl.switchLED()));

    runIntake.whileTrue(new RunIntake(m_intake));
    reverseIntake.whileTrue(new ReverseIntake(m_intake));
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

  public void setDisabledDrivetrainDefault() {
    m_drivetrain.setDefaultCommand(new DisabledSwerve(m_drivetrain));
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
