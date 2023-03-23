// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ControlArm;
import frc.robot.commands.Arm.IdleArm;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.DoubleSubstation;
import frc.robot.commands.Drivetrain.ConeAlign;
import frc.robot.commands.Drivetrain.DisabledSwerve;
import frc.robot.commands.Drivetrain.GoalPose;
import frc.robot.commands.Drivetrain.SubstationAlign;
import frc.robot.commands.Drivetrain.SwerveDrive;
import frc.robot.commands.Drivetrain.UpdatePose;
import frc.robot.commands.Hood.ControlHood;
import frc.robot.commands.Hood.IdleHood;
import frc.robot.commands.Hood.SetHood;
import frc.robot.commands.Hood.SetHoodUpright;
import frc.robot.commands.InsideBotSequences.InsideBot;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Intake.IdleSpinIntake;
import frc.robot.commands.Intake.ReverseCone;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shooter.IdleShooter;
import frc.robot.commands.Shooter.SetShooter;
import frc.robot.commands.Shooter.ShooterScore;
import frc.robot.commands.SideConeSequences.Intake.SideIntakeSequence;
import frc.robot.commands.SideConeSequences.Score.SideScoringSequence;
import frc.robot.commands.Telescope.ControlTelescope;
import frc.robot.commands.Telescope.IdleTelescope;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.Telescope.SetTelescopeScore;
import frc.robot.commands.UprightConeSequences.Intake.UprightIntakeSequence;
import frc.robot.commands.Wrist.ControlWrist;
import frc.robot.commands.Wrist.IdleWrist;
import frc.robot.commands.Wrist.SetWrist;
import frc.robot.commands.auton.AutonManager;
import frc.robot.commands.auton.AutonSelection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;
import java.util.Map;

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

  /* LEDs */

  private final JoystickButton toggleMode =
      new JoystickButton(m_controller, XboxController.Button.kY.value);

  public boolean coneMode = true;
  public boolean reverseShooter = false;

  private final int armAxis = XboxController.Axis.kLeftY.value;
  private final int telescopeAxis = XboxController.Axis.kRightY.value;
  private final int wristAxis = XboxController.Axis.kRightX.value;
  private final int coneModifyAxis = XboxController.Axis.kRightTrigger.value;

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

  private final Trigger coneModify = m_controllerCommand.axisGreaterThan(coneModifyAxis, 0.2);

  /* Telescope */

  private final Trigger telescopeMove = m_controllerCommand.axisLessThan(telescopeAxis, -0.08);
  private final Trigger telescopeMove2 = m_controllerCommand.axisGreaterThan(telescopeAxis, 0.08);

  /* Wrist */

  private final Trigger wristMove = m_controllerCommand.axisLessThan(wristAxis, -0.08);
  private final Trigger wristMove2 = m_controllerCommand.axisGreaterThan(wristAxis, 0.08);

  /* Full setpoints */

  private final POVButton setBotHigh = new POVButton(m_controller, 0);

  private final POVButton setBotMid = new POVButton(m_controller, 90);

  private final POVButton setBotIntake = new POVButton(m_controller, 180);

  private final POVButton setBotInside = new POVButton(m_controller, 270);

  private final JoystickButton substationSetpoint =
      new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);

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

  /* Subsystems */
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Limelight m_limelight = new Limelight();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Wrist m_wrist = new Wrist();
  private final Telescope m_telescope = new Telescope();
  private final LEDs m_leds = new LEDs();
  private final Hood m_hood = new Hood();
  private final Shooter m_shooter = new Shooter();

  private enum CommandSelector {
    ARM,
    HOOD
  }

  private CommandSelector select() {
    if (coneMode) {
      return CommandSelector.ARM;
    } else {
      return CommandSelector.HOOD;
    }
  }

  private Command m_LeftJoystickCommand =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new ControlArm(m_arm, m_controller)),
              Map.entry(CommandSelector.HOOD, new ControlHood(m_hood, m_controller))),
          this::select);

  private Command m_WristCommand =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new ControlWrist(m_wrist, m_controller)),
              Map.entry(CommandSelector.HOOD, new IdleWrist(m_wrist))),
          this::select);

  private Command m_TelescopeCommand =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new ControlTelescope(m_telescope, m_controller)),
              Map.entry(CommandSelector.HOOD, new IdleTelescope(m_telescope))),
          this::select);

  private Command m_OperatorUpDpad =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(
                  CommandSelector.ARM,
                  new SideScoringSequence(m_arm, m_telescope, m_wrist, 1, false)),
              Map.entry(
                  CommandSelector.HOOD, new SetHood(m_hood, Constants.Hood.Hood_High_Setpoint))),
          this::select);
  private Command m_OperatorUpDpadConeModify =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(
                  CommandSelector.ARM,
                  new SideScoringSequence(m_arm, m_telescope, m_wrist, 1, true)),
              Map.entry(
                  CommandSelector.HOOD,
                  (new SequentialCommandGroup(
                      new SetHood(m_hood, Constants.Hood.Hood_Hybrid_Setpoint),
                      new ShooterScore(m_shooter, Constants.Shooter.LowScoreSpeed))))),
          this::select);

  private Command m_OperatorMidDpad =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(
                  CommandSelector.ARM,
                  new SideScoringSequence(m_arm, m_telescope, m_wrist, 0, false)),
              Map.entry(
                  CommandSelector.HOOD, new SetHood(m_hood, Constants.Hood.Hood_Mid_Setpoint))),
          this::select);

  private Command m_OperatorMidDpadConeModify =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(
                  CommandSelector.ARM,
                  new SideScoringSequence(m_arm, m_telescope, m_wrist, 0, true)),
              Map.entry(CommandSelector.HOOD, new IdleHood(m_hood))),
          this::select);

  private Command m_OperatorIntakeDpad =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new SideIntakeSequence(m_arm, m_telescope, m_wrist)),
              Map.entry(
                  CommandSelector.HOOD, new SetHood(m_hood, Constants.Hood.Hood_Intake_Setpoint))),
          this::select);
  private Command m_OperatorXButton =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new RunIntake(m_intake)),
              Map.entry(
                  CommandSelector.HOOD, new SetShooter(m_shooter, Constants.Shooter.IntakeSpeed))),
          this::select);

  private Command m_OperatorInsideButton =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new InsideBot(m_arm, m_telescope, m_wrist)),
              Map.entry(
                  CommandSelector.HOOD, new SetHood(m_hood, Constants.Hood.Hood_Upright_Setpoint))),
          this::select);

  private Command m_OperatorXButtonFalse =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new IdleSpinIntake(m_intake)),
              Map.entry(CommandSelector.HOOD, new IdleHood(m_hood))),
          this::select);

  private Command m_OperatorBButton =
      new SelectCommand(
          Map.ofEntries(
              Map.entry(CommandSelector.ARM, new ReverseCone(m_intake)),
              Map.entry(CommandSelector.HOOD, new SetShooter(m_shooter, -0.3))),
          this::select);

          private Command m_OperatorBButtonFalse =
          new SelectCommand(
              Map.ofEntries(
                  Map.entry(CommandSelector.ARM, new IdleSpinIntake(m_intake)),
                  Map.entry(CommandSelector.HOOD, new IdleHood(m_hood))),
              this::select);


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
    m_limelight.setDefaultCommand(new UpdatePose(m_limelight, m_drivetrain));
    m_hood.setDefaultCommand(new IdleHood(m_hood));
    m_shooter.setDefaultCommand(new IdleShooter(m_shooter));
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

    creepMode.onTrue(new InstantCommand(m_drivetrain::yesCreepMode));
    creepMode.onFalse(new InstantCommand(m_drivetrain::noCreepMode));

    alignTag.whileTrue(new GoalPose(m_drivetrain, m_limelight, 0, 3));

    alignTag2.whileTrue(new GoalPose(m_drivetrain, m_limelight, 1, 3));

    alignTag3.whileTrue(new GoalPose(m_drivetrain, m_limelight, 2, 3));

    left.whileTrue(new ConeAlign(m_drivetrain, false, m_limelight));

    right.whileTrue(new ConeAlign(m_drivetrain, true, m_limelight));

    substationButton.whileTrue(
        new SubstationAlign(m_drivetrain, DriverStation.getAlliance() == Alliance.Red));

    /** Arm */
    manual.and(armMove).whileTrue(m_LeftJoystickCommand);
    manual.and(armMove2).whileTrue(m_LeftJoystickCommand);

    /** Telescope */
    manual.and(telescopeMove).whileTrue(m_TelescopeCommand);
    manual.and(telescopeMove2).whileTrue(m_TelescopeCommand);

    /** Wrist */
    manual.and(wristMove).whileTrue(m_WristCommand);
    manual.and(wristMove2).whileTrue(m_WristCommand);

    /** ARM TESTING */
    // setBotHigh.whileTrue(new SetArm(m_arm, Constants.Arm.ARM_SETPOINT_HIGH));
    // setBotMid.whileTrue(new SetArm(m_arm, Constants.Arm.ARM_SETPOINT_MID));
    // setBotIntake.whileTrue(new SetArm(m_arm, Constants.Arm.ARM_SETPOINT_SIDE_CONE_INTAKE));

    /** WRIST TESTING */
    // setBotHigh.whileTrue(new SetWrist(m_wrist, Constants.Wrist.wrist_upright_cone_intake));
    // setBotMid.whileTrue(new SetWrist(m_wrist, Constants.Wrist.wrist_cone_intake));
    // setBotIntake.whileTrue(new SetWrist(m_wrist,
    // Constants.Wrist.wrist_zero));

    /** TELESCOPE TESTING */

    // setBotHigh.whileTrue(new SetTelescope(m_telescope,
    // Constants.Telescope.TELESCOPE_SETPOINT_HIGH));
    // setBotMid.whileTrue(new SetTelescope(m_telescope,
    // Constants.Telescope.TELESCOPE_SETPOINT_MID));
    // setBotIntake.whileTrue(new SetTelescope(m_telescope,
    // Constants.Telescope.TELESCOPE_SETPOINT_SIDE_CONE_INTAKE));

    /** Hood TESTING */
    // setBotHigh.whileTrue(new SetHood(m_hood, Constants.Hood.Hood_High_Setpoint));
    // setBotMid.whileTrue(new SetHood(m_hood, Constants.Hood.Hood_Mid_Setpoint));
    // setBotIntake.whileTrue(new SetHood(m_hood, Constants.Hood.Hood_Intake_Setpoint));

    // Real stuff

    setBotHigh.whileTrue(
        new ParallelCommandGroup(
            new SetTelescopeScore(m_arm, m_telescope, coneMode, true), m_OperatorUpDpad));
    setBotHigh
        .and(coneModify)
        .whileTrue(
            new ParallelCommandGroup(
                new SetTelescopeScore(m_arm, m_telescope, coneMode, true),
                m_OperatorUpDpadConeModify));

    setBotMid.whileTrue(
        new ParallelCommandGroup(
            new SetTelescopeScore(m_arm, m_telescope, coneMode, true), m_OperatorMidDpad));
    setBotMid
        .and(coneModify)
        .whileTrue(
            new ParallelCommandGroup(
                new SetTelescopeScore(m_arm, m_telescope, coneMode, true),
                m_OperatorMidDpadConeModify));

    setBotIntake.whileTrue(
        new ParallelCommandGroup(
            new SetTelescopeScore(m_arm, m_telescope, coneMode, false), m_OperatorIntakeDpad));

    setBotIntake
        .and(coneModify)
        .whileTrue(
            new ParallelCommandGroup(
                new SetTelescopeScore(m_arm, m_telescope, coneMode, false),
                new UprightIntakeSequence(m_arm, m_telescope, m_wrist)));

    setBotInside.whileTrue(m_OperatorInsideButton);

    setBotHigh
        .and(reverseIntake)
        .whileTrue(
            new SequentialCommandGroup(
                new SetHood(m_hood, Constants.Hood.Hood_High_Setpoint),
                new ShooterScore(m_shooter, Constants.Shooter.HighScoreSpeed)));

    
    setBotMid
        .and(reverseIntake)
        .whileTrue(
            new SequentialCommandGroup(
                new SetHood(m_hood, Constants.Hood.Hood_Mid_Setpoint),
                new ShooterScore(m_shooter, Constants.Shooter.MidScoreSpeed)));

    setBotInside.and(coneModify).whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(new SetWrist(m_wrist, Constants.Wrist.wrist_cone_intake),new SetTelescope(m_telescope, Constants.Telescope.TELESCOPE_SETPOINT_ZERO)),new SetArm(m_arm, Constants.Arm.ARM_SETPOINT_BOT)));

    runIntake.whileTrue(m_OperatorXButton);
    runIntake.whileFalse(m_OperatorXButtonFalse);
    reverseIntake.whileTrue(m_OperatorBButton);
    reverseIntake.whileFalse(m_OperatorBButtonFalse);

    substationSetpoint.whileTrue(new DoubleSubstation(m_arm, m_telescope, m_wrist));

    toggleMode.onTrue(
        new InstantCommand(
            () -> {
              coneMode = !coneMode;
              m_leds
                  .setAnimationCommand(
                      coneMode ? LEDs.Animation.YELLOW_CHASE : LEDs.Animation.PURPLE_CHASE)
                  .schedule();
            }));
   
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
    return new AutonManager(
        m_drivetrain, m_arm, m_telescope, m_wrist, m_intake, m_shooter, m_hood, m_limelight);
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
