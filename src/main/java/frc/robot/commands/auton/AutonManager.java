// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.Balance;
import frc.robot.commands.Drivetrain.FollowPath;
import frc.robot.commands.InsideBot.InsideBot;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Intake.ReverseCone;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.SideCone.Intake.SideIntakeSequence;
import frc.robot.commands.SideCone.Score.SideScoringSequence;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.UprightCone.Score.SetpointUprightScore;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;
import frc.robot.utility.BezierCurve;
import frc.robot.utility.Path;

// Gets states from NetworkTables (published by ros) during auton
public class AutonManager extends CommandBase {
  private Drivetrain drivetrain;
  private Arm arm;
  private Telescope telescope;
  private Wrist wrist;
  private Intake intake;
  private Shooter shooter;

  private Path[] paths;
  private Command currentFollowPathCommand;
  private int previousPath = -1;
  private Command currentBalanceCommand;
  private Command currentArmCommand;
  private Command currentIntakeCommand;
  private Command currentShooterCommand;

  public AutonManager(
      Drivetrain drivetrain, Arm arm, Telescope telescope, Wrist wrist, Intake intake, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.telescope = telescope;
    this.wrist = wrist;
    this.intake = intake;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updatePaths();
    drivetrain.setPose();
    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");
    SmartDashboard.getEntry("/auto/arm/state").setString("none");

    SmartDashboard.putBoolean("/auto/state", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stop the drivetrain if a new path was not started
    startNTPath();
    startNTBalance();
    updateNTArm();
    updateNTIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentFollowPathCommand.cancel();

    SmartDashboard.putBoolean("/auto/state", false);

    if (DriverStation.getAlliance() == Alliance.Red) {
      double flippedRotation = Math.PI - drivetrain.getPose().getRotation().getRadians();
      drivetrain.zeroGyroscope(new Rotation2d(flippedRotation));
    }

    System.out.println("Auton ended!");
  }

  private boolean startNTBalance() {
    if (currentBalanceCommand != null) return false;

    String balanceSet = SmartDashboard.getEntry("/auto/balance/set").getString("false false");

    String[] balanceSetArr = balanceSet.split(" ");

    if (balanceSetArr.length != 2) return false;

    boolean startBalance = Boolean.parseBoolean(balanceSetArr[0]);
    boolean reversed = Boolean.parseBoolean(balanceSetArr[1]);

    if (!startBalance) return false;

    currentFollowPathCommand.end(true);

    currentBalanceCommand = new Balance(drivetrain, reversed);
    currentBalanceCommand.schedule();

    System.out.println("Starting balance!!!");
    return true;
  }

  private void updateNTIntake() {
    String state = SmartDashboard.getEntry("/auto/intake/set").getString("idle");

    switch (state) {
      case "intake":
        if (currentIntakeCommand != null
            && (currentIntakeCommand instanceof RunIntake && currentIntakeCommand.isScheduled()))
          break;
        currentIntakeCommand.end(true);
        currentIntakeCommand = new RunIntake(intake);
        currentIntakeCommand.schedule();
        break;
      case "cone":
        if (currentIntakeCommand != null
            && (currentIntakeCommand instanceof ReverseCone && currentIntakeCommand.isScheduled()))
          break;
        currentIntakeCommand.end(true);
        currentIntakeCommand = new ReverseCone(intake);
        currentIntakeCommand.schedule();
        break;

      case "cube":
        if (currentIntakeCommand != null
            && (currentIntakeCommand instanceof ReverseCone && currentIntakeCommand.isScheduled()))
          break;
        currentIntakeCommand.end(true);
        currentIntakeCommand = new ReverseCone(intake);
        currentIntakeCommand.schedule();
        break;
      case "idle":
      default:
        currentIntakeCommand = new IdleIntake(intake, true);
        currentIntakeCommand.schedule();
    }
  }

  private void updateNTArm() {
    String state = SmartDashboard.getEntry("/auto/arm/set").getString("none");

    System.out.println("State: " + state);

    if (state.equals("none")
        || (this.currentArmCommand != null && this.currentArmCommand.isScheduled())) {
      if (state.equals("move_intake")) {
        System.out.println("Canceling arm command!!!");
      }
      return;
    }

    switch (state) {
      case "move_intake":
        this.currentArmCommand =
            new SideIntakeSequence(arm, telescope, wrist)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("intake");
                    });
        System.out.println("RUNNING INTAKE!!!\n\n\n");
        break;

      case "move_cube_high":
      case "move_cone_high":
        this.currentArmCommand =
            new SideScoringSequence(arm, telescope, wrist, 1, true)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("high");
                    });
        break;

      case "move_cube_mid":
      case "move_cone_mid":
        this.currentArmCommand =
            new SideScoringSequence(arm, telescope, wrist, 0, true)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("mid");
                    });
        break;

      case "move_cube_low":
      case "move_cone_low":
        SmartDashboard.getEntry("/auto/arm/state").setString("cone_intake");
        this.currentArmCommand =
            new SetpointUprightScore(arm, telescope, wrist, 3)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("low");
                    });

        break;

      case "move_inside_bot":
        this.currentArmCommand =
            new InsideBot(arm, telescope, wrist)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("inside");
                    });

        break;

      case "retract":
      default:
        this.currentArmCommand =
            new SetTelescope(telescope, 0.0)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("retract");
                    });
    }

    this.currentArmCommand.schedule();

    SmartDashboard.getEntry("/auto/arm/set").setString("none");
  }

  private void updateNTShooter() {
    String state = SmartDashboard.getEntry("/auto/shooter/set").getString("idle");

    switch (state) {
      case "prime_high":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "prime_mid":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "prime_low":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "intake":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "shoot_high":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "shoot_mid":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "shoot_low":
        // TODO: Schedule command
        currentShooterCommand.schedule();
      case "idle":
      default:
        // TODO: Schedule command
        currentShooterCommand.schedule();
    }
  }

  // Starts the path specified by ROS in NetworkTables
  // Returns whether a path was started
  private boolean startNTPath() {
    if (currentFollowPathCommand != null && currentFollowPathCommand.isScheduled()) return false;

    Number[] indexes =
        SmartDashboard.getEntry("/pathTable/startPathIndex").getNumberArray(new Number[0]);

    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    for (Number index : indexes) {
      int i = (int) index.doubleValue();

      System.out.println("Starting path" + i);

      if (i < 0 || i >= paths.length || i <= previousPath) return false;

      commandGroup.addCommands(new FollowPath(this.drivetrain, this.paths[i]));

      previousPath = i;
    }

    currentFollowPathCommand = commandGroup;

    commandGroup.schedule();

    SmartDashboard.getEntry("/pathTable/startPathIndex").setNumberArray(new Number[0]);

    return true;
  }

  // Grabs all paths from NetworkTables and stores it in this.paths
  private void updatePaths() {
    int numPaths = SmartDashboard.getEntry("/pathTable/num_paths").getNumber(0).intValue();

    this.paths = new Path[numPaths];

    for (int i = 0; i < numPaths; i++) {
      this.paths[i] = getPath(i);
    }
  }

  // Grabs a path of specified index from NetworkTables
  private Path getPath(int pathIndex) {
    String pathRoot = "/pathTable/path" + pathIndex;

    // Pull properties of the path from NetworkTables
    double timeSeconds = SmartDashboard.getEntry(pathRoot + "/time").getNumber(0).doubleValue();

    Rotation2d startHeading =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                SmartDashboard.getEntry(pathRoot + "/start_heading").getNumber(0).doubleValue()));

    Rotation2d endHeading =
        Rotation2d.fromRadians(
            MathUtil.angleModulus(
                SmartDashboard.getEntry(pathRoot + "/end_heading").getNumber(0).doubleValue()));

    boolean stopAtEnd = SmartDashboard.getEntry(pathRoot + "/stop_at_end").getBoolean(true);

    double maxAcceleration =
        SmartDashboard.getEntry(pathRoot + "/max_acceleration").getNumber(5.0).doubleValue();

    // Pull control points of bezier curve from NetworkTables

    Translation2d[] control_points = new Translation2d[4];

    for (int i = 0; i < 4; i++) {
      String controlPointRoot = pathRoot + "/control_point" + i;

      double x = SmartDashboard.getEntry(controlPointRoot + "/X").getNumber(0).doubleValue();
      double y = SmartDashboard.getEntry(controlPointRoot + "/Y").getNumber(0).doubleValue();

      control_points[i] = new Translation2d(x, y);
    }

    BezierCurve curve = new BezierCurve(control_points);

    double startVelocity = pathIndex == 0 ? 0.0 : paths[pathIndex - 1].getEndVelocity();

    return new Path(
        curve,
        startHeading,
        endHeading,
        startVelocity,
        stopAtEnd,
        maxAcceleration,
        pathIndex,
        timeSeconds);
  }
}
