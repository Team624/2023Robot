// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmTelescopeWrist;
import frc.robot.commands.FunnelSequence;
import frc.robot.commands.Drivetrain.Balance;
import frc.robot.commands.Drivetrain.FollowPath;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
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

  private Path[] paths;
  private Command currentFollowPathCommand;
  private int previousPath = -1;
  private Command currentBalanceCommand;
  private Command currentArmCommand;
  private Command currentIntakeCommand;

  public AutonManager(Drivetrain drivetrain, Arm arm, Telescope telescope, Wrist wrist, Intake intake) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.telescope = telescope;
    this.wrist = wrist;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updatePaths();
    drivetrain.setPose();
    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");

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

    System.out.println("Auton ended!");
  }

  private boolean startNTBalance() {
    if (currentBalanceCommand != null) return false;

    boolean startBalance = SmartDashboard.getEntry("/auto/balance/set").getBoolean(false);

    if (startBalance) {
      currentFollowPathCommand.end(true);

      currentBalanceCommand = new Balance(drivetrain);
      currentBalanceCommand.schedule();

      System.out.println("Starting balance!!!");
    }

    return startBalance;
  }

  private void updateNTVision() {
    String state = SmartDashboard.getEntry("/auto/vision/set").getString("-1 -1");

    int grid = Integer.parseInt(state.split(" ")[0]);
    int column = Integer.parseInt(state.split(" ")[0]);

    System.out.println("Aligning with grid " + grid + " column " + column + " (in my imagination)");
  }

  private void updateNTIntake() {
    String state = SmartDashboard.getEntry("/auto/intake/set").getString("idle");

    System.out.println("State : " + state);

    switch (state) {
      case "intake":
        if (currentIntakeCommand != null && (currentIntakeCommand instanceof RunIntake && currentIntakeCommand.isScheduled())) break;
        currentIntakeCommand.end(true);
        currentIntakeCommand = new RunIntake(intake);
        currentIntakeCommand.schedule();
        break;
      case "cone":
        if (currentIntakeCommand != null && (currentIntakeCommand instanceof ReverseIntake && currentIntakeCommand.isScheduled())) break;
        currentIntakeCommand.end(true);
        arm.cone = true;
        currentIntakeCommand = new ReverseIntake(intake, arm);
        currentIntakeCommand.schedule();
        break;
        
      case "cube":
        if (currentIntakeCommand != null && (currentIntakeCommand instanceof ReverseIntake && currentIntakeCommand.isScheduled())) break;
        currentIntakeCommand.end(true);
        arm.cone = false;
        currentIntakeCommand = new ReverseIntake(intake, arm);
        currentIntakeCommand.schedule();
        break;
      case "idle":
      default:
        currentIntakeCommand = new IdleIntake(intake);
        currentIntakeCommand.schedule();
    }
  }

  private void updateNTArm() {
    String state = SmartDashboard.getEntry("/auto/arm/set").getString("none");

    if (state.equals("none") || (this.currentArmCommand != null && this.currentArmCommand.isScheduled())) return;

    switch (state) {
      case "move_intake":
        this.currentArmCommand = new ArmTelescopeWrist(arm, telescope, wrist, 2).deadlineWith(new RunIntake(intake)).andThen(() -> {
          SmartDashboard.getEntry("/auto/arm/state").setString("intake");
        });
        break;

      case "intake":
        this.currentArmCommand = new RunIntake(intake);
        break;

      case "move_cube_high":
      case "move_cone_high":
        this.currentArmCommand = new ArmTelescopeWrist(arm, telescope, wrist, 4).andThen(() -> {
          SmartDashboard.getEntry("/auto/arm/state").setString("high");
        });
        break;

      case "move_cube_mid":
      case "move_cone_mid":
        this.currentArmCommand = new ArmTelescopeWrist(arm, telescope, wrist, 3).andThen(() -> {
          SmartDashboard.getEntry("/auto/arm/state").setString("mid");
        });
        break;

      case "move_cube_low":
      case "move_cone_low":
        SmartDashboard.getEntry("/auto/arm/state").setString("cone_intake");
        this.currentArmCommand = new ArmTelescopeWrist(arm, telescope, wrist, 1).andThen(() -> {
          SmartDashboard.getEntry("/auto/arm/state").setString("low");
        });

        break;

      case "place":
        this.currentArmCommand = new ReverseIntake(intake, arm);
        break;

      case "retract":
      default:
        this.currentArmCommand = new FunnelSequence(arm, telescope, wrist);
        SmartDashboard.getEntry("/auto/arm/state").setString("retract");
    }

    this.currentArmCommand.schedule();

    SmartDashboard.getEntry("/autl/arm/set").setString("none");
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
