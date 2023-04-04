// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArm;
import frc.robot.commands.DisabledLimelight;
import frc.robot.commands.Drivetrain.Balance2;
import frc.robot.commands.Drivetrain.FollowPath;
import frc.robot.commands.Drivetrain.ReflectiveAlign;
import frc.robot.commands.Hood.SetHood;
import frc.robot.commands.InsideBotSequences.InsideBot;
import frc.robot.commands.Intake.ReverseCone;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Shooter.IdleShooter;
import frc.robot.commands.Shooter.SetShooter;
import frc.robot.commands.Shooter.ShooterScore;
import frc.robot.commands.SideConeSequences.Intake.SideIntakeSequence;
import frc.robot.commands.SideConeSequences.Score.SideScoringArmWrist;
import frc.robot.commands.SideConeSequences.Score.SideScoringParallel;
import frc.robot.commands.SideConeSequences.Score.SideScoringSequence;
import frc.robot.commands.Telescope.SetTelescope;
import frc.robot.commands.Telescope.SetTelescopeScore;
import frc.robot.commands.UprightConeSequences.Intake.UprightIntakeSequence;
import frc.robot.commands.UprightConeSequences.Score.SetpointUprightScore;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.Animation;
import frc.robot.subsystems.Limelight;
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
  private Hood hood;
  private Limelight limelightTop;
  private Limelight limelightBottom;
  private LEDs leds;

  private Path[] paths;
  private Command currentFollowPathCommand;
  private int previousPath = -1;
  private Command currentBalanceCommand;
  private Command currentArmCommand;
  private Command currentVisionCommand;
  private Command currentShooterCommand;

  private DisabledLimelight disabledLimelightCommand;

  private String prevArmState = "";
  private String prevShooterState = "";
  private String prevVisionState = "";
  private Timer timer = new Timer();

  public AutonManager(
      Drivetrain drivetrain,
      Arm arm,
      Telescope telescope,
      Wrist wrist,
      Intake intake,
      Shooter shooter,
      Hood hood,
      Limelight limelightTop,
      Limelight limelightBottom,
      LEDs leds) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.telescope = telescope;
    this.wrist = wrist;
    this.intake = intake;
    this.shooter = shooter;
    this.hood = hood;
    this.limelightTop = limelightTop;
    this.limelightBottom = limelightBottom;
    this.leds = leds;

    disabledLimelightCommand = new DisabledLimelight(limelightTop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    disabledLimelightCommand.schedule();
    arm.resetRotationsCommand().schedule();
    leds.setAnimationCommand(Animation.CRYPTONITE).schedule();

    arm.setSlowMode(true);

    updatePaths();
    drivetrain.setPose();
    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");
    SmartDashboard.getEntry("/auto/arm/state").setString("none");
    SmartDashboard.getEntry("/auto/shooter/state").setString("idle");
    SmartDashboard.getEntry("/auto/shooter/set").setString("idle");
    SmartDashboard.getEntry("/auto/vision/state").setBoolean(false);
    SmartDashboard.getEntry("/auto/vision/set").setString("none");

    Command visionCommand = CommandScheduler.getInstance().requiring(limelightBottom);
    if (visionCommand != null) visionCommand.cancel();

    Command shooterCommand = CommandScheduler.getInstance().requiring(hood);
    if (shooterCommand != null) shooterCommand.cancel();

    prevShooterState = SmartDashboard.getEntry("/auto/shooter/set").getString("idle");
    prevVisionState = SmartDashboard.getEntry("/auto/vision/set").getString("none");

    SmartDashboard.getEntry("/auto/balance/state").setBoolean(false);

    SmartDashboard.putBoolean("/auto/state", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stop the drivetrain if a new path was not started
    startNTPath();
    startNTBalance();
    updateNTArm();
    updateNTShooter();
    updateNTVision();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("/auto/state", false);

    if (currentArmCommand != null) currentArmCommand.cancel();
    if (currentShooterCommand != null) currentShooterCommand.cancel();
    if (currentFollowPathCommand != null) currentFollowPathCommand.cancel();
    if (currentBalanceCommand != null) currentBalanceCommand.cancel();
    if (currentVisionCommand != null) currentVisionCommand.cancel();

    SmartDashboard.getEntry("/pathTable/status/finishedPath").setString("false -1");
    SmartDashboard.getEntry("/auto/arm/state").setString("none");
    SmartDashboard.getEntry("/auto/shooter/state").setString("idle");
    SmartDashboard.getEntry("/auto/shooter/set").setString("idle");
    SmartDashboard.getEntry("/auto/vision/state").setBoolean(false);
    SmartDashboard.getEntry("/auto/vision/set").setString("none");

    System.out.println("Auton ended!");

    disabledLimelightCommand.cancel();
  }

  private void updateNTVision() {
    if (timer.get() < 1) return;

    String state = SmartDashboard.getEntry("/auto/vision/set").getString("none");

    if (currentVisionCommand != null && currentVisionCommand.isScheduled()) return;

    if (state.equals(prevVisionState)) return;
    prevVisionState = state;

    String[] stateArray = state.split(" ");
    if (stateArray.length == 0) return;

    if (stateArray[0].startsWith("cone")) {
      System.out.println("Starting vision " + state);

      this.currentVisionCommand =
          new ReflectiveAlign(
                  drivetrain,
                  limelightBottom,
                  () -> {
                    return 0;
                  },
                  false)
              .andThen(
                  new InstantCommand(
                      () -> {
                        SmartDashboard.getEntry("/auto/vision/state").setBoolean(true);
                      }));
      currentVisionCommand.schedule();
      return;
    } else if (stateArray[0].startsWith("cube")) {
      if (stateArray.length < 2) return;

      int grid = Integer.parseInt(stateArray[1]);

      this.currentVisionCommand =
          new PrintCommand("Add apriltag align")
              .andThen(
                  new InstantCommand(
                      () -> {
                        SmartDashboard.getEntry("/auto/vision/state").setBoolean(true);
                      }));
    } else if (currentVisionCommand != null && currentVisionCommand.isScheduled()) {
      currentVisionCommand.cancel();
      currentVisionCommand = null;
    }

    SmartDashboard.getEntry("/auto/vision/state").setBoolean(false);

    if (currentVisionCommand != null) currentVisionCommand.schedule();
  }

  private boolean startNTBalance() {
    if (currentBalanceCommand != null) return false;

    String balanceSet = SmartDashboard.getEntry("/auto/balance/set").getString("false false");

    String[] balanceSetArr = balanceSet.split(" ");

    if (balanceSetArr.length != 2) return false;

    boolean startBalance = Boolean.parseBoolean(balanceSetArr[0]);
    boolean reversed = Boolean.parseBoolean(balanceSetArr[1]);

    if (!startBalance) return false;

    currentBalanceCommand = new Balance2(drivetrain, reversed);
    currentBalanceCommand.schedule();

    System.out.println("Starting balance!!!");
    return true;
  }

  private void updateNTArm() {
    String state = SmartDashboard.getEntry("/auto/arm/set").getString("none");

    if (state.equals(prevArmState)) return;

    if (currentArmCommand != null) currentArmCommand.cancel();

    prevArmState = state;

    if (state.equals("none")) return;

    switch (state) {
      case "tipped_intake":
        this.currentArmCommand =
          new ParallelCommandGroup(
            new SetTelescopeScore(arm, telescope, true, false).alongWith(new ReverseCone(intake).withTimeout(0.43).andThen(new RunIntake(intake))),
            new SideIntakeSequence(arm, telescope, wrist)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("tipped_intake");
                    })
                );
        break;

      case "standing_intake":
        this.currentArmCommand =
            new UprightIntakeSequence(arm, telescope, wrist)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("standing_intake");
                    })
                .alongWith(new RunIntake(intake));
        break;

      case "full_score_high":
        this.currentArmCommand =
            new SideScoringSequence(arm, telescope, wrist, 1, true)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("full_score_high");
                    });
        break;

      case "full_score_mid":
        this.currentArmCommand =
            new SideScoringSequence(arm, telescope, wrist, 0, true)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("full_score_mid");
                    });
        break;

      case "full_score_low":
        this.currentArmCommand =
            new SetpointUprightScore(arm, telescope, wrist, 3)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("full_score_low");
                    });

        break;

      case "fast_score_high":
        this.currentArmCommand =
            new SideScoringParallel(arm, telescope, wrist, 1, true)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("fast_score_high");
                    });
        break;

      case "pre_score_high":
        this.currentArmCommand =
          new ParallelCommandGroup(
            new SetTelescopeScore(arm, telescope, true, true),
            new SequentialCommandGroup(
                    new SideScoringArmWrist(arm, wrist, 1, true),
                    new SetTelescope(telescope, Constants.Telescope.TELESCOPE_SETPOINT_HIGH))
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("pre_score_high");
                    })
          );
        break;

      case "pre_score_mid":
        this.currentArmCommand =
            new SequentialCommandGroup(
                    new SideScoringArmWrist(arm, wrist, 0, true),
                    new SetTelescope(telescope, Constants.Telescope.TELESCOPE_SETPOINT_MID))
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("pre_score_mid");
                    });
        break;

      case "finish_score_high":
        this.currentArmCommand =
            new SetArm(arm, Constants.Arm.ARM_SETPOINT_HIGH)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("finish_score_high");
                    });
        break;

      case "finish_score_mid":
        this.currentArmCommand =
            new SetArm(arm, Constants.Arm.ARM_SETPOINT_MID)
                .andThen(
                    () -> {
                      SmartDashboard.getEntry("/auto/arm/state").setString("finish_score_mid");
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
            new SetTelescope(telescope, Constants.Telescope.TELESCOPE_SETPOINT_ZERO)
                .alongWith(new ReverseCone(intake).withTimeout(0.7))
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

    if (state.equals(prevShooterState)) return;

    prevShooterState = state;

    if (currentShooterCommand != null && currentShooterCommand.isScheduled())
      currentShooterCommand.cancel();

    switch (state) {
      case "prime_high":
        currentShooterCommand =
            new SetHood(hood, Constants.Hood.Hood_High_Setpoint)
                .andThen(
                    () -> {
                      setNTShooterState("prime_high");
                    })
                .deadlineWith(new IdleShooter(shooter));
        currentShooterCommand.schedule();
        break;
      case "prime_mid":
        currentShooterCommand =
            new SetHood(hood, Constants.Hood.Hood_Mid_Setpoint)
                .andThen(
                    () -> {
                      setNTShooterState("prime_mid");
                    })
                .deadlineWith(new IdleShooter(shooter));
        ;
        currentShooterCommand.schedule();
        break;
      case "prime_low":
        currentShooterCommand =
            new SetHood(hood, Constants.Hood.Hood_Hybrid_Setpoint)
                .andThen(
                    () -> {
                      setNTShooterState("prime_low");
                    })
                .deadlineWith(new IdleShooter(shooter));
        ;
        currentShooterCommand.schedule();
        break;
      case "deploy_intake":
        currentShooterCommand =
            new SequentialCommandGroup(
                new SetHood(hood, Constants.Hood.Hood_Intake_Setpoint)
                    .andThen(
                        () -> {
                          setNTShooterState("intake");
                        })
                    .alongWith(new SetShooter(shooter, Constants.Shooter.IntakeSpeed)));
        currentShooterCommand.schedule();
        break;
      case "shoot_high":
        currentShooterCommand =
            new ShooterScore(shooter, Constants.Shooter.HighScoreSpeed)
                .withTimeout(0.6)
                .andThen(
                    () -> {
                      setNTShooterState("shoot_high");
                    });
        currentShooterCommand.schedule();
        break;
      case "shoot_mid":
        currentShooterCommand =
            new ShooterScore(shooter, Constants.Shooter.MidScoreSpeed)
                .withTimeout(0.6)
                .andThen(
                    () -> {
                      setNTShooterState("shoot_mid");
                    });
        currentShooterCommand.schedule();
        break;
      case "shoot_low":
        currentShooterCommand =
            new ShooterScore(shooter, Constants.Shooter.LowScoreSpeed)
                .withTimeout(0.6)
                .andThen(
                    () -> {
                      setNTShooterState("shoot_low");
                    });
        currentShooterCommand.schedule();
        break;
      case "idle":
      default:
        new IdleShooter(shooter).schedule();

        currentShooterCommand =
            new SetHood(hood, Constants.Hood.Hood_Upright_Setpoint)
                .andThen(
                    () -> {
                      setNTShooterState("idle");
                    });
        currentShooterCommand.schedule();
        break;
    }
  }

  private void setNTShooterState(String state) {
    SmartDashboard.getEntry("/auto/shooter/state").setString(state);
  }

  // Starts the path specified by ROS in NetworkTables
  // Returns whether a path was started
  private boolean startNTPath() {
    if (currentFollowPathCommand != null && currentFollowPathCommand.isScheduled()) return false;

    Number[] indexes =
        SmartDashboard.getEntry("/pathTable/startPathIndex").getNumberArray(new Number[0]);

    if (indexes.length == 0) return false;

    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    for (Number index : indexes) {
      int i = (int) index.doubleValue();

      if (i < 0 || i >= paths.length || i <= previousPath) return false;

      System.out.println("Starting path" + i);

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
