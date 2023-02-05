package frc.robot;

public class RobotContainer {

  public final XboxController d_controller = new XboxController(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

 // Subsystems
  private final Drivetrain m_drivetrain = new Drivetrain();

  public RobotContainer() {

    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new SwerveDrive(
            m_drivetrain,
            () -> -modifyAxis(d_controller.getRawAxis(translationAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(strafeAxis)),
            () -> -modifyAxis(d_controller.getRawAxis(rotationAxis))));

    configureBindings();

  }

    
}
