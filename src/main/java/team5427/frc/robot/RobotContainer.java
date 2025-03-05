package team5427.frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team5427.frc.robot.io.OperatingControls;
import team5427.frc.robot.io.PilotingControls;

import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.Vision.VisionSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      Constants.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      System.out.println("Robot Config Failing");
      e.printStackTrace();
    }

    factoryCreateSubsystems();

    // Configure AutoBuilder last
    AutoBuilder.configure(
        SwerveSubsystem.getInstance()::getPose, // Robot pose supplier
        SwerveSubsystem.getInstance()
            ::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        SwerveSubsystem.getInstance()
            ::getCurrentChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            SwerveSubsystem.getInstance()
                .setChassisSpeeds(
                    speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        Constants.config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          if (Constants.kAlliance.isPresent()) {
            return Constants.kAlliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        SwerveSubsystem.getInstance() // Reference to this subsystem to set requirements
        );
    // VisionSubsystem vision = new VisionSubsystem(SwerveSubsystem.getInstance()::addVisionMeasurement);
    // new InstantCommand(() -> {
      
    // });
    configureButtonBindings();
  }

  private void factoryCreateSubsystems() {
    SwerveSubsystem.getInstance();
    ProngSubsystem.getInstance();
    CascadeSubsystem.getInstance();
  }

  public void configureButtonBindings() {
    new PilotingControls();
    new OperatingControls();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous

  // }
}
