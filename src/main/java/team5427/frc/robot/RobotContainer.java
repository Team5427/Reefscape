package team5427.frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import team5427.frc.robot.Constants.SwerveConstants;
import team5427.frc.robot.commands.AllCommands;
import team5427.frc.robot.io.OperatingControls;
import team5427.frc.robot.io.PilotingControls;
import team5427.frc.robot.subsystems.LightsSubsystem;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.Vision.VisionSubsystem;

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

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    LightsSubsystem.getInstance();

    try {
      Constants.config = RobotConfig.fromGUISettings();
      System.out.println("Robot Config Loaded");
      System.out.println(
          "Module Count: "
              + Constants.config.numModules
              + " Max Torque Friction: "
              + Constants.config.maxTorqueFriction
              + " Wheel Friction Force: "
              + Constants.config.wheelFrictionForce);
    } catch (Exception e) {
      // Handle exception as needed
      System.out.println("Robot Config Failing");
      e.printStackTrace();
    }
    SwerveSubsystem.getInstance(RobotState.getInstance()::addOdometryMeasurement);
    VisionSubsystem.getInstance(
        Optional.of(RobotState.getInstance()::addVisionMeasurement),
        Optional.of(RobotState.getInstance()::getEstimatedPose),
        Optional.of(RobotState.getInstance()::getOdometryHeading));
    createNamedCommands();

    // Configure AutoBuilder last
    AutoBuilder.configure(
        RobotState.getInstance()::getAdaptivePose, // Robot pose supplier
        RobotState.getInstance()
            ::resetAllPose, // Method to reset odometry (will be called if your auto has a
        // starting pose)
        SwerveSubsystem.getInstance()
            ::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        SwerveSubsystem.getInstance()::setInputSpeeds,
        // (speeds) ->
        //     SwerveSubsystem.getInstance()
        //         .setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT
        // RELATIVE
        // ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(SwerveConstants.kRotationalKp , 0.0, 0.0), // Translation PID constants
            new PIDConstants(SwerveConstants.kTranslationalKp  , 0.0, 0.0) // Rotation PID constants
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

    // VisionSubsystem vision = new
    // VisionSubsystem(SwerveSubsystem.getInstance()::addVisionMeasurement);
    // new InstantCommand(() -> {

    // });
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
        autoChooser = AutoBuilder.buildAutoChooser();
          
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     SwerveSubsystem.getInstance(Optional.empty())
    //         .sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     SwerveSubsystem.getInstance(Optional.empty())
    //         .sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)",
    //     SwerveSubsystem.getInstance(Optional.empty())
    //         .sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)",
    //     SwerveSubsystem.getInstance(Optional.empty())
    //         .sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();
  }

  private void createNamedCommands() {
    NamedCommands.registerCommand("Score L3", AllCommands.scoreL3);
    NamedCommands.registerCommand("Score L4", AllCommands.scoreL4);
    NamedCommands.registerCommand("Eject Coral", AllCommands.eject);
    NamedCommands.registerCommand("Intake Station", AllCommands.intake);
    NamedCommands.registerCommand("Reset All", AllCommands.resetSubsystems);
    NamedCommands.registerCommand("Stop Chassis", new InstantCommand(()-> {SwerveSubsystem.getInstance().setInputSpeeds(new ChassisSpeeds(0,0,0));}));
  }

  public void configureButtonBindings() {
    new PilotingControls();
    new OperatingControls();
    // new LightTriggers();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
