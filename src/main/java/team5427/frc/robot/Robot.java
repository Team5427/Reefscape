package team5427.frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.BuildConstants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

// import frc.robot.BuildConstants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private SwerveSubsystem m_swerve;
  private CascadeSubsystem m_cascade;
  private ProngSubsystem m_endEffector;
  private ClimberSubsystem m_climber;

  // private SteelTalonFX talon0;
  // private SteelTalonFX talonSteer0;
  // SwerveModule module;
  // private SteelTalonFX steerMotor = new
  // SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontLeftModuleIdx]);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @SuppressWarnings("resource")
  public Robot() {

    Logger.recordMetadata("Reefscape", "Steel Talons 5427 Robot Code for the Game Reefscape, 2025");
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    m_swerve = SwerveSubsystem.getInstance();
    m_cascade = CascadeSubsystem.getInstance();
    m_endEffector = ProngSubsystem.getInstance();
    m_climber = ClimberSubsystem.getInstance();

    if (RobotBase.isReal()) {
      Constants.currentMode = Constants.Mode.REAL;
    } else if (RobotBase.isSimulation()) {
      Constants.currentMode = Constants.Mode.SIM;
    } else {
      Constants.currentMode = Constants.Mode.REPLAY;
    }
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.registerURCL(URCL.startExternal());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    AutoLogOutputManager.addPackage("team5427.lib");

    Logger.start();
    // steerMotor.apply(SwerveConstants.kSteerMotorConfiguration);
    m_robotContainer = new RobotContainer();

    FollowPathCommand.warmupCommand().schedule();

    // module = new SwerveModule(0);
    // SwerveSubsystem subsystem = SwerveSubsystem.getInstance();
    // MagicSteelTalonFX talon1 = new
    // MagicSteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[1]);
    // talon0 = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[0]);
    // talon0.apply(SwerveConstants.kDriveMotorConfiguration);

    // talonSteer0 = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[0]);
    // talonSteer0.apply(SwerveConstants.kSteerMotorConfiguration);
    // SteelTalonFX talon2 = new
    // SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[2]);
    // SteelTalonFX talon3 = new
    // SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[3]);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.

    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // SwerveSubsystem.state = DrivingStates.INACTIVE;
    // SwerveSubsystem.getInstance().stop();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Logger.recordOutput("ANGLES",steerMotor.getTalonFX().getPosition().getValue().in(Degree));
    // steerMotor.setSetpoint(Rotation2d.fromDegrees(90));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
