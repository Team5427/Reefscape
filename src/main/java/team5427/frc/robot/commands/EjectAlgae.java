package team5427.frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class EjectAlgae extends Command {

  private ProngSubsystem prongSubsystem;

  private Timer timer;

  public EjectAlgae() {
    prongSubsystem = ProngSubsystem.getInstance();
    addRequirements(prongSubsystem);

    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute() {
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kBargePosition);
    if (timer.get() > 1.0) {
      prongSubsystem.setRollerSetpoint(MetersPerSecond.of(-8.0));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kZeroPosition);
    prongSubsystem.setRollerSetpoint(MetersPerSecond.of(0.0));
  }
}
