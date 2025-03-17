package team5427.frc.robot.commands.outtake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

// import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.EETask;

public class EjectGamePiece extends Command {

  private ProngSubsystem prongSubsystem;

  private boolean isCoral;

  public EjectGamePiece(boolean isCoral) {
    prongSubsystem = ProngSubsystem.getInstance();
    addRequirements(prongSubsystem);
    this.isCoral = isCoral;
  }

  @Override
  public void initialize() {
    // ProngSubsystem.task = EETask.EJECTING;
    prongSubsystem.setRollerVelocity(MetersPerSecond.of(4.0 * (isCoral ? .25 : -1.1)));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    prongSubsystem.setRollerVelocity(MetersPerSecond.of(0.0));
  }
}
