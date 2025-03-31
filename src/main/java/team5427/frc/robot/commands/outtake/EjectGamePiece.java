package team5427.frc.robot.commands.outtake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import team5427.frc.robot.RawScoringConfiguration;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

// import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem.EETask;

public class EjectGamePiece extends Command {

  private ProngSubsystem prongSubsystem;

  private Optional<RawScoringConfiguration> scoringConfiguration;

  private boolean isCoral;

  public EjectGamePiece(boolean isCoral, Optional<RawScoringConfiguration> scoringConfig) {
    prongSubsystem = ProngSubsystem.getInstance();
    this.scoringConfiguration = scoringConfig;
    addRequirements(prongSubsystem);
    this.isCoral = isCoral;
  }

  @Override
  public void execute() {
    // ProngSubsystem.task = EETask.EJECTING;
    if (scoringConfiguration.isPresent()) {
      prongSubsystem.setRollerVelocity(scoringConfiguration.get().getKCoralRollerVelocity());
    }
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
