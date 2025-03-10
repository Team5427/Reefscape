package team5427.frc.robot.commands.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class CoralIntakeTest extends Command {

  private ProngSubsystem prongEffectorSubsystem;
  private CascadeSubsystem cascadeSubsystem;

  public CoralIntakeTest() {
    prongEffectorSubsystem = ProngSubsystem.getInstance();
    cascadeSubsystem = CascadeSubsystem.getInstance();
    addRequirements(prongEffectorSubsystem, cascadeSubsystem);
  }

  @Override
  public void initialize() {
    // ProngSubsystem.state = ProngEffectorStates.CORAL_INTAKE;
  }

  @Override
  public void execute() {
    prongEffectorSubsystem.setRollerSetpoint(MetersPerSecond.of(-1.5));
    prongEffectorSubsystem.setWristSetpoint(ProngEffectorConstants.kIntakePosition);
    cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kIntakeDistance);
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kIntakeRotation);
  }

  @Override
  public boolean isFinished() {
    // return ProngSubsystem.state == ProngEffectorStates.ALGAE_STOWED
    //     || ProngSubsystem.state == ProngEffectorStates.CORAL_STOWED;
    return false;
  }

  @Override
  public void end(boolean interruped) {
    prongEffectorSubsystem.setRollerSetpoint(MetersPerSecond.of(0.0));
    prongEffectorSubsystem.setWristSetpoint(ProngEffectorConstants.kZeroPosition);
    cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kStowRotation);
  }
}
