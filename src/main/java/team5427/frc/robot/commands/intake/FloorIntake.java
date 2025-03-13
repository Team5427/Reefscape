package team5427.frc.robot.commands.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class FloorIntake extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private ProngSubsystem prongSubsystem;

  public FloorIntake() {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    prongSubsystem = ProngSubsystem.getInstance();
    addRequirements(cascadeSubsystem, prongSubsystem);
  }

  @Override
  public void execute() {
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kAlgaeIntakeRotation);
    cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kFloorIntakeDistance);
    prongSubsystem.setRollerVelocity(MetersPerSecond.of(3.0));
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kFloorIntakePosition);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kStowRotation);
    cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
    prongSubsystem.setRollerVelocity(MetersPerSecond.of(2.0));
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kStowPosition);
  }
}
