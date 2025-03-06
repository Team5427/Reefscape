package team5427.frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.ProngEffectorConstants;
import team5427.frc.robot.RawIntakeConfiguration;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

public class Intake extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private ProngSubsystem prongSubsystem;

  private RawIntakeConfiguration config;

  public Intake(RawIntakeConfiguration config) {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    prongSubsystem = ProngSubsystem.getInstance();
    addRequirements(cascadeSubsystem, prongSubsystem);

    this.config = config;
  }

  @Override
  public void initialize() {
    cascadeSubsystem.setPivotSetpoint(config.getCascadeAngle());
    cascadeSubsystem.setCascadeSetpoint(config.getCascadeHeight());
    prongSubsystem.setWristSetpoint(config.getWristAngle());
    prongSubsystem.setRollerSetpoint(config.getRollerSpeeds().times(config.isCoral() ? -1 : 1));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    prongSubsystem.setWristSetpoint(ProngEffectorConstants.kStowPosition);
    prongSubsystem.setRollerSetpoint(
        MetersPerSecond.of(
            Math.copySign(config.getRollerSpeeds().magnitude() / 3, config.isCoral() ? -1 : 1)));
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kStowRotation);
  }
}
