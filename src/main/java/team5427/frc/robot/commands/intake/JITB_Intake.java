package team5427.frc.robot.commands.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants.CascadeConstants;
import team5427.frc.robot.Constants.SupremeEffectorConstants;
import team5427.frc.robot.RawIntakeConfiguration;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.SupremeEffector.SupremeSubsystem;

public class JITB_Intake extends Command {

  private CascadeSubsystem cascadeSubsystem;
  private SupremeSubsystem jitbIntake;
  private RawIntakeConfiguration configuration;

  public JITB_Intake(RawIntakeConfiguration configuration) {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    jitbIntake = SupremeSubsystem.getInstance();
    addRequirements(jitbIntake);
    this.configuration = configuration;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    cascadeSubsystem.setCascadeSetpoint(configuration.getCascadeHeight());
    cascadeSubsystem.setPivotSetpoint(configuration.getCascadeAngle());
    jitbIntake.setAlgaeRollerSetpoint(configuration.getAlgaeRollerSpeeds());
    jitbIntake.setCoralRollerSetpoint(configuration.getCoralRollerSpeeds());
    jitbIntake.setPivotSetpoint(configuration.getWristAngle());
  }

  @Override
  public boolean isFinished() {
    return jitbIntake.isAlgaeIntaked() || jitbIntake.isCoralIntaked();
  }

  @Override
  public void end(boolean interrupted) {
    cascadeSubsystem.setCascadeSetpoint(CascadeConstants.kStowDistance);
    cascadeSubsystem.setPivotSetpoint(CascadeConstants.kStowRotation);
    jitbIntake.setPivotSetpoint(SupremeEffectorConstants.kStowPosition);
    jitbIntake.setAlgaeRollerSetpoint(
        MetersPerSecond.of(configuration.getAlgaeRollerSpeeds().in(MetersPerSecond) * 0.5));
    jitbIntake.setCoralRollerSetpoint(
        MetersPerSecond.of(configuration.getCoralRollerSpeeds().in(MetersPerSecond) * 0.5));
  }
}
