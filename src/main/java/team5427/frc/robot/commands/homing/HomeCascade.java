package team5427.frc.robot.commands.homing;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.Cascade.CascadeSubsystem;
import team5427.frc.robot.subsystems.Climb.ClimberSubsystem;

public class HomeCascade extends Command {

  public static int step = 0;

  private CascadeSubsystem cascadeSubsystem;
  private ClimberSubsystem climberSubsystem;

  public HomeCascade() {
    cascadeSubsystem = CascadeSubsystem.getInstance();
    climberSubsystem = ClimberSubsystem.getInstance();
    addRequirements(cascadeSubsystem, climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.manualRunVoltage(true);
    climberSubsystem.voltageRun(Volt.of(0.5));
  }

  @Override
  public boolean isFinished() {
    if (climberSubsystem.isStalled()) {
      climberSubsystem.setPosition(Rotation2d.kZero);
      climberSubsystem.manualRunVoltage(false);
      climberSubsystem.voltageRun(Volt.zero());
      return true;
    }
    return false;
  }
}
