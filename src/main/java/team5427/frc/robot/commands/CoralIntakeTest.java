package team5427.frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.SuperStructureEnum.EndEffectorStates;
import team5427.frc.robot.subsystems.EndEffector.EndEffectorSubsystem;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class CoralIntakeTest extends Command {

    private EndEffectorSubsystem endEffectorSubsystem;

    public CoralIntakeTest() {
        endEffectorSubsystem = EndEffectorSubsystem.getInstance();
        addRequirements(endEffectorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        EndEffectorSubsystem.state = EndEffectorStates.CORAL_INTAKE;
        endEffectorSubsystem.setCoralRollerSetpoint(MetersPerSecond.of(10.0));
        endEffectorSubsystem.setWristSetpoint(Rotation2d.fromDegrees(180));
        endEffectorSubsystem.setPivotSetpoint(Rotation2d.fromDegrees(90));
    }

    // @Override
    // public boolean isFinished() {
    //     return endEffectorSubsystem.isCoralRollerAtSetpoint() && endEffectorSubsystem.isPivotAtSetpoint()
    //             && endEffectorSubsystem.isWristAtSetpoint() && endEffectorSubsystem.isCoralIntaked;
    //     // return false;
    // }

    @Override
    public void end(boolean interruped) {
        if (!interruped) {
            endEffectorSubsystem.setCoralRollerSetpoint(MetersPerSecond.of(0.0));
            endEffectorSubsystem.setWristSetpoint(Rotation2d.fromDegrees(0));
            endEffectorSubsystem.setPivotSetpoint(Rotation2d.fromDegrees(0));
        }
    }

}