package team5427.frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

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
        prongSubsystem.setRollerSetpoint(MetersPerSecond.of(5.0 * (isCoral ? 1: -1)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        prongSubsystem.setRollerSetpoint(MetersPerSecond.of(0.0));
    }
    
}
