// package team5427.frc.robot.commands.outtake;

// import static edu.wpi.first.units.Units.MetersPerSecond;

// import edu.wpi.first.wpilibj2.command.Command;
// import team5427.frc.robot.subsystems.ProngEffector.ProngSubsystem;

// public class EjectCoral extends Command {

//   private ProngSubsystem prongSubsystem;

//   public EjectCoral() {
//     prongSubsystem = ProngSubsystem.getInstance();
//     addRequirements(prongSubsystem);
//   }

//   @Override
//   public void execute() {
//     prongSubsystem.setRollerVelocity(MetersPerSecond.of(2.0));
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void end(boolean interrupted) {
//     prongSubsystem.setRollerVelocity(MetersPerSecond.of(0.5));
//   }
// }
