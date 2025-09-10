package team5427.frc.robot.commands.chassis.assistance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class BestAutoAlign extends Command{
    private ProfiledPIDController xController, yController, rotController;
    private boolean isRightScore;
    private SwerveSubsystem driveBase;
    public double tagID = -1;




    public BestAutoAlign(boolean isRightScore, SwerveSubsystem driveBase){
        rotController = new ProfiledPIDController(Constants.SwerveConstants.kAutoAlignRotationalKp, 0.0, 0.0, new Constraints(10.0, 10.0));
        xController = new ProfiledPIDController(Constants.SwerveConstants.kAutoAlignTranslationKp, 0.0, 0.0, new Constraints(10.0, 10.0));
        yController = new ProfiledPIDController(Constants.SwerveConstants.kAutoAlignTranslationKp, 0.0, 0.0, new Constraints(10.0, 10.0));
        this.isRightScore = isRightScore;
        this.driveBase = driveBase;
        addRequirements(driveBase);
        
    }
    @Override
    public void initialize(){
        
        
    }
    
}
