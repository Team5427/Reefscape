package team5427.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team5427.frc.robot.RobotContainer;
import team5427.lib.kinematics.inverse.ArmInverseKinematics;
import team5427.lib.kinematics.inverse.ArmInverseKinematics.VariableLengthArm;

public class InverseKinematicTest {

    public InverseKinematicTest(){
        // createRobotContainer();
        // inverseKinematicsTest();
    }
    
    public void createRobotContainer() {
    // Instantiate RobotContainer
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void inverseKinematicsTest(){
    ArmInverseKinematics pIk = new ArmInverseKinematics();
    VariableLengthArm ik = pIk.new VariableLengthArm(new double[]{0.2}, Rotation2d.fromDegrees(90), new Translation3d(10, 0, 10));
    SmartDashboard.putString("Variable Inverse Kinematic Result", ik.getAngleAndArmLength().toString());
  }
}
