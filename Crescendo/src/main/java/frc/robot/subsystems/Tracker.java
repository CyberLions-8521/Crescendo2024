package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveModuleConstants.*;

public class Tracker extends SubsystemBase{

    private Drive m_drive = Drive.getInstance();
    private static Tracker m_tracker = new Tracker();

    private Field2d field = new Field2d();
    
    private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS, 
        m_drive.getDriveHeading(),
        m_drive.getModulePositions(), 
        new Pose2d());

    public Tracker(){
        SmartDashboard.putData(field);
    }

    public static Tracker getInstance(){
      return m_tracker;
    }    
    

    @Override
    public void periodic() {
        update();
        field.setRobotPose(getPose());
    }

    public void update(){
        m_poseEstimator.update(m_drive.getDriveHeading(), m_drive.getModulePositions());
    }

    public Pose2d getPose(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose){
        m_poseEstimator.resetPosition(m_drive.getDriveHeading(), m_drive.getModulePositions(), newPose);
    }

    public void resetHeading(){
        setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    }
}
