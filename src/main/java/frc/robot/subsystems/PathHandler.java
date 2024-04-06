package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class PathHandler {
    private final Drive m_drive;
    private final Tracker m_tracker;

    public PathHandler(Drive drive, Tracker tracker) {
        m_drive = drive;
        m_tracker = tracker;
        configEvents();
        configAutoBuilder();
    }

    public void configEvents(){
        NamedCommands.registerCommand("print", new PrintCommand("Hello World!"));
    }

    public void configAutoBuilder(){
        AutoBuilder.configureHolonomic(
            m_tracker::getPose, // Robot pose supplier
            m_tracker::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            m_drive::getRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            m_drive::driveFromChassis, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            m_drive // Reference to this subsystem to set requirements
        );
    }

    public Command getAuto(String fileName){
        return AutoBuilder.buildAuto(fileName);
    }

    public Command followPath(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }
}
