package frc.robot.commands.swerve;

import java.util.function.Consumer;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;

public class PathPlannerFollow {
    public static Consumer<Pose2d> pathTargetPose = pose -> SmartDashboard.putNumberArray("Auto Path Pose Targets",
            new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });

    /***
     * @param swerve      the subsystem object. Do not make a new instance
     * @param trajectory  a viable trajectory object containing information
     *                    about where the robot should go
     * @param isFirstPath if it is, it will reset odometry at its current
     *                    position
     * @return a SwerveControllerCommand based on the trajectory
     * @summary takes a trajectory and moves on it
     */
    private static Command followTrajectoryCommand(PathPlannerPath path, boolean isFirstPath,
            Swerve swerve) {
    
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        swerve.resetGyro();
                        swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
                    }
                }),
                new FollowPathWithEvents(
                    new FollowPathHolonomic(
                        path,
                        swerve::getPose, // Robot pose supplier
                        swerve::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                            3, // Max module speed, in m/s
                            Math.sqrt(2*(DriveConstants.kTrackWidth * DriveConstants.kTrackWidth)), // Drive base radius in meters. Distance from robot center to furthest module.
                            new ReplanningConfig() // Default path replanning config. See the API for the options here
                        ),
                        swerve // Reference to this subsystem to set requirements
                    ),
                    path, // FollowPathWithEvents also requires the path
                    swerve::getPose // FollowPathWithEvents also requires the robot pose supplier
                )
                );
               

    }

    public static Command examplePath(Swerve swerve){
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        
        return followTrajectoryCommand(path, true, swerve);
    }
        


}
