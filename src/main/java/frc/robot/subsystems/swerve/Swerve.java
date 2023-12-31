// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.sql.Driver;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.GyroIO.GyroData;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.Constants.DriveConstants;

/***
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 * @author Harkirat
 * 
 *         Subsystem class for swerve drive, used to manage four swerve modules
 *         and set their states. Also includes a pose estimator, gyro, and
 *         logging information
 */
public class Swerve extends SubsystemBase {
    private SwerveModuleIO[] modules = new SwerveModuleIO[4];
    private ModuleData[] moduleData = new ModuleData[4];

    private GyroIO gyro;
    private GyroData gyroData;
    // equivilant to a odometer, but also intakes vision
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final PIDController turnController = new PIDController(0.0335, 0.00, 0);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    public Swerve() {

        if (Constants.isSim) {
            gyro = new GyroIO() {
            };

            gyroData = new GyroData();
            for (int i = 0; i < 4; i++) {
                modules[i] = new SwerveModuleSim();
                moduleData[i] = new ModuleData();
            }
        } else if (!Constants.isSim) {

        }

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics,
                new Rotation2d(0),
                new SwerveModulePosition[] { moduleData[0].position, moduleData[1].position, moduleData[2].position,
                        moduleData[3].position },
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));

        turnController.enableContinuousInput(-180, 180);

    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
               // 5. Convert chassis speeds to individual module states
               SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
       
               // 6. Output each module states to wheels
               setModuleStates(moduleStates);
    }

    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i<4; i++){
            states[i] = new SwerveModuleState(moduleData[i].driveVelocityMPerSec, new Rotation2d( moduleData[i].turnAbsolutePositionRad));
        }
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }

    public void resetGyro() {
        gyro.resetGyro();
        System.out.println("RESET");
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroData.yaw / 180 * Math.PI);
    }

    public Pose2d getPose() {

        Pose2d estimatedPose = swerveDrivePoseEstimator.getEstimatedPosition();

        return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
    }

    public SwerveDrivePoseEstimator getPoseEstimator(){
        return swerveDrivePoseEstimator;
    }
 
    public void resetOdometry(Pose2d pose) {


        swerveDrivePoseEstimator.resetPosition(getRotation2d(),
                new SwerveModulePosition[] { moduleData[0].position, moduleData[1].position, moduleData[2].position,
                        moduleData[3].position },
                pose);
    }

    public void updateOdometry() {

        swerveDrivePoseEstimator.update(getRotation2d(),
                new SwerveModulePosition[] { moduleData[0].position, moduleData[1].position, moduleData[2].position,
                        moduleData[3].position });

    }


    public void stopModules() {
        for (SwerveModuleIO module : modules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(desiredStates[i], moduleData[i]);
        }
    }

    /***
     * 
     * @param angle the angle to move at, in degrees, -180 to 180
     * @param speed the speed to move at, 0-1
     */
    public void moveAtAngle(double angle, double speed) {
        angle = Math.toRadians(angle);
        double xSpeed = Math.cos(angle) * speed;
        double ySpeed = -Math.sin(angle) * speed;
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, 0, getRotation2d());

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }

    /***
     * 
     * @param angle the rotational angle to move to, -180 to 180
     */
    public void turnToRotation(double angle) {
        // negative so that we move towards the target, not away
        double turning_speed = -turnController.calculate(getRotation2d().getDegrees(), angle);
        turning_speed = turningLimiter.calculate(turning_speed);
        // turning_speed = Math.abs(turning_speed) * Math.signum(getHeading());

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, turning_speed, getRotation2d());
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(chassisSpeeds);
        // 6. Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public double getVerticalTilt() {
        return gyroData.pitch;
    }


    @Override
    public void periodic() {
        updateOdometry();

        for (int i = 0; i < 4; i++) {
            modules[i].updateData(moduleData[i]);
        }


        SmartDashboard.putNumberArray("Odometry",
                new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
        
        double[] realStates = {
                moduleData[0].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[0].driveVelocityMPerSec,
                moduleData[1].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[1].driveVelocityMPerSec,
                moduleData[2].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[2].driveVelocityMPerSec,
                moduleData[3].turnAbsolutePositionRad / Math.PI * 180,
                moduleData[3].driveVelocityMPerSec };


        double[] theoreticalStates = {
                moduleData[0].theoreticalState.angle.getDegrees(),
                moduleData[0].theoreticalState.speedMetersPerSecond,
                moduleData[1].theoreticalState.angle.getDegrees(),
                moduleData[1].theoreticalState.speedMetersPerSecond,
                moduleData[2].theoreticalState.angle.getDegrees(),
                moduleData[2].theoreticalState.speedMetersPerSecond,
                moduleData[3].theoreticalState.angle.getDegrees(),
                moduleData[3].theoreticalState.speedMetersPerSecond,
        };

        SmartDashboard.putNumberArray("Theoretical States", theoreticalStates);
        SmartDashboard.putNumberArray("Real Staets", realStates);
        SmartDashboard.putNumber("Yaw", getRotation2d().getDegrees());
        SmartDashboard.putNumber("Pitch", getVerticalTilt());
        SmartDashboard.putNumber("Robot Pose X", getPose().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
    }
}