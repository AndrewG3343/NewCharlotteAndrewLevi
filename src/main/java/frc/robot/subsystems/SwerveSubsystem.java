// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;
    public boolean thingY = false;

    private Field2d m_field = new Field2d();
    private static SwerveSubsystem instance;

    /** Creates a new SwerveSubsystem. */
    private SwerveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (IOException e) {
            System.exit("https://www.youtube.com/watch?v=dQw4w9WgXcQ".hashCode());
        }

        setBrake(true);
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward((0.19 + 0.225 + 0.214 + 0.2256) / 4.0,
                (2.2565 + 2.2785 + 2.2754 + 2.291) / 4.0,
                (0.277 + 0.31) / 2.0));
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, boolean soft, Matrix<N3, N1> visionSTDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, soft, visionSTDevs);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, boolean soft, double trustWothiness) {
        swerveDrive.addVisionMeasurement(pose, timestamp, soft, trustWothiness);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public SwerveController getController() {
        return swerveDrive.swerveController;
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
                getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(),
                getHeading().getRadians());
    }

    public Rotation3d getGyroRotation() {
        return swerveDrive.getGyroRotation3d();
    }

    public ChassisSpeeds getFORobotVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Locks the wheels by pointing them all toward the center of the Robot
     */
    public Command xxDrivexx() {
        return run(swerveDrive::lockPose);
    }

    public void resetOdometry(Pose2d initHoloPos) {
        swerveDrive.resetOdometry(initHoloPos);
    }

    public void setBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        swerveDrive.setModuleStates(states, false);
    }

    public Command getTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getRobotPose, // Pose supplier
                        swerveDrive.kinematics, // SwerveDriveKinematics
                        new PIDController(4, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(4, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(4, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }
    /*  */

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        m_field.setRobotPose(getRobotPose());
        SmartDashboard.putBoolean("thingyyyy", thingY);
        SmartDashboard.putNumber("Heading", swerveDrive.getPose().getRotation().getDegrees());

        SmartDashboard.putData(m_field);
        // This method will be called once per scheduler run
    }

    public static SwerveSubsystem getInstance(File directory) {
        return instance == null ? instance = new SwerveSubsystem(directory) : instance;
    }
}