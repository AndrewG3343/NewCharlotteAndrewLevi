// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.Constants.Auton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {

    public static Command eAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory e = PathPlanner.loadPath("e", new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));

        return Commands.sequence(new FollowTrajectory(swerve, e, true));
    }

    public static Command notGayAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory notGay = PathPlanner.loadPath("straight just like me",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));

        return Commands.sequence(new FollowTrajectory(swerve, notGay, true));
    }

    public static Command notGayEventMapAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory notGayMap = PathPlanner.loadPath("straight map",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("print", new InstantCommand(() -> swerve.thingY = true));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerve::getRobotPose,
                swerve::resetOdometry,
                new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
                new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
                swerve::setChassisSpeeds,
                eventMap,
                swerve);
        return Commands.sequence(autoBuilder.fullAuto(notGayMap));
    }

    public static Command e2Path(SwerveSubsystem swerve, Limelight limelight) {
        PathPlannerTrajectory e2Traj = PathPlanner.loadPath("e2",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("e2Event", new InstantCommand(() -> {
            var pose = limelight.getEstimatedPos();
            if (pose.isPresent()) {
                var poseUnwrapped = pose.get();
                swerve.addVisionMeasurement(poseUnwrapped.estimatedPose.toPose2d(), poseUnwrapped.timestampSeconds,
                        false,
                        0.5);
            }

        }));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerve::getRobotPose,
                swerve::resetOdometry,
                new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
                new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
                swerve::setChassisSpeeds,
                eventMap,
                swerve);
        return Commands.sequence(autoBuilder.fullAuto(e2Traj));
    }

    public static Command bruhPath(SwerveSubsystem swerve, Climber climber) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Bruh PAth", 2, 2);

        Command bruh = swerve.getTrajectoryCommand(traj, true);

        return bruh;
    }

    public static Command zestyPath(SwerveSubsystem swerve, Climber climber) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Zesty ahh Path", 2, 2);

        Command zest = swerve.getTrajectoryCommand(traj, true);

        return zest;
    }

    public static Command zestyInfeedPath(SwerveSubsystem swerve, Infeed infeed, Shooter shooter, Conveyer conveyer) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Zesty Infeed", 2, 2);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("infed", infeed.setInfeedDownCommand(true).andThen(infeed.runInfeedCommand(.7)).withTimeout(2).andThen(infeed.stopInfeedCommand()));
        eventMap.put("Shooter", shooter.runShooterCommand(.3).alongWith(conveyer.runConveyerCommand(.5)).withTimeout(2.5).andThen(conveyer.stopConveyerCommand(),shooter.stopShooterCommand()));
        SwerveAutoBuilder autoBuilder  = new SwerveAutoBuilder(
            swerve::getRobotPose,
            swerve::resetOdometry,
            new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
            swerve::setChassisSpeeds,
            eventMap,
            swerve);
        return Commands.sequence(autoBuilder.fullAuto(traj));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

}
