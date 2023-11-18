// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleDrive;
import frc.robot.commands.Climber.CurrentZero;
import frc.robot.commands.Climber.CurrentZeroLeft;
import frc.robot.commands.Climber.CurrentZeroRight;
import frc.robot.commands.auton.Autos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Climber climber = Climber.getInstance();
    private final ShooterHood shooterHood = ShooterHood.getinstance();
    private final Shooter shooter = Shooter.getInstance();
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem
            .getInstance(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final Conveyer conveyer = Conveyer.getInstance();
    private final Infeed infeed = Infeed.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final SendableChooser<CommandBase> autonChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        initAutonChooser();
    }

    private void initAutonChooser() {
        autonChooser.setDefaultOption("not gayyyyy", Autos.notGayAuto(swerveSubsystem));
        autonChooser.addOption("e ^ i * pi = -1", Autos.eAuto(swerveSubsystem));
        autonChooser.addOption("not gay with map", Autos.notGayEventMapAuto(swerveSubsystem));
        autonChooser.addOption("e2", Autos.e2Path(swerveSubsystem, limelight));
        SmartDashboard.putData("Auto Choices", autonChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(new TeleDrive(swerveSubsystem,
                () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> true, false, false/* DON'T USE TRUE!! */, this::getScalar));

        m_driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        // m_driverController.x().whileTrue(swerveSubsystem.xxDrivexx());
        m_driverController.leftTrigger().onTrue(infeed.runInfeedCommand(.85)).onFalse(infeed.stopInfeedCommand());
        // m_driverController.y().onTrue(conveyer.runConveyerCommand(.85)).onFalse(conveyer.stopConveyerCommand());
        // m_driverController.rightBumper().onTrue(infeed.runInfeedCommand(-.85)).onFalse(infeed.stopInfeedCommand());
        m_driverController.rightBumper().onTrue(climber.runGrippySolenoidCommand());
        m_driverController.leftBumper().onTrue(infeed.setInfeedDownCommand(!infeed.getInfeedDown()));
        // m_driverController.x().onTrue(shooter.runShooterCommand(.75)).onFalse(shooter.stopShooterCommand());
        // m_driverController.a().onTrue(shooterHood.runShooterHoodMotorCommand(.05)).onFalse(shooterHood.stopShooterHoodMotorCommand());
        // m_driverController.b().onTrue(shooterHood.runShooterHoodMotorCommand(-.05)).onFalse(shooterHood.stopShooterHoodMotorCommand());
        //m_driverController.a().onTrue(shooterHood.setShooterHoodPositionCommand(15));
        m_driverController.b().onTrue(shooterHood.setShooterHoodPositionCommand(0));
        m_driverController.x().onTrue(climber.setClimberMotorPositionCommand(2));
        m_driverController.y().onTrue(climber.setClimberMotorPositionCommand(140));
        m_driverController.a().onTrue(new CurrentZero(climber));  
       
        //m_driverController.x().onTrue(climber.runClimberMotorsCommand(-.1)).onFalse(climber.stopClimberMotorsCommand());
        //m_driverController.y().onTrue(climber.runClimberMotorsCommand(.1)).onFalse(climber.stopClimberMotorsCommand());
    }

    private double getScalar() {
        return m_driverController.getRightTriggerAxis() * 0.50 + 0.5;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autonChooser.getSelected();
    }
}
