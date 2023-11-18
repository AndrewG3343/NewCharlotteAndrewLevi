// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class CurrentZeroRight extends CommandBase {
  Climber m_climber;
  /** Creates a new CurrentZero. */
  public CurrentZeroRight(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.runClimberMotorRightCommand(-.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.runClimberMotorRightCommand(-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimberMotorRightCommand(0);
    m_climber.setRightEncoderPos(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.rightCurrent() > 5;
  }
}
