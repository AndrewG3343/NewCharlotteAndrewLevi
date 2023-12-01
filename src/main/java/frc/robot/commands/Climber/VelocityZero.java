// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class VelocityZero extends CommandBase {
  Climber m_climber;
  Timer m_timer;
  /** Creates a new CurrentZeroLeft. */
  public VelocityZero(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_climber.runClimberMotorLeft(-.1);
    m_climber.runClimberMotorRight(-.1);
    System.out.println(" Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.runClimberMotorLeft(-.1);
    m_climber.runClimberMotorRight(-.1);
    System.out.println("Exectute");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.runClimberMotorLeft(0);
    m_climber.setLeftEncoderPos(0.);
    m_climber.runClimberMotorRight(0);
    m_climber.setRightEncoderPos(0.);
    m_timer.stop();
System.out.println("End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("is Finished" + m_climber.leftCurrent());
    return m_timer.get() >= .2 && Math.abs(m_climber.climberMotorRightVelocity()) < 30 && Math.abs(m_climber.climberMotorLeftVelocity()) < 30;
   
  }

}
