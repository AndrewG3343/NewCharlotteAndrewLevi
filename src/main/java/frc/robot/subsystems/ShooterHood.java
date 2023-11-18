// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
  /** Creates a new ShooterHood. */
  CANSparkMax shooterHoodMotor;
  static ShooterHood instance;
  RelativeEncoder shooterHoodEncoder;
  SparkMaxPIDController hoodPID;

  public ShooterHood() {
    shooterHoodMotor = new CANSparkMax(19, MotorType.kBrushless);
    shooterHoodMotor.setInverted(true);
    shooterHoodEncoder = shooterHoodMotor.getEncoder();
    shooterHoodEncoder.setPosition(0);
    hoodPID = shooterHoodMotor.getPIDController();
    hoodPID.setP(.1);
    hoodPID.setFeedbackDevice(shooterHoodEncoder);
    
  }

  private void runShooterHoodMotor(double speed) {
    shooterHoodMotor.set(speed);
  }

  public Command runShooterHoodMotorCommand(double speed) {
    return run(() -> runShooterHoodMotor(speed));
  }

  public Command stopShooterHoodMotorCommand() {
    return run(() -> runShooterHoodMotor(0));
  }

  private void setShooterHoodPosition(double position) {
    hoodPID.setReference(position, ControlType.kPosition);
  }
  public Command setShooterHoodPositionCommand(double position) {
    return run(() -> setShooterHoodPosition(position));
  }

  public static ShooterHood getinstance() {
    if (instance == null) {
      instance = new ShooterHood();
    }
    return instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Hood Postition", shooterHoodEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
