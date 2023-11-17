// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  TalonFX shooterFrontMotor;
  TalonFX shooterBackMotor;
  TalonFX shooterKickerMotor;
  static Shooter instance;

  public Shooter() {
    shooterFrontMotor = new TalonFX(17);
    shooterBackMotor = new TalonFX(18);
    shooterKickerMotor = new TalonFX(16);
    shooterKickerMotor.setInverted(true);
  }

  private void runShooter(double speed) {
    shooterFrontMotor.set(ControlMode.PercentOutput, speed);
    shooterBackMotor.set(ControlMode.PercentOutput, speed);
    shooterKickerMotor.set(ControlMode.PercentOutput, speed);
  }

  public Command runShooterCommand(double speed) {
    return run(() -> runShooter(speed));
  }

  public Command stopShooterCommand() {
    return runOnce(() -> runShooter(0));
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  @Override
  public void periodic() {

  }
}
