// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  CANSparkMax climberMotorRight;
  CANSparkMax climberMotorLeft;
  Solenoid grippySolenoid;
  RelativeEncoder climberMotorRightEncoder;
  RelativeEncoder climberMotorLeftEncoder;
  SparkMaxPIDController climberPIDRight;
  SparkMaxPIDController climberPIDLeft;
  static Climber instance;

  // values 140 and 2
  public Climber() {
    climberMotorRight = new CANSparkMax(21, MotorType.kBrushless);
    climberMotorLeft = new CANSparkMax(20, MotorType.kBrushless);
    grippySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    climberMotorLeftEncoder = climberMotorLeft.getEncoder();
    climberMotorRightEncoder = climberMotorRight.getEncoder();
    climberMotorLeftEncoder.setPosition(0);
    climberMotorRightEncoder.setPosition(0);
    climberPIDRight = climberMotorRight.getPIDController();
    climberPIDLeft = climberMotorLeft.getPIDController();
    climberPIDLeft.setP(.01);
    climberPIDRight.setP(.01);
    climberPIDLeft.setFeedbackDevice(climberMotorLeftEncoder);
    climberPIDRight.setFeedbackDevice(climberMotorRightEncoder);
    climberMotorLeft.setSmartCurrentLimit(40);
    climberMotorRight.setSmartCurrentLimit(40);
    climberMotorLeft.setIdleMode(IdleMode.kBrake);
    climberMotorRight.setIdleMode(IdleMode.kBrake);
  }

  private void runGrippySolenoid() {
    grippySolenoid.toggle();
  }

  private void setClimberMotorPosition(double position) {
    climberPIDLeft.setReference(position, ControlType.kPosition);
    climberPIDRight.setReference(position, ControlType.kPosition);
  }

  public Command setClimberMotorPositionCommand(double position) {
    return run(() -> setClimberMotorPosition(position));
  }

  public Command runGrippySolenoidCommand() {
    return runOnce(() -> runGrippySolenoid());
  }

  private void runClimberMotors(double speed) {
    climberMotorLeft.set(speed);
    climberMotorRight.set(speed);
  }

  public void runClimberMotorRight(double speed) {
    climberMotorRight.set(speed);
  }

  public void runClimberMotorLeft(double speed) {
    climberMotorLeft.set(speed);
  }

  public Command runClimberMotorRightCommand(double speed) {
    return run(() -> runClimberMotorRight(speed));
  }

  public Command runClimberMotorLeftCommand(double speed) {
    return run(() -> runClimberMotorLeft(speed));
  }

  public Command runClimberMotorsCommand(double speed) {
    return run(() -> runClimberMotors(speed));
  }

  public Command stopClimberMotorsCommand() {
    return run(() -> runClimberMotors(0));
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public double rightCurrent() {
    return climberMotorLeft.getOutputCurrent();
  }

  public double leftCurrent() {
    return climberMotorRight.getOutputCurrent();
  }

  public double getLeftEncoderValue() {
    return climberMotorLeftEncoder.getPosition();
  }

  public double getRightEncoderValue() {
    return climberMotorRightEncoder.getPosition();
  }

  public void setLeftEncoderPos(double pos) {
    climberMotorLeftEncoder.setPosition(pos);
  }

  public void setRightEncoderPos(double pos) {
    climberMotorRightEncoder.setPosition(pos);
  }  
public double climberMotorRightVelocity(){
  return climberMotorRightEncoder.getVelocity();
}
public double climberMotorLeftVelocity(){
  return climberMotorLeftEncoder.getVelocity();
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Motor Left Postition", climberMotorLeftEncoder.getPosition());
    SmartDashboard.putNumber("Climber Motor Right Postition", climberMotorRightEncoder.getPosition());
    SmartDashboard.putNumber("Left Motor Current", climberMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Motor Current", climberMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Climber Left Motor Velocity", climberMotorLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Climber Motor Right Velocity", climberMotorRightEncoder.getVelocity());
    // This method will be called once per scheduler run
  }
}
