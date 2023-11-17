// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
  TalonSRX infeedMotor;
  CANSparkMax singulatorMotor;
  Solenoid solenoid;
  static Infeed instance;

  /** Creates a new Infeed. */
  public Infeed() {
    infeedMotor = new TalonSRX(13);
    singulatorMotor = new CANSparkMax(14, MotorType.kBrushless);
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    infeedMotor.setInverted(true);
    singulatorMotor.setInverted(false);
  }

  private void runInfeed(double speed) {
    infeedMotor.set(ControlMode.PercentOutput, speed);
    singulatorMotor.set(speed);
  }

  public Command runInfeedCommand(double speed) {
    return run(() -> runInfeed(speed));
  }

  public Command stopInfeedCommand() {
    return runOnce(() -> runInfeed(0.));
  }

  private void setInfeedDown(boolean isDown) {
    solenoid.set(isDown);
  }

  public boolean getInfeedDown() {
    return solenoid.get();
  }

  public Command setInfeedDownCommand(boolean isDown) {
    return runOnce(() -> {
      setInfeedDown(isDown == getInfeedDown() ? !isDown : isDown); // TODO: This is stupid bad crap fix plz but no work
                                                                   // :(
      System.out.println("RanSetInfeed, CurrentVal: " + getInfeedDown() + ", setting to " + isDown);
    });
  }

  public static Infeed getInstance() {
    if (instance == null) {
      instance = new Infeed();
    }
    return instance;
  }

  @Override
  public void periodic() {
  }
}
