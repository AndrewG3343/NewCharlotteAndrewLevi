// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyer extends SubsystemBase {
  CANSparkMax conveyerMotor;
  static Conveyer instance;

  /** Creates a new Conveyer. */
  public Conveyer() {
    conveyerMotor = new CANSparkMax(15, MotorType.kBrushless);

  }

  private void runConveyer(double speed) {
    conveyerMotor.set(speed);
  }

  public Command runConveyerCommand(double speed) {
    return run(() -> runConveyer(speed));
  }

  public Command stopConveyerCommand() {
    return runOnce(() -> runConveyer(0));
  }

  public static Conveyer getInstance() {
    if (instance == null) {
      instance = new Conveyer();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
