// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final SparkMax climber;
  private final SparkMaxConfig climberConfig;

  public Climber() {

    climber = new SparkMax(21, MotorType.kBrushless);
    climberConfig = new SparkMaxConfig();
    climberConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
  }

  public void setVoltageClimbMotor(double volts) {
    volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
    volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
