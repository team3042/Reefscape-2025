// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulators;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Wrist_FFSetPos extends Command {
  /** Creates a new Wrist_PIDSetPos. */

  private double goalPosition;
  private double ticksToRadians = goalPosition / 165.5212;
  public double voltage;
  private double currentPos;

  Manipulators manipulators = Robot.manipulators;

  private ArmFeedforward armFeedforward = Constants.ManipulatorConstants.armFeedforward;

  public Wrist_FFSetPos(double goalPositionLocal) {

    addRequirements(Robot.manipulators);
    goalPosition = goalPositionLocal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    voltage = armFeedforward.calculate(goalPosition * ticksToRadians, 3);
    manipulators.setPowerToWristRotationMotor(voltage);
    currentPos = manipulators.getWristRotationMotorPosition();

    SmartDashboard.putNumber("Current Power To Wrist", voltage);
    SmartDashboard.putNumber("Current Wrist Position", currentPos);
    SmartDashboard.putNumber("Goal Wrist Position", goalPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
