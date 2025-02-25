// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulators;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Wrist_SetPos extends Command {
  /** Creates a new Wrist_SetPower. */

  private double speed;
  private double goalPositionLocal;
  private double currentPosition;
  private double distanceToGoal;
  private final int marginOE = 3;

  Manipulators manipulators = Robot.manipulators;

  public Wrist_SetPos(double goalPosition) {

    addRequirements(Robot.manipulators);
    speed = 0;
    goalPositionLocal = goalPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = manipulators.getWristRotationMotorPosition();
    distanceToGoal = Math.abs(goalPositionLocal - currentPosition);

    if (distanceToGoal < marginOE) {
      manipulators.stopWristRotationMotor();
    } else {
      if (goalPositionLocal > currentPosition) {
        manipulators.setPowerToWristRotationMotor(0.3);
      } else {
        manipulators.setPowerToWristRotationMotor(-0.3);
      }
    }

    SmartDashboard.putNumber("Wrist ERROR", distanceToGoal);
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
