// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetPos extends Command {
  private double goalPos;
  private double distanceToGoal;
  private final int marginOE = 5;
  private boolean reachedGoal = false;
  /** Creates a new ElevatorSetPosition. */

  Elevator elevator = Robot.elevator;

  public ElevatorSetPos(double count) {
    // Use addRequirements() here to declare subsystem dependencies.
    goalPos = count;
    addRequirements(elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // finds distance from current position to target position
    distanceToGoal = Math.abs(goalPos - elevator.getElevatorMotorPosition());

    if (distanceToGoal < marginOE) {
      reachedGoal = true;
    }

    if (reachedGoal) {
      elevator.stopElevatorMotor();
    } else {
      if (goalPos > elevator.getElevatorMotorPosition()) { // if the elevator is lower than the goal (it needs to go up)
        elevator.setVoltageElevatorMotor(3);
      } else {
        elevator.setVoltageElevatorMotor(-3);
      }
    }

    SmartDashboard.putNumber("Elevator ERROR", distanceToGoal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevatorMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;// this shouldn't end
  }
}
