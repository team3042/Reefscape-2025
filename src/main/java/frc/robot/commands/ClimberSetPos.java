// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberSetPos extends Command {
  /** Creates a new ClimberSetPos. */

  private double goalPos;
  private double distanceToGoal;
  private final int climberMOE = 30;
  private boolean reachedGoal = false;

  Climber climber = Robot.climber;

  public ClimberSetPos(double count) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    goalPos = count;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    distanceToGoal = Math.abs(goalPos - climber.getClimberPosition());

    if (distanceToGoal < climberMOE) {
      reachedGoal = true;
    }

    if (reachedGoal) {
      climber.stopClimberMotor();
    } else {
      if (goalPos > climber.getClimberPosition()) {
        climber.setVoltageClimbMotor(6);
      } else {
        climber.setVoltageClimbMotor(-6);
      }
    }

    SmartDashboard.putNumber("Climber Distance To Goal", distanceToGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimberMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
