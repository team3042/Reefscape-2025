// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulators;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Wrist_PIDSetPos extends Command {
  /** Creates a new Wrist_PIDSetPos. */

  private double goalPosition;
  private double kP = Constants.ManipulatorConstants.wristkP;
  private double kI = Constants.ManipulatorConstants.wristkI;
  private double kD = Constants.ManipulatorConstants.wristkD;
  public double powerToApply;
  private double currentPos;

  Manipulators manipulators = Robot.manipulators;

  private final TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private ArmFeedforward armFeedforward = Constants.ManipulatorConstants.armFeedforward;

  private PIDController pid = new PIDController(kP, kI, kD);

  public Wrist_PIDSetPos(double goalPositionLocal) {

    addRequirements(Robot.manipulators);
    goalPosition = goalPositionLocal;
    pid.setSetpoint(goalPosition);
    constraints = new TrapezoidProfile.Constraints(Constants.ManipulatorConstants.wristMaxVelocity,
        Constants.ManipulatorConstants.wristMaxAcceleration);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPos = (double) (manipulators.getWristRotationMotorPosition());
    powerToApply = pid.calculate(currentPos);
    manipulators.setPowerToWristRotationMotor(powerToApply);

    SmartDashboard.putNumber("Current Power To Wrist", powerToApply);
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
