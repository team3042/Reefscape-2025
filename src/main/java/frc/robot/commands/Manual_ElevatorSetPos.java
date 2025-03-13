// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.ElevatorManualPower;
import frc.robot.commands.manipulator.Wrist_PIDSetPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Manual_ElevatorSetPos extends ParallelCommandGroup {
  /** Creates a new Manual_SetPos. */
  public Manual_ElevatorSetPos(int manpow) {

    ElevatorManualPower elevatorManualPower = new ElevatorManualPower(manpow);
    Wrist_PIDSetPos wristSetPos = new Wrist_PIDSetPos(Robot.manipulators.getWristRotationMotorPosition());

    addCommands(elevatorManualPower, wristSetPos);
  }
}
