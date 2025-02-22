// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.elevator.ElevatorSetPos;
import frc.robot.commands.manipulator.Wrist_SetPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score_SetPos extends ParallelCommandGroup {
  /** Creates a new Intake_SetPos. */
  public Score_SetPos(double elevatorgoalCounts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    ElevatorSetPos elevatorsetpos = new ElevatorSetPos(elevatorgoalCounts);
    Wrist_SetPos wristsetpos = new Wrist_SetPos(0.3, ManipulatorConstants.wristLowAngle);

    addCommands(elevatorsetpos, wristsetpos);
  }
}
