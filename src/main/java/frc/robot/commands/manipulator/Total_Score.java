// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Score_SetPos;
import frc.robot.commands.elevator.ElevatorSetPos;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Total_Score extends SequentialCommandGroup {
  /** Creates a new Total_Score. */
  public Total_Score(double elevatorgoalCounts, double wristgoalCounts) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // ElevatorSetPos elevatorsetpos = new ElevatorSetPos(elevatorgoalCounts);
    CoralForTime coralfortime = new CoralForTime(-5, 1.0);
    // Wrist_SetPos wristsetpos = new Wrist_SetPos(wristgoalCounts);
    // ElevatorSetPos intakeElevatorSetPos = new
    // ElevatorSetPos(ElevatorConstants.intakeEncoderCounts);
    // Wrist_SetPos intakeWrist_SetPos = new
    // Wrist_SetPos(ManipulatorConstants.wristIntakeAngle);

    // ParallelCommandGroup both = new ParallelCommandGroup(coral, wristsetpos);

    addCommands(coralfortime);
  }
}
