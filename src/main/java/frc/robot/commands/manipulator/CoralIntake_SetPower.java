// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Manipulators;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
*/
public class CoralIntake_SetPower extends Command {
    /** Creates a new Intake_SetPower. */
    double speed;

    Manipulators manipulators = Robot.manipulators;

    public CoralIntake_SetPower(double speedlocal) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(manipulators);
        speed = speedlocal;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        manipulators.setVoltageToCoralWheelMotor(speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        manipulators.stopCoralWheelMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
