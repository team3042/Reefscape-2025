// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.subsystems.Climber;

// public class ClimberManualPower extends Command {

// private Double manualPowerLocal;

// Climber climber = Robot.climber;

// public ClimberManualPower(Double manualPower) {
// addRequirements(climber);
// manualPowerLocal = manualPower;
// }

// // Called when the command is initially scheduled.
// @Override
// public void initialize() {
// }

// // Called every time the scheduler runs while the command is scheduled.
// @Override
// public void execute() {
// climber.setVoltageClimbMotor(manualPowerLocal);

// }

// // Called once the command ends or is interrupted.
// @Override
// public void end(boolean interrupted) {
// climber.setVoltageClimbMotor(0);
// }

// // Returns true when the command should end.
// @Override
// public boolean isFinished() {
// return false;
// }
// }
