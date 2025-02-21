
package frc.robot.subsystems;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotor;
    // private final SparkMax elevatorFollowingMotor;
    private final SparkMaxConfig elevatorEncoderConfig;
    public final DigitalInput ElevatorLimitSwitch;

    private ElevatorFeedforward elevatorFeedforward;
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxVelocity,
            ElevatorConstants.kMaxAcceleration);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kP,
            ElevatorConstants.kI, ElevatorConstants.kD, m_constraints, ElevatorConstants.kDt);
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kG,
            ElevatorConstants.kV);

    public Elevator() {
        elevatorMotor = new SparkMax(16, MotorType.kBrushless);
        // elevatorFollowingMotor = new SparkMax(17, MotorType.kBrushless);
        elevatorEncoderConfig = new SparkMaxConfig();
        ElevatorLimitSwitch = new DigitalInput(3);

        // this code inverts motor, may or may not be used later(Untested)
        elevatorEncoderConfig.inverted(Constants.elevatorMotorReversed);
        // elevatorMotor.setInverted(Constants.elevatorMotorReversed);

        // Setting idle mode to break when not in use (Untested)
        elevatorEncoderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        elevatorMotor.configure(elevatorEncoderConfig, SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

    }

    // Methods for setting power to the motors
    // public void setPowerToElevatorMotor(double percentPower) {
    // if (ElevatorLimitSwitch.get() || (!ElevatorLimitSwitch.get() && percentPower
    // >= 0)) {
    // elevatorMotor.set(percentPower);
    // } else {
    // stopElevatorMotor();
    // }
    // }

    // Methods for setting voltage to the motors
    public void setVoltageElevatorMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts
        // UNTESTED CODE!!!!!!!!!!!!!!!!!!!! WATCH
        // OUT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // if (!ElevatorLimitSwitch.get()) {
        // elevatorMotor.setVoltage(volts);
        // } else {
        // stopElevatorMotor();
        // }
        elevatorMotor.setVoltage(volts);
    }

    // Methods for stopping the motors
    public void stopElevatorMotor() {
        setVoltageElevatorMotor(0);
    }

    // Encoder methods for getting the motor position
    public double getElevatorMotorPosition() {
        return elevatorMotor.getEncoder().getPosition();
    }

    // Encoder methods for getting the motor velocity
    public double getElevatorMotorVelocity() {
        return elevatorMotor.getEncoder().getVelocity();
    }

    // Reset both encoders to 0
    public void resetEncoders() {
        elevatorMotor.getEncoder().setPosition(0);

    }

    // Reset the elevator encoder to 0
    public void resetElevatorEncoder() {
        elevatorMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {

    }

}
