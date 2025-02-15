
package frc.robot.subsystems;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final SparkMax elevatorFollowingMotor;
    private final SparkMaxConfig elevatorEncoderConfig;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    public final DigitalInput ElevatorLimitSwitch;

    public Elevator() {// TODO: Change deviceId to canspark ids or something idk im new
        elevatorMotor = new SparkMax(16, MotorType.kBrushless);
        elevatorFollowingMotor = new SparkMax(17, MotorType.kBrushless);
        elevatorEncoderConfig = new SparkMaxConfig();
        ElevatorLimitSwitch = new DigitalInput(3);

        // follows other elevator motor
        elevatorEncoderConfig.follow(17);

        // this no longer works with new sparkmax code
        // elevatorMotor.restoreFactoryDefaults();
        // coralWheelMotor.restoreFactoryDefaults();

        // this code inverts motor, may or may not be used later(Untested)
        elevatorEncoderConfig.inverted(Constants.elevatorMotorReversed);
        // elevatorMotor.setInverted(Constants.elevatorMotorReversed);

        // Setting idle mode to break when not in use (Untested)
        motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        elevatorMotor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
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
        if (!ElevatorLimitSwitch.get()) {
            elevatorMotor.setVoltage(volts);
        } else {
            stopElevatorMotor();
        }
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

}
