
package frc.robot.subsystems;

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
    private final SparkMax coralWheelMotor;
    private final SparkMax rotationMotor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    public final DigitalInput ExtensionLimitSwitch;
    public final DigitalInput ElevatorLimitSwitch;
    public final DigitalInput CoralWheelLimitSwitch;
    public final DigitalInput RotationLimitSwitch;

    public Elevator() {// TODO: Change deviceId to canspark ids or something idk im new
        elevatorMotor = new SparkMax(0, MotorType.kBrushless);
        coralWheelMotor = new SparkMax(0, MotorType.kBrushless);
        rotationMotor = new SparkMax(0, MotorType.kBrushless);

        ExtensionLimitSwitch = new DigitalInput(3);
        RotationLimitSwitch = new DigitalInput(4);
        // this no longer works with new sparkmax code
        // elevatorMotor.restoreFactoryDefaults();
        // coralWheelMotor.restoreFactoryDefaults();

        // this code inverts motor, may or may not be used later(Untested)
        elevatorMotor.setInverted(Constants.elevatorMotorReversed);
        coralWheelMotor.setInverted(Constants.coralWheelMotorReversed);

        // Setting idle mode to break when not in use (Untested)
        motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        elevatorMotor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        coralWheelMotor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

    }

    public void setPowerToHeight(double percentPower) {
        if (RotationLimitSwitch.get() || (!RotationLimitSwitch.get() && percentPower >= 0)) {
            elevatorMotor.set(percentPower);
        } else {
            stopElevatorMotor();
        }
    }

    public void setPowertoCoralMotor(double percentPower) {
        if (ExtensionLimitSwitch.get() || (!ExtensionLimitSwitch.get() && percentPower >= 0)) {
            coralWheelMotor.set(percentPower);
        } else {
            stopCoralWheelMotor();
        }
    }

    public void setVoltageRotationMotor(double volts) {
        volts = Math.max(volts, -12.0);
        volts = Math.min(volts, 12.0);

        if (RotationLimitSwitch.get() || (!RotationLimitSwitch.get() && volts >= 0)) {
            rotationMotor.setVoltage(volts);
        } else {
            stopRotationMotor();
        }
    }

    /////////////////////////////

    // Methods for setting power to the motors
    public void setPowerToCoral(double percentPower) {
        if (CoralWheelLimitSwitch.get() || (!CoralWheelLimitSwitch.get() && percentPower >= 0)) {
            coralWheelMotor.set(percentPower);
        } else {
            stopCoralWheelMotor();
        }
    }

    public void setPowerToLift(double percentPower) {
        if (ElevatorLimitSwitch.get() || (!ElevatorLimitSwitch.get() && percentPower >= 0)) {
            elevatorMotor.set(percentPower);
        } else {
            stopElevatorMotor();
        }
    }

    public void setPowerToRotation(double percentPower) {
        if (RotationLimitSwitch.get() || (!RotationLimitSwitch.get() && percentPower >= 0)) {
            rotationMotor.set(percentPower);
        } else {
            stopRotationMotor();
        }
    }

    // Methods for setting voltage to the motors
    public void setVoltageCoralMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts
        // sets voltage for coral wheel motor
        if (CoralWheelLimitSwitch.get() || (!CoralWheelLimitSwitch.get() && volts >= 0)) {
            coralWheelMotor.setVoltage(volts);
        } else {
            stopCoralWheelMotor();
        }
    }

    public void setVoltageLiftMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

        if (ElevatorLimitSwitch.get() || (!ElevatorLimitSwitch.get() && volts >= 0)) {
            elevatorMotor.setVoltage(volts);
        } else {
            stopElevatorMotor();
        }
    }

    // Methods for stopping the motors
    public void stopCoralWheelMotor() {
        setPowerToCoral(0);
    }

    public void stopElevatorMotor() {
        setPowerToLift(0);
    }

    public void stopRotationMotor() {
        setPowerToRotation(0);
    }

    // Encoder methods for getting the motor position
    public double getCoralMotorPosition() {
        return coralWheelMotor.getEncoder().getPosition();
    }

    public double getelevatorMotorPosition() {
        return elevatorMotor.getEncoder().getPosition();
    }

    // Encoder methods for getting the motor velocity
    public double getCoralMotorVelocity() {
        return coralWheelMotor.getEncoder().getVelocity();
    }

    public double getelevatorMotorVelocity() {
        return elevatorMotor.getEncoder().getVelocity();
    }

    // Reset both encoders to 0
    public void resetEncoders() {
        coralWheelMotor.getEncoder().setPosition(0);
        elevatorMotor.getEncoder().setPosition(0);
    }

    // Reset the coral encoder to 0
    public void resetCoralEncoder() {
        coralWheelMotor.getEncoder().setPosition(0);
    }

    // Reset the lift encoder to 0
    public void resetLiftEncoder() {
        elevatorMotor.getEncoder().setPosition(0);
    }

    // Returns the absolute height of the elevator (Draft)
    // public double getElevatorHeight() {
    // double encoderCounts = getelevatorMotorPosition();
    // does this need to be the robot
    // height???///////////////////////////////////////////////////////////////////////////NOTICE
    // ME
    // return encoderCounts;
}
