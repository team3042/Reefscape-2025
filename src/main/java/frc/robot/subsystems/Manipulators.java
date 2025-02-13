
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

public class Manipulators extends SubsystemBase {

    private final SparkMax wristRotationMotor;
    private final SparkMax coralWheelMotor;
    private final SparkMax algaeWheelMotor;
    private final AbsoluteEncoderConfig wristRotationEncoderConfig;
    private final AbsoluteEncoderConfig coralWheelEncoderConfig;
    private final AbsoluteEncoderConfig algaeWheelEncoderConfig;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    public final DigitalInput WristRotationLimitSwitch;
    public final DigitalInput CoralWheelLimitSwitch;
    public final DigitalInput AlgaeWheelLimitSwitch;

    public Manipulators() {// TODO: Change deviceId to canspark ids or something idk im new
        wristRotationMotor = new SparkMax(0, MotorType.kBrushless);
        coralWheelMotor = new SparkMax(0, MotorType.kBrushless);
        algaeWheelMotor = new SparkMax(0, MotorType.kBrushless);

        wristRotationEncoderConfig = new AbsoluteEncoderConfig();
        coralWheelEncoderConfig = new AbsoluteEncoderConfig();
        algaeWheelEncoderConfig = new AbsoluteEncoderConfig();

        WristRotationLimitSwitch = new DigitalInput(3);
        CoralWheelLimitSwitch = new DigitalInput(4);
        AlgaeWheelLimitSwitch = new DigitalInput(4);

        // this no longer works with new sparkmax code
        // elevatorMotor.restoreFactoryDefaults();
        // coralWheelMotor.restoreFactoryDefaults();

        // this code inverts motor, may or may not be used later(Untested)
        wristRotationEncoderConfig.inverted(Constants.wristRotationMotorReversed);
        coralWheelEncoderConfig.inverted(Constants.coralWheelMotorReversed);
        algaeWheelEncoderConfig.inverted(Constants.algaeWheelMotorReversed);
        wristRotationMotor.setInverted(Constants.wristRotationMotorReversed);
        coralWheelMotor.setInverted(Constants.coralWheelMotorReversed);
        algaeWheelMotor.setInverted(Constants.algaeWheelMotorReversed);

        // Setting idle mode to break when not in use (Untested)
        motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        wristRotationMotor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        coralWheelMotor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        algaeWheelMotor.configure(motorConfig, SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

    }

    // Methods for setting power to the motors
    public void setPowerToWristRotationMotor(double percentPower) {
        if (WristRotationLimitSwitch.get() || (!WristRotationLimitSwitch.get() && percentPower >= 0)) {
            wristRotationMotor.set(percentPower);
        } else {
            stopWristRotationMotor();
        }
    }

    public void setPowertoCoralWheelMotor(double percentPower) {
        if (CoralWheelLimitSwitch.get() || (!CoralWheelLimitSwitch.get() && percentPower >= 0)) {
            coralWheelMotor.set(percentPower);
        } else {
            stopCoralWheelMotor();
        }
    }

    public void setPowertoAlgaeWheelMotor(double percentPower) {
        if (AlgaeWheelLimitSwitch.get() || (!AlgaeWheelLimitSwitch.get() && percentPower >= 0)) {
            algaeWheelMotor.set(percentPower);
        } else {
            stopAlgaeWheelMotor();
        }
    }

    // Methods for setting voltage to the motors
    public void setVoltageWristRotationMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

        if (WristRotationLimitSwitch.get() || (!WristRotationLimitSwitch.get() && volts >= 0)) {
            wristRotationMotor.setVoltage(volts);
        } else {
            stopWristRotationMotor();
        }
    }

    public void setVoltageCoralWheelMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts
        // sets voltage for coral wheel motor
        if (CoralWheelLimitSwitch.get() || (!CoralWheelLimitSwitch.get() && volts >= 0)) {
            coralWheelMotor.setVoltage(volts);
        } else {
            stopCoralWheelMotor();
        }
    }

    public void setVoltageAlgaeWheelMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts
        // sets voltage for coral wheel motor
        if (AlgaeWheelLimitSwitch.get() || (!AlgaeWheelLimitSwitch.get() && volts >= 0)) {
            algaeWheelMotor.setVoltage(volts);
        } else {
            stopAlgaeWheelMotor();
        }
    }

    // Methods for stopping the motors
    public void stopWristRotationMotor() {
        setPowerToWristRotationMotor(0);
    }

    public void stopCoralWheelMotor() {
        setPowertoCoralWheelMotor(0);
    }

    public void stopAlgaeWheelMotor() {
        setPowertoAlgaeWheelMotor(0);
    }

    // Encoder methods for getting the motor position
    public double getWristRotationMotorPosition() {
        return wristRotationMotor.getEncoder().getPosition();
    }

    public double getCoralWheelMotorPosition() {
        return coralWheelMotor.getEncoder().getPosition();
    }

    public double getAlgaeWheelMotorPosition() {
        return algaeWheelMotor.getEncoder().getPosition();
    }

    // Encoder methods for getting the motor velocity
    public double getWristRotationMotorVelocity() {
        return wristRotationMotor.getEncoder().getVelocity();
    }

    public double getCoralWheelMotorVelocity() {
        return coralWheelMotor.getEncoder().getVelocity();
    }

    public double getAlgaeWheelMotorVelocity() {
        return algaeWheelMotor.getEncoder().getVelocity();
    }

    // Reset all encoders to 0
    public void resetEncoders() {
        wristRotationMotor.getEncoder().setPosition(0);
        coralWheelMotor.getEncoder().setPosition(0);
        algaeWheelMotor.getEncoder().setPosition(0);
    }

    // Reset the wrist rotation encoder to 0
    public void resetWristRotationEncoder() {
        wristRotationMotor.getEncoder().setPosition(0);
    }

    // Reset the coral wheel encoder to 0
    public void resetCoralWheelEncoder() {
        coralWheelMotor.getEncoder().setPosition(0);
    }// Reset the coral wheel encoder to 0

    public void resetAlgaeWheelEncoder() {
        algaeWheelMotor.getEncoder().setPosition(0);
    }

}
