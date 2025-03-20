
package frc.robot.subsystems;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final SparkMax algaeWheelMotor2;
    private final SparkMaxConfig wristRotationEncoderConfig;
    private final AbsoluteEncoderConfig coralWheelEncoderConfig;
    private final SparkMaxConfig algaeWheelEncoderConfig;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    public final DigitalInput wristRotationLimitSwitchDown;

    public Manipulators() {
        algaeWheelMotor = new SparkMax(17, MotorType.kBrushless);
        algaeWheelMotor2 = new SparkMax(18, MotorType.kBrushless);
        wristRotationMotor = new SparkMax(20, MotorType.kBrushless);
        coralWheelMotor = new SparkMax(19, MotorType.kBrushless);

        wristRotationEncoderConfig = new SparkMaxConfig();
        coralWheelEncoderConfig = new AbsoluteEncoderConfig();
        algaeWheelEncoderConfig = new SparkMaxConfig();

        wristRotationLimitSwitchDown = new DigitalInput(1);

        // this no longer works with new sparkmax code
        // elevatorMotor.restoreFactoryDefaults();
        // coralWheelMotor.restoreFactoryDefaults();

        // this code inverts motor, may or may not be used later(Untested)

        coralWheelEncoderConfig.inverted(Constants.coralWheelMotorReversed);
        algaeWheelEncoderConfig.inverted(Constants.algaeWheelMotorReversed);

        // Setting idle mode to break when not in use (Untested)
        motorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        wristRotationMotor.configure(motorConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        coralWheelMotor.configure(motorConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        algaeWheelMotor.configure(motorConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        algaeWheelMotor2.configure(motorConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);
        wristRotationEncoderConfig.inverted(Constants.wristRotationMotorReversed);
    }

    // Methods for setting power to the motors
    public void setPowerToWristRotationMotor(double percentPower) {
        SmartDashboard.putNumber("Wrist Power", percentPower);
        if (!wristRotationLimitSwitchDown.get() && percentPower > 0) {
            stopWristRotationMotor();
        } else {
            if (percentPower > 0 && getWristRotationMotorPosition() < -12) {
                stopWristRotationMotor();
            } else {
                wristRotationMotor.set(percentPower);
            }
        }
    }

    public void setPowertoCoralWheelMotor(double percentPower) {
        coralWheelMotor.set(percentPower);
    }

    public void setVoltageToCoralWheelMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0);
        coralWheelMotor.setVoltage(volts);
    }

    public void setPowertoAlgaeWheelMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0);
        algaeWheelMotor.setVoltage(volts);
        algaeWheelMotor2.setVoltage(-volts);
    }

    // Methods for stopping the motors
    public void stopWristRotationMotor() {
        wristRotationMotor.setVoltage(0);
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
    public void resetWristEncoders() {
        wristRotationMotor.getEncoder().setPosition(0);
    }

    // Reset the wrist rotation encoder to 0
    public void resetWristRotationEncoder() {
        wristRotationMotor.getEncoder().setPosition(0);
    }

    public boolean wristLimitSwitchClicked() {
        return wristRotationLimitSwitchDown.get();
    }

}
