
package frc.robot.subsystems;

import frc.robot.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends Subsystems {

    private final CANSparkMax coralMotor;
    private final CANSparkMax liftMotor;

    public final DigitalInput CoralLimitSwitch;
    public final DigitalInput LiftLimitSwitch;


    public Elevator() {
        coralMotor = new CANSparkMax(Constants.kCoralMotorPort, MotorType.kBrushless);
        liftMotor = new CANSparkMax(Constants.kLiftMotorPort, MotorType.kBrushless);

        CoralLimitSwitch = new DigitalInput(3);
        LiftLimitSwitch = new DigitalInput(4);
    
        // Configure Motor Settings
        
        coralMotor.restoreFactoryDefaults();
        liftMotor.restoreFactoryDefaults();

        coralMotor.setInverted(Constants.coralMotorReversed);
        liftMotor.setInverted(Constants.liftMotorReversed);

        coralMotor.setIdleMode(IdleMode.kBrake);
        liftMotor.setIdleMode(IdleMode.kBrake);

        coralMotor.getEncoder().setPositionConversionFactor(42);
        liftMotor.getEncoder().setPositionConversionFactor(42);
    }

    // Methods for setting power to the motors
    public void setPowerToCoral(double percentPower) {
        if (CoralLimitSwitch.get() || (!CoralLimitSwitch.get() && percentPower >= 0)) {
            coralMotor.set(percentPower);
        } else {
            stopCoralMotor();
        }
    }
    public void setPowerToLift(double percentPower) {
        if (LiftLimitSwitch.get() || (!LiftLimitSwitch.get() && percentPower >= 0)) {
            liftMotor.set(percentPower);
        } else {
            stopLiftMotor();
        }
    }

    // Methods for setting voltage to the motors
    public void setVoltageCoralMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

        if (CoralLimitSwitch.get() || (!CoralLimitSwitch.get() && volts >= 0)) {
            coralMotor.setVoltage(volts);
        } else {
            stopCoralMotor();
        }
    }
    public void setVoltageLiftMotor(double volts) {
        volts = Math.max(volts, -12.0); // Don't allow setting less than -12 volts
        volts = Math.min(volts, 12.0); // Don't allow setting more than 12 volts

        if (LiftLimitSwitch.get() || (!LiftLimitSwitch.get() && volts >= 0)) {
            liftotor.setVoltage(volts);
        } else {
            stopLiftMotor();
        }
    }

    // Methods for stopping the motors
    public void stopCoralMotor() {
        setPowerToCoral(0);
    }
    public void stopLiftMotor() {
        setPowerToLift(0);
    }

    // Encoder methods for getting the motor position
    public double getCoralMotorPosition() {
        return coralMotor.getEncoder().getPosition();
    }
    public double getLiftMotorPosition() {
        return liftMotor.getEncoder().getPosition();
    }
    // Encoder methods for getting the motor velocity
    public double getCoralMotorVelocity() {
        return coralMotor.getEncoder().getVelocity();
    }
    public double getLiftMotorVelocity() {
        return liftMotor.getEncoder().getVelocity();
    }

    // Reset both encoders to 0
    public void resetEncoders() {
        coralMotor.getEncoder().setPosition(0);
        liftMotor.getEncoder().getPosition(0);
    }

    // Reset the coral encoder to 0
    public void resetCoralEncoder() {
        coralMotor.getEncoder().setPosition(0);
    }
    // Reset the lift encoder to 0
    public void resetLiftEncoder() {
        liftMotor.getEncoder().setPosition(0);
    }

    // Returns the absolute height of the elevator (Draft)
    public void getElevatorHeight() {
        //double encoderCounts = getLiftMotorPosition();

        //return encoderCounts * Constants.????
    }

}
