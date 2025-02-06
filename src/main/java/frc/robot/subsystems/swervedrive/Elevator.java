
// package frc.robot.subsystems.swervedrive;

// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {
    
//     private final CANSparkMax heightMotor;
//     private final CANSparkMax coralWheelMotor;

//     public final DigitalInput ExtensionLimitSwitch;
//     public final DigitalInput RotationLimitSwitch;


//     public Elevator() {
//         heightMotor = new CANSparkMax(Constants.kHeightMotorPort, MotorType.kBrushless);
//         coralWheelMotor = new CANSparkMax(Constants.kCoralWheelMotor, MotorType.kBrushless);

//         ExtensionLimitSwitch = new DigitalInput(3);
//         RotationLimitSwitch = new DigitalInput(4);

//         heightMotor.restoreFactoryDefaults();
//         coralWheelMotor.restoreFactoryDefaults();

//         heightMotor.setInverted(Constants.heightMotorReversed);
//         coralWheelMotor.setInverted(Constants.coralWheelMotorReversed);

//         heightMotor.setIdleMode(IdleMode.kBrake);
//         coralWheelMotor.setIdleMode(IdleMode.kBrake);
        
//         heightMotor.getEncoder().setPositionConversionFactor(42);
//         coralWheelMotor.getEncoder().setPositionConversionFactor(42);

//     }

//     public void setPowerToHeight(double percentPower){
//         if(RotationLimitSwitch.get() ||(!RotationLimitSwitch.get() && percentPower >= 0)){
//             rotationMotor.set(percentPower);
//         } else {
//             stopRotationMotor();
//         }
//     }
//      public void setPowertoExtend(double percentPower) {
//     if (ExtensionLimitSwitch.get() || (!ExtensionLimitSwitch.get() && percentPower >= 0)){
//       extendMotor.set(percentPower);
//     } else {
//       stopExtendMotor();
//     }
//   }

//   public void setVoltageRotationMotor(double volts) {
//     volts = Math.max(volts, -12.0); 
//     volts = Math.min(volts, 12.0);

//     if (RotationLimitSwitch.get() || (!RotationLimitSwitch.get() && volts >= 0)){
//       rotationMotor.setVoltage(volts);
//     } else {
//       stopRotationMotor();
//     }
//   }
//   public void setVoltageExtendMotor(double volts) {
//     volts = Math.max(volts, -12.0);
//     volts = Math.min(volts, 12.0);

//     if (ExtensionLimitSwitch.get() || (!ExtensionLimitSwitch.get() && volts >= 0)){
//       extendMotor.setVoltage(volts);
//     } else{
//       stopExtendMotor();
//     }
//   }


    
// }
