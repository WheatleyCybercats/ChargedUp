package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public CANSparkMax armMotor = new CANSparkMax(Constants.armMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
    //public AbsoluteEncoder armEncoder;
    public RelativeEncoder armEncoder;

    private final static Arm INSTANCE = new Arm();

    /**
     * Returns the Singleton instance of this Arm. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code Arm.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Arm getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Arm. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public Arm() {
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.fromId(5));
        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);
    }
    public void setArmMotor(double armSpeed) {
        armMotor.set(armSpeed);
        SmartDashboard.putNumber("Arm Encoder", getEncoderValue());
    }
    public double getEncoderValue(){
        return armEncoder.getPosition();
    }
}


