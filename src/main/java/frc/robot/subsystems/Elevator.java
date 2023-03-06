package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    public CANSparkMax elevatorMotor1 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax elevatorMotor2 = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder1;
    private RelativeEncoder encoder2;
    private final static Elevator INSTANCE = new Elevator();

    /**
     * Returns the Singleton instance of this Elevator. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code Elevator.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Elevator getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Elevator. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public Elevator() {
        elevatorMotor1.setInverted(true);
        //encoder1 = elevatorMotor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        //encoder2 = elevatorMotor2.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder1 = elevatorMotor1.getEncoder();
        encoder2 = elevatorMotor2.getEncoder();
        elevatorMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elevatorMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        getEncoderValue();
        encoder1.setPosition(0);
        encoder2.setPosition(0);
    }

    public void setElevatorMotors(double elevatorSpeed) {
        elevatorMotor1.set(elevatorSpeed);
        elevatorMotor2.set(elevatorSpeed);
        SmartDashboard.putNumber("Elevator Encoder", getEncoderValue()[2]);
    }
    //TODO: Refractor the code so that all the places take the value from Constants so that the value stays the same
    public double[] getEncoderValue(){
        double[] encoderValues = {encoder1.getPosition(), encoder2.getPosition(), (encoder1.getPosition() + encoder2.getPosition())/2};
        return encoderValues;
    }
}

