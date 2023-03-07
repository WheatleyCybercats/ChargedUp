package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {

    AHRS ahrs;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this NaVX. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static NavX INSTANCE = new NavX();

    /**
     * Returns the Singleton instance of this NaVX. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code NaVX.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static NavX getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this NaVX. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public NavX() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        ahrs = new AHRS();
    }

    public double getPitch(){
        return ahrs.getPitch();
    }

    public double getRoll(){
        return ahrs.getRoll();
    }

    public double getYaw(){
        return (ahrs.getYaw()+360)%360;
    }
}