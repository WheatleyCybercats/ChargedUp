package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class LemonLight extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this LemonLight. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static LemonLight LL = new LemonLight();
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    /**
     * Returns the Singleton instance of this LemonLight. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LemonLight.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LemonLight getInstance() {
        return LL;
    }

    /**
     * Creates a new instance of this LemonLight. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public LemonLight() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tv = table.getEntry("tv");
    }
    public double[] readValues(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double v = tv.getDouble(0.0);
        double[] LemonValues = new double[]{x, y, area, v};
        smartDashboard(LemonValues);
        return LemonValues;
    }
    private void smartDashboard(double[] values){
        SmartDashboard.putNumber("LemonlightX", values[0]);
        SmartDashboard.putNumber("LemonlightY", values[1]);
        SmartDashboard.putNumber("LemonlightArea", values[2]);
        SmartDashboard.putNumber("LemonlightV", values[3]);
    }
}

