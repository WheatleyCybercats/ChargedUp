package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,1);
    private final static Claw INSTANCE = new Claw();
    @SuppressWarnings("WeakerAccess")
    public static Claw getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this Claw. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private Claw() {
       claw.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw(){
        claw.set(DoubleSolenoid.Value.kReverse);
    }

    public void openClaw(){
        claw.set(DoubleSolenoid.Value.kForward);
    }
}

