package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

    private final AddressableLED led = new AddressableLED(1);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(1);

    private final static LEDs INSTANCE = new LEDs();
    @SuppressWarnings("WeakerAccess")
    public static LEDs getInstance() {
        return INSTANCE;
    }
    private LEDs() {}

    public void setColor(int[] colors){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            ledBuffer.setRGB(i, colors[0], colors[1], colors[2]);
        }
        led.setData(ledBuffer);
    }
}

