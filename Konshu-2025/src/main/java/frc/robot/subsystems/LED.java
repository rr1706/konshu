package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase {
  double m_dist = 1.0;    // Vary LEDs based on dist from target (0.0 - 1.0)

  // LED colors are actually GRB, not RGB
  Color m_black = new Color(0, 0, 0);
  Color m_green = new Color(255, 0, 0); // ALGAE - green
  Color m_blue = new Color(0, 0, 255);  // CORAL - blue
  Color m_purple = new Color(0, 100, 100); // DEFAULT - not holding anything

  private final CoralArm coralArm;
  private final AlgaeArm algaeArm;

  // PWM port 9 Must be a PWM header, not MXP or DIO
  AddressableLED m_led;

  // Reuse buffers
  // Length is expensive to set, so only set it once, then just update data
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(96);
  AddressableLEDBufferView leftLEDs = m_ledBuffer.createView(0, 47);
  AddressableLEDBufferView rightLEDs = m_ledBuffer.createView(48, 95);

  public LED(CoralArm coralarm, AlgaeArm algaearm) {
    this.coralArm = coralarm;
    this.algaeArm = algaearm;
    m_led = new AddressableLED(9);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void periodic() {
    // Sets all to blue with coral, green with algae, or white if neither
    if (coralArm.haveCoral()) {
      leftStrand(m_blue, m_dist);
      // rightStrand(m_blue, m_dist); // Not currently used as left/right LEDs are wired in parallel
    } else if (algaeArm.haveAlgae()) {
      leftStrand(m_green, 1.0);
      // rightStrand(m_green, 1.0);
    } else {
      leftStrand(m_purple, 1.0);
      // rightStrand(m_purple, 1.0);
    }
    m_led.setData(m_ledBuffer); // Send our output to the LED strips
  }

  // Vary LEDs based on distance from target. A dist of 1.0 lights all LEDs and
  // 0.0 lights none.
  public void setDist(double dist) {
    m_dist = dist;
  }

  // left strand LED
  // A dist of 1.0 lights all LEDs (solid), 0.5 lights half the strand.
  public void leftStrand(Color ledColor, double dist) {
    LEDPattern left;

    // left = LEDPattern.solid(ledColor);
    left = LEDPattern.steps(Map.of((1.0 - m_dist), ledColor));

    // Apply the LED pattern to the data buffer
    left.applyTo(leftLEDs);
  }

  public void rightStrand(Color ledColor, double dist) {
    LEDPattern right;

    // right = LEDPattern.solid(ledColor);
    right = LEDPattern.steps(Map.of((1.0 - m_dist), ledColor));

    // Apply the LED pattern to the data buffer
    right.applyTo(rightLEDs);
  }
}
