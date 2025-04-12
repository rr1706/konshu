package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase {
  boolean m_LEDScore = false;

  // LED colors are actually GRB, not RGB
  Color m_black = new Color(0, 0, 0);
  Color m_green = new Color(255, 0, 0); // ALGAE - green
  Color m_blue = new Color(0, 0, 255);  // CORAL - blue
  Color m_yellow = new Color(216, 255, 30);  // Coral in close - yellow
  Color m_purple = new Color(0, 100, 100); // DEFAULT - not holding anything

  private final BooleanSupplier m_hasCoral;
  private final BooleanSupplier m_hasAlgae;

  // PWM port 9 Must be a PWM header, not MXP or DIO
  AddressableLED m_led;

  // Reuse buffers
  // Length is expensive to set, so only set it once, then just update data
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(96);
  AddressableLEDBufferView leftLEDs = m_ledBuffer.createView(0, 47);
  AddressableLEDBufferView rightLEDs = m_ledBuffer.createView(48, 95);

  public LED(BooleanSupplier hasCoral, BooleanSupplier hasAlgae) {
    m_hasCoral = hasCoral;
    m_hasAlgae = hasAlgae;
    m_led = new AddressableLED(9);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void periodic() {
    // Sets all to blue with coral, green with algae, or white if neither
    if (m_hasCoral.getAsBoolean()) {
      if (m_LEDScore) {
        leftStrand(m_yellow);
        rightStrand(m_yellow);    // Not currently used as left/right LEDs are wired in parallel
      } else {
        leftStrand(m_blue);
        rightStrand(m_blue);
      }
    } else if (m_hasAlgae.getAsBoolean()) {
      leftStrand(m_green);
      // rightStrand(m_green);
    } else {
      leftStrand(m_purple);
      // rightStrand(m_purple);
    }
    m_led.setData(m_ledBuffer); // Send our output to the LED strips
  }

  // Used to change LEDs based on distance to reef when holding coral
  public void setLEDScore(boolean LEDScore) {
    m_LEDScore = LEDScore;
  }

  // left strand LED
  public void leftStrand(Color ledColor) {
    LEDPattern left;

    left = LEDPattern.solid(ledColor);

    // Apply the LED pattern to the data buffer
    left.applyTo(leftLEDs);
  }

  public void rightStrand(Color ledColor) {
    LEDPattern right;

    right = LEDPattern.solid(ledColor);

    // Apply the LED pattern to the data buffer
    right.applyTo(rightLEDs);
  }
}
