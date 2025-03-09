package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ReefTargetCalculator;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.CoralArm;

public class LED extends SubsystemBase
{
  // LED colors are actually GRB, not RGB
  Color m_black = new Color (0,50,0); // used for FLASHING 
  Color m_green = new Color(255, 0, 0); // ALGAE - green 
  Color m_blue = new Color (0, 0, 255); // CORAL - blue 
  Color m_white = new Color (50,50,50); // DEFAULT not holding anything

  private final CoralArm coralArm;
  // private final AlgaeArm algaeArm;

  // PWM port 9 Must be a PWM header, not MXP or DIO
  AddressableLED m_led;

  // Reuse buffers
  // Default to a length of 120, start empty output
  // Length is expensive to set, so only set it once, then just update data
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(96);
  AddressableLEDBufferView leftLEDs = m_ledBuffer.createView(0, 47);
  AddressableLEDBufferView rightLEDs = m_ledBuffer.createView(48, 95);

  public LED(CoralArm coralarm) {
// publich LED(CoralArm coralarm, AlgaeArm algaearm) {
    this.coralArm = coralarm;
//   this.algaeArm = algaearm;
    m_led = new AddressableLED(9);
    m_led.setLength(m_ledBuffer.getLength());
    leftStrand(m_white, true);
    rightStrand(m_white, true);
    m_led.setData(m_ledBuffer); 
    m_led.start();
  }
  
  public void periodic() {
    // Sets all to blue with coral, green with algae, or white if neither
    if (coralArm.haveCoral()) {     // set LEDs to blue flashing
      leftStrand(m_blue, false);
      rightStrand(m_blue, false);
 // } else if (algaeArm.haveAlgae()) {
    } else if (false) {             // set LEDs to green flashing
      leftStrand(m_green, false);
      rightStrand(m_green, false);
    } else {
      leftStrand(m_white, true);
      rightStrand(m_white, true);
    }
    m_led.setData(m_ledBuffer);   // Send our output to the LED strips
  }

  // left strand LED - lights up when any of the states are in effect
  public void leftStrand(Color ledColor, boolean solid) {
    // LED pattern and color
    // Create an LED pattern that sets the entire strip to solid or flashing color
    LEDPattern left;

    // if (solid) left = LEDPattern.solid(ledColor);
    // else left = LEDPattern.gradient(GradientType.kDiscontinuous, ledColor, m_black);
    left = LEDPattern.solid(ledColor);

    // Apply the LED pattern to the data buffer
    left.applyTo(leftLEDs);
  }

  // right strand LED - lights up when any of the states are in effect
  public void rightStrand(Color ledColor, boolean solid) {

    // LED pattern and color
    // Create an LED pattern that sets the entire strip to solid or flashing color
    LEDPattern right;

    SmartDashboard.putBoolean("LED solid", solid);
    if (solid) right = LEDPattern.solid(ledColor);
    else right = LEDPattern.gradient(GradientType.kContinuous, ledColor, m_black, m_green, m_blue, m_white);

    // Apply the LED pattern to the data buffer
    right.applyTo(rightLEDs);
  }
}

 // TODO: How/when do we want to use this?  

        // if (ReefTargetCalculator.left.getAsBoolean() == true) {
        //     leftStrand(m_green, true); }
        // else {
        //     leftStrand(m_pink, true);
        // }
        // if (ReefTargetCalculator.right.getAsBoolean() == true) {
        //     rightStrand(m_green, true); }
        // else {
        //     rightStrand(m_pink, true);
        // }