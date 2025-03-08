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
  Color m_black = new Color (0,0,0); // used for FLASHING - actually in grb
  Color m_green = new Color(255, 0, 0); // ALGAE - green actually in grb
  Color m_pink = new Color (105, 255, 180); // CORAL - pink actually in grb
  Color m_white = new Color (255,255,255); // DEFAULT not holding anything - white actually in grb

  final boolean m_isSolid = true; // !is_solid for flashing
  private final CoralArm coralArm;

  // PWM port 9 Must be a PWM header, not MXP or DIO
  AddressableLED m_led;

  // Reuse buffers
  // Default to a length of 120, start empty output
  // Length is expensive to set, so only set it once, then just update data
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(96);
  AddressableLEDBufferView leftLEDs = m_ledBuffer.createView(0, 47);
  AddressableLEDBufferView rightLEDs = m_ledBuffer.createView(48, 95);

  public LED(CoralArm coralarm) {
    this.coralArm = coralarm;
    m_led = new AddressableLED(9);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
    defaultLEDs();
  }

  // default state - holding nothing - white solid not flashing
  public void defaultLEDs() {
    // check if holdCoral() and holdAlgae() are false then show default LEDs
    if (!holdCoral() && !holdAlgae()) {
        leftStrand(m_white, m_isSolid);
        rightStrand(m_white, m_isSolid);
    }
  }

  // holding coral - pink solid flashing 
  public boolean holdCoral() {
    boolean isHoldingCoral = coralArm.haveCoral();

    if (isHoldingCoral) {
        // set LEDs to white flashing
        leftStrand(m_pink, !m_isSolid);
        rightStrand(m_pink, !m_isSolid);
    }

    return isHoldingCoral;
  }

  // holding algae - green solid flashing
  public boolean holdAlgae() {

    //TODO
    boolean isHoldingAlgae = false; // update this with some method of finding hold status - Boolean

    if (isHoldingAlgae) {     
        // set LEDs to green flashing
        leftStrand(m_green, !m_isSolid);
        rightStrand(m_green, !m_isSolid);
    }

    return isHoldingAlgae;
  }

  // left strand LED - lights up when any of the states are in effect
  public void leftStrand(Color ledColor, boolean ledPattern) {
    // LED pattern and color
    // Create an LED pattern that sets the entire strip to solid or flashing color
    LEDPattern left;

    if (ledPattern == m_isSolid) left = LEDPattern.solid(ledColor);
    else left = LEDPattern.gradient(GradientType.kDiscontinuous, ledColor, m_black);
    
    // Apply the LED pattern to the data buffer
    left.applyTo(leftLEDs);
  }

  // right strand LED - lights up when any of the states are in effect
  public void rightStrand(Color ledColor, boolean ledPattern) {

    // LED pattern and color
    // Create an LED pattern that sets the entire strip to solid or flashing color
    LEDPattern right;

    if (ledPattern == m_isSolid) right = LEDPattern.solid(ledColor);
    else right = LEDPattern.gradient(GradientType.kDiscontinuous, ledColor, m_black);
    
    // Apply the LED pattern to the data buffer
    right.applyTo(rightLEDs);
  }

    public void periodic() {

        defaultLEDs();

        // if (ReefTargetCalculator.left.getAsBoolean() == true) {
        //     leftStrand(m_green, m_isSolid); }
        // else {
        //     leftStrand(m_pink, m_isSolid);
        // }
        // if (ReefTargetCalculator.right.getAsBoolean() == true) {
        //     rightStrand(m_green, m_isSolid); }
        // else {
        //     rightStrand(m_pink, m_isSolid);
        // }

        m_led.setData(m_ledBuffer);   // Send our output to the LED strips
    }

}