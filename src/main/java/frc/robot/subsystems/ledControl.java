package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;

public class ledControl extends SubsystemBase {

  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kPurpleTarget = new Color(0.54, 0.17, 0.89);
  private final Color kYellowTarget = new Color(1, 1, 0);

  private Color[] purpleWhiteArray = {Color.kPurple, Color.kWhiteSmoke};
  private TrobotAddressableLEDPattern m_purpleChasePattern = new ChasePattern(purpleWhiteArray, 3);

  private Color[] yellowWhiteArray = {Color.kYellow, Color.kWhiteSmoke};
  private TrobotAddressableLEDPattern m_yellowChasePattern = new ChasePattern(yellowWhiteArray, 3);

  private Color[] greenRedArray = {Color.kGreen, Color.kRed};
  private TrobotAddressableLEDPattern m_redChasePattern = new ChasePattern(greenRedArray, 3);

  private Color[] greenBlueArray = {Color.kGreen, Color.kBlue};
  private TrobotAddressableLEDPattern m_blueChasePattern = new ChasePattern(greenBlueArray, 3);

  private TrobotAddressableLED m_led;

  private int ledState;

  private boolean red_alliance;

  public ledControl(TrobotAddressableLED m_led_strip, boolean red_alliance) {
    m_led = m_led_strip;
    ledState = 0;
    colorMatcher.addColorMatch(kPurpleTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    this.red_alliance = red_alliance;
  }

  public void periodic() {
    if (ledState == 1) {
      m_led.setPattern(m_purpleChasePattern);
    } else if (ledState == 2) {
      m_led.setPattern(m_yellowChasePattern);
    } else {
      m_led.setPattern(red_alliance ? m_redChasePattern : m_blueChasePattern);
    }
  }

  public void setLED2Blank() {
    ledState = 0;
  }

  public void setLED2Cube() {
    ledState = 1;
  }

  public void setLED2Cone() {
    ledState = 2;
  }

  public int getLEDstate() {
    return ledState;
  }

  public void switchLED() {
    if (ledState == 1) ledState = 2;
    else ledState = 1;
  }
}
