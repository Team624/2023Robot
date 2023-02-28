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
  private TrobotAddressableLEDPattern m_purpleWhiteChasePattern =
      new ChasePattern(purpleWhiteArray, 3);

  private Color[] yellowWhiteArray = {Color.kYellow, Color.kWhiteSmoke};
  private TrobotAddressableLEDPattern m_yellowWhiteChasePattern =
      new ChasePattern(yellowWhiteArray, 3);

  private Color[] purpleArray = {Color.kPurple};
  private TrobotAddressableLEDPattern m_purpleChasePattern = new ChasePattern(purpleArray, 3);

  private Color[] yellowArray = {Color.kYellow};
  private TrobotAddressableLEDPattern m_yellowChasePattern = new ChasePattern(yellowArray, 3);

  private TrobotAddressableLED m_led;

  private boolean cone = false;
  private boolean double_substation = false;

  public ledControl(TrobotAddressableLED m_led_strip) {
    m_led = m_led_strip;
    colorMatcher.addColorMatch(kPurpleTarget);
    colorMatcher.addColorMatch(kYellowTarget);
  }

  public void periodic() {
    if (!cone && double_substation) {
      m_led.setPattern(m_purpleChasePattern);
    } else if (!cone && !double_substation) {
      m_led.setPattern(m_purpleWhiteChasePattern);
    } else if (cone && double_substation) {
      m_led.setPattern(m_yellowChasePattern);
    } else {
      m_led.setPattern(m_yellowWhiteChasePattern);
    }
  }

  public void updateCargo() {
    cone = !cone;
  }

  public void updateStation() {
    double_substation = !double_substation;
  }
}
