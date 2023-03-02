package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;

public class Limelight extends SubsystemBase {

  private final NetworkTable networkTable;
  private final NetworkTableEntry botpose_network;
  private final NetworkTableEntry tid_network;
  private final NetworkTableEntry ta_network;
  private double ta;
  private double tl;
  private double x_coordinate;
  private double y_coordinate;
  private double angle;
  private double tid;

  private double[] botpose;
  private Map<Double, Double> id_json;

  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    ta_network = networkTable.getEntry("ta");
    botpose_network = networkTable.getEntry("botpose");
    tid_network = networkTable.getEntry("tid");
    id_json = new HashMap<Double, Double>();
    id_json.put(1.0, -6.94659);
    id_json.put(2.0, -5.27019);
    id_json.put(3.0, -3.59379);
    id_json.put(4.0, -1.26839);
    id_json.put(5.0, -1.26839);
    id_json.put(6.0, -3.59379);
    id_json.put(7.0, -5.2701);
    id_json.put(8.0, -6.94659);
  }

  public boolean hasTarget() {
    return tid >= 1 && tid <= 8;
  }

  public double getID() {
    return tid;
  }

  public double[] getAlignmentValues() {
    if (tid <= 8 && tid >= 1) {
      double distance = Math.abs(y_coordinate - id_json.get(tid));
      if (angle < 0) {
        double[] output = {-distance, -angle};
        return output;

      } else {
        double[] output = {distance, -angle};
        return output;
      }
    }
    double[] output = {};
    return output;
  }

  @Override
  public void periodic() {
    double[] botpose_data = botpose_network.getDoubleArray(new double[] {});
    botpose = botpose_data;
    if (botpose_data.length == 7) {
      x_coordinate = botpose_data[0] + 8.27;
      y_coordinate = botpose_data[1] - 4.01;
      tl = botpose[6];

    } else {
      x_coordinate = 0;
      y_coordinate = 0;
      tl = 0;
    }
    tl = (tl + 11) / 1000.0;
    tid = tid_network.getDouble(-1.0);
    ta = ta_network.getDouble(0);
  }

  public double getTA() {
    return ta;
  }

  public double getX() {
    return x_coordinate;
  }

  public double getLatency() {
    return tl;
  }

  public double getY() {
    return y_coordinate;
  }

  public double getAngle() {
    return angle;
  }

  public double[] getBotPose() {
    botpose[0] = getX();
    botpose[1] = getY();
    return botpose;
  }

  public double getYofTag() {
    if (!hasTarget()) return 0;
    return id_json.get(getID());
  }
}
