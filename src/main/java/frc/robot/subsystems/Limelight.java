package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Limelight extends SubsystemBase {

  private final NetworkTable networkTable;
  private final NetworkTableEntry botpose_network;
  private final NetworkTableEntry camtran_network;
  private final NetworkTableEntry tv_network;
  private final NetworkTableEntry tid_network;
  private double x_coordinate;
  private double y_coordinate;
  private double angle;
  private double tv;
  private double tid;

  private double[] botpose;
  private Map<Double, Double> id_json;

  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    botpose_network = networkTable.getEntry("botpose");
    camtran_network = networkTable.getEntry("camtran");
    tv_network = networkTable.getEntry("tv");
    tid_network = networkTable.getEntry("tid");
    id_json = new HashMap<Double, Double>();
    id_json.put(1.0, -2.93659);
    id_json.put(2.0, -1.26019);
    id_json.put(3.0, 0.41621);
    id_json.put(4.0, 2.74161);
    id_json.put(5.0, 2.74161);
    id_json.put(6.0, 0.41621);
    id_json.put(7.0, -1.26019);
    id_json.put(8.0, -2.93659);
  }

  public boolean hasTarget() {
    return tv == 1;
  }

  public double getID() {
    return tid;
  }

  public double[] alignment_values() {
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
    if (botpose_data.length == 6) {
      x_coordinate = botpose_data[0];
      y_coordinate = botpose_data[1];
    } else {
      x_coordinate = 0;
      y_coordinate = 0;
    }
    double[] camtran = camtran_network.getDoubleArray(new double[] {});
    if (camtran.length == 6) {
      angle = camtran[4];
    } else {
      angle = 180;
    }
    tv = tv_network.getDouble(0);
    tid = tid_network.getDouble(-1.0);
  }

  public double getX() {
    return x_coordinate;
  }

  public double getY() {
    return y_coordinate;
  }

  public double getAngle() {
    return angle;
  }

  public double[] getBotPose() {

    // width = 8.02
    // length = 16.54

    botpose[0] = getX() + 8.27;
    botpose[1] = getY() - 4.01;
    return botpose;
  }

  public String getValues() {
    return Arrays.toString(alignment_values());
  }
}
