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
  private final NetworkTableEntry tx_network;
  private final NetworkTableEntry ty_network;
  private final NetworkTableEntry tv_network;
  private final NetworkTableEntry tid_network;
  private final NetworkTableEntry ts_network;
  private double x_coordinate;
  private double y_coordinate;
  private double tx;
  private double ty;
  private double tv;
  private double tid;
  private double ts;
  private Map<Double, Double> id_json;

  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    botpose_network = networkTable.getEntry("botpose");
    tx_network = networkTable.getEntry("tx");
    ty_network = networkTable.getEntry("ty");
    tv_network = networkTable.getEntry("tv");
    tid_network = networkTable.getEntry("tid");
    ts_network = networkTable.getEntry("ts");
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

  public double horiz_offset() {
    return tx;
  }

  public double vert_offset() {
    return ty;
  }

  public boolean hasTarget() {
    return tv == 1;
  }

  public double getID() {
    return tid;
  }

  public double getSkew(){
    return ts;
  }

  public double[] get_displacement() {
    if (tid <= 8 && tid >= 1) {
      System.out.println(tid);
      double distance = Math.abs(y_coordinate - id_json.get(tid));
      if (tx < 0) {
        double[] output = {distance, -tx};
        return output;
      } else {
        double[] output = {-distance, -tx};
        return output;
      }
    }
    double[] output = {};
    return output;
  }

  @Override
  public void periodic() {
    double[] botpose_data = botpose_network.getDoubleArray(new double[] {});
    if (botpose_data.length > 2) {
      x_coordinate = botpose_data[0];
      y_coordinate = botpose_data[1];
    } else {
      x_coordinate = 0;
      y_coordinate = 0;
    }
    ty = ty_network.getDouble(0);
    tx = tx_network.getDouble(0);
    tv = tv_network.getDouble(0);
    tid = tid_network.getDouble(-1.0);
    ts = ts_network.getDouble(0);
  }

  public double getX() {
    // return length / 2 - x_coordinate - (length - 15.513558);
    return x_coordinate;
  }

  public double getY() {
    return y_coordinate;
  }

  public String getValues() {
    return Arrays.toString(get_displacement());
  }
}
