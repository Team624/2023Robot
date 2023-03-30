package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;

public class Limelight extends SubsystemBase {

  private final NetworkTable networkTable;
  private final NetworkTableEntry botpose_network;
  private final NetworkTableEntry campose_network;
  private final NetworkTableEntry tid_network;
  private final NetworkTableEntry ta_network;
  private final NetworkTableEntry tx_network;
  private double ta;
  private double tl;
  private double x_coordinate;
  private double y_coordinate;
  private double angle;
  private double tid;
  private double[] botpose;
  private Map<Double, Double> id_json;
  private int pipeline_index = 0;

  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    ta_network = networkTable.getEntry("ta");
    botpose_network = networkTable.getEntry("botpose");
    campose_network = networkTable.getEntry("campose");
    tid_network = networkTable.getEntry("tid");
    tx_network = networkTable.getEntry("tx");
    id_json = new HashMap<Double, Double>();
    id_json.put(1.0, -6.94659);
    id_json.put(2.0, -5.27019);
    id_json.put(3.0, -3.59379);
    id_json.put(4.0, -1.26839);
    id_json.put(5.0, -1.26839);
    id_json.put(6.0, -3.59379);
    id_json.put(7.0, -5.2701);
    id_json.put(8.0, -6.94659);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }

  public void changePipelined(int pipeline) {
    System.out.println("Setting the pipeline");
    pipeline_index = pipeline;
    NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("pipeline")
        .setNumber(pipeline);
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
    double[] output = {0, 0};
    return output;
  }

  @Override
  public void periodic() {
    if (pipeline_index == 1) {
      angle = tx_network.getDouble(0);
    } else {
      botpose = botpose_network.getDoubleArray(new double[] {});
      if (botpose.length == 7) {
        if (DriverStation.getAlliance() == Alliance.Blue) {
          botpose[0] = botpose[0] + 8.27;
          botpose[1] = botpose[1] - 4.01;
        } else {
          botpose[0] = -botpose[0] + 8.27;
          botpose[1] = -botpose[1] - 4.01;
        }
        x_coordinate = botpose[0];
        y_coordinate = botpose[1];
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
    return botpose;
  }

  public double getYofTag() {
    if (!hasTarget()) return 0;
    return id_json.get(getID());
  }
}
