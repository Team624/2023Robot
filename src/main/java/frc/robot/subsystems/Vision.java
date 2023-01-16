package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private final NetworkTable networkTable;
  private final NetworkTableEntry botpose_network;
  private final NetworkTableEntry tx_network;
  private final NetworkTableEntry ty_network;
  private final NetworkTableEntry tv_network;
  private double x_coordinate;
  private double y_coordinate;
  private double tx;
  private double ty;
  private double tv;
  private final double width = 8.02;
  private final double length = 16.54;

  public Vision() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    botpose_network = networkTable.getEntry("botpose");
    tx_network = networkTable.getEntry("tx");
    ty_network = networkTable.getEntry("ty");
    tv_network = networkTable.getEntry("tv");
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
  }

  public double getX() {
    //return length / 2 - x_coordinate - (length - 15.513558);
    return x_coordinate;
  }

  public double getY() {
    return y_coordinate;
  }

  public String getValues() {
    return "X: " + getX() + " Y: " + getY();
  }
}
