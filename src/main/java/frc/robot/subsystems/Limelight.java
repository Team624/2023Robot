package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.LimelightData;

public class Limelight extends SubsystemBase {
  private final NetworkTable networkTable;

  private LimelightData data;
  private String name;

  public Limelight(String name) {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight-" + name);
    this.name = name;
  }

  public void setPipeline(int pipeline) {
    networkTable
        .getEntry("pipeline")
        .setNumber(pipeline);
  }

  public double[] getAlignmentValues() {
    if (data.tid <= 8 && data.tid >= 1) {
      double distance = Math.abs(data.botpose.getY() - Constants.Limelight.tagLocations[data.tid]);
      if (data.ta < 0) {
        double[] output = {-distance, -data.ta};
        return output;

      } else {
        double[] output = {distance, -data.ta};
        return output;
      }
    }
    double[] output = {0, 0};
    return output;
  }

  @Override
  public void periodic() {
    data = new LimelightData(networkTable);
  }

  public LimelightData getData() {
    return data;
  }

  public double getYofTag() {
    if (!data.tv) return 0;
    return Constants.Limelight.tagLocations[data.tid];
  }
}
