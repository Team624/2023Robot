package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightData {
  // See https://docs.limelightvision.io/en/latest/networktables_api.html for variable descriptions

  public final int pipeline;

  // The time the the data was pulled from NetworkTables
  public final double dataTimestamp;

  public final boolean tv;

  public final double tx, ty, ta, tl, cl, tshort, tlong, thor, tvert;

  public final Pose3d botpose, botpose_wpiblue, botpose_wpired, camerapose_targetspace, targetpose_cameraspace, targetpose_robotspace, botpose_targetspace, camerapose_robotspace;

  public final int tid;

  public LimelightData(NetworkTable table) {
    dataTimestamp = Timer.getFPGATimestamp();

    pipeline = (int) table.getEntry("getpipe").getInteger(-1);
    
    tv = table.getEntry("tv").getInteger(0) == 1;
    tx = table.getEntry("tx").getInteger(0);
    ty = table.getEntry("ty").getInteger(0);
    ta = table.getEntry("ta").getInteger(0);
    tl = table.getEntry("tl").getInteger(0);
    cl = table.getEntry("cl").getInteger(0);
    tshort = table.getEntry("tshort").getInteger(0);
    tlong = table.getEntry("tlong").getInteger(0);
    thor = table.getEntry("thor").getInteger(0);
    tvert = table.getEntry("tvert").getInteger(0);

    botpose = toPose3d(table.getEntry("botpose").getDoubleArray(new double[0]));
    botpose_wpiblue = modifiedPose3d(table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]));
    botpose_wpired = modifiedPose3d(table.getEntry("botpose_wpired").getDoubleArray(new double[0]));
    camerapose_targetspace = toPose3d(table.getEntry("camerapose_targetspace").getDoubleArray(new double[0]));
    targetpose_cameraspace = toPose3d(table.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]));
    targetpose_robotspace = toPose3d(table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]));
    botpose_targetspace = toPose3d(table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]));
    camerapose_robotspace = toPose3d(table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]));

    tid = (int) table.getEntry("tid").getInteger(-1);
  }

  // Converts the Limelight NetworkTables position array to a Pose3d object
  public static Pose3d toPose3d(double[] array) {
    if (array.length < 6) return new Pose3d();
    return new Pose3d(new Translation3d(array[0], array[1], array[2]), new Rotation3d(array[3], array[4], array[5]));
  }

  public static Pose3d modifiedPose3d(double[] array){
    array[1] = -(8.0137-array[1]);
    return new Pose3d(new Translation3d(array[0], array[1], array[2]), new Rotation3d(array[3], array[4], array[5]));
  }

  public Pose3d getPose3d(){
    if(DriverStation.getAlliance()==Alliance.Red){
      return botpose_wpired;
    }
    else{
      return botpose_wpiblue;
    }
  }
}
