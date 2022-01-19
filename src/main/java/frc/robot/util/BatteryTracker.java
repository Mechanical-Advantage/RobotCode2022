package frc.robot.util;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Robot;

public class BatteryTracker extends SubsystemBase {
  private static BatteryTracker instance = null;

  private BatteryTracker() {}

  public static BatteryTracker getInstance() {
    if (instance == null) {
      instance = new BatteryTracker();
    }
    return instance;
  }

  private static final List<Robot> supportedRobots = List.of(Robot.ROBOT_KITBOT);

  private static final int nameLength = 12;
  private static final byte[] scanCommand = new byte[] {0x7e, 0x00, 0x08, 0x01,
      0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd};
  private static final byte[] responsePrefix =
      new byte[] {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};
  private static final byte endMark = 0x0d; // CR
  private static final int fullResponseLength =
      responsePrefix.length + nameLength + 1;

  private static class BatteryInfo implements LoggableInputs {
    public String name = "BAT-0000-000";

    @Override
    public void toLog(LogTable table) {
      table.put("Name", name);
    }

    @Override
    public void fromLog(LogTable table) {
      name = table.getString("Name", name);
    }
  }

  private final BatteryInfo data = new BatteryInfo();

  /**
   * Scans the battery. This should be called before the first loop cycle
   * 
   * @param timeout The time to wait before giving up
   */
  public void scanBattery(double timeout) {
    if (Constants.getMode() == Mode.REAL) {
      if (supportedRobots.contains(Constants.getRobot())) {
        // Only scan on supported robots and in real mode

        try (SerialPort port = new SerialPort(9600, SerialPort.Port.kUSB)) {
          port.setTimeout(timeout);
          port.setWriteBufferSize(scanCommand.length);
          port.setReadBufferSize(fullResponseLength);

          port.write(scanCommand, scanCommand.length);
          byte[] response = port.read(fullResponseLength);

          // Ensure response is correct length
          if (response.length != fullResponseLength) {
            System.out.println("[BatteryTracker] Expected " + fullResponseLength
                + " bytes from scanner, got " + response.length);
            return;
          }

          // Ensure response starts with prefix
          for (int i = 0; i < responsePrefix.length; i++) {
            if (response[i] != responsePrefix[i]) {
              System.out.println(
                  "[BatteryTracker] Invalid prefix from scanner.  Got data:");
              System.out
                  .println("[BatteryTracker] " + Arrays.toString(response));
              return;
            }
          }

          // Ensure response ends with suffix
          if (response[response.length - 1] != endMark) {
            System.out
                .println("[BatteryTracker] Invalid suffix from scanner.  Got "
                    + response[response.length - 1]);
          }

          // Read name from data
          byte[] batteryNameBytes = new byte[nameLength];
          System.arraycopy(response, responsePrefix.length, batteryNameBytes, 0,
              nameLength);
          data.name = new String(batteryNameBytes);
          System.out.println("[BatteryTracker] Scanned battery " + data.name);


        } catch (Exception e) {
          System.out.println(
              "[BatteryTracker] Exception while trying to scan battery");
          e.printStackTrace();
        }
      }
    }
  }

  @Override
  public void periodic() {
    Logger.getInstance().processInputs("BatteryInfo", data);
  }

  /**
   * Returns the name of the detected battery, or BAT-0000-000 if no battery was detected. The
   * format is BAT-YEAR-NUM, where NUM starts at 001.
   * 
   * @return
   */
  public String getBatteryName() {
    return data.name;
  }

}
