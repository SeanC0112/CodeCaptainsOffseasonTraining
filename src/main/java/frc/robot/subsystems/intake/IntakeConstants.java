package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class IntakeConstants {
  // ! Must not be in tuning mode for dashboard switching to work
  public static final LoggedNetworkBoolean groundAlgae =
      new LoggedNetworkBoolean("Ground Algae", false);

  public static final int rollersFrontID = 18;
  public static final int rollersBackID = 15;

  public static final boolean rollersFrontInverted = true;
  public static final boolean rollersBackInverted = false;
  public static final double rollersPositionFactor = .2;
  // In volts
  public static LoggedTunableNumber rollersSpeedIn = new LoggedTunableNumber("Intake/rollerSpeedVoltsIn", 8);
  public static LoggedTunableNumber rollersSpeedOut = new LoggedTunableNumber("Intake/rollerSpeedVoltsOut", -8);;
}
