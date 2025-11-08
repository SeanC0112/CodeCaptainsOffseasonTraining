package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.groundAlgae;
import static frc.robot.util.AllianceFlipUtil.apply;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.PoseManager;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Put high level commands here */
public final class RobotCommands {
  public static boolean allowAutoDrive = true;

  private static BooleanSupplier nearPose(PoseManager poseManager, Supplier<Pose2d> goalPose) {
    return () -> {
      boolean extra = false;
      if (DriverStation.isTeleop()) {
        extra = !allowAutoDrive;
      } else if (DriverStation.isTest()) {
        extra = true;
      }
      ;
      return extra
          || poseManager.getDistanceTo(goalPose.get()) < elevatorSafeExtensionDistanceMeters.get();
    };
  }

  public static BooleanSupplier atGoal(Drive drive, DriveCommandsConfig driveCommandsConfig) {
    return () ->
        driveCommandsConfig.finishScoring() || (drive.linearAtGoal() && drive.thetaAtGoal());
  }

}
