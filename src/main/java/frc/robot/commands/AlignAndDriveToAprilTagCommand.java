package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ToolBox;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.VisionConstants;

import static edu.wpi.first.units.Units.Meters;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignAndDriveToAprilTagCommand extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private final PIDController rotationController, XDriveController, YDriveController;
  private final List<Integer> targetTags;

  private boolean targetVisible = false;
  private double targetYaw = 0.0; 
  private double targetXRange = 0.0;
  private double targetYRange = 0.0;
  private static final double ROTATION_TOLERANCE = 1; // Degrees
  private static final double DISTANCE_TOLERANCE = 0.1; // Meters (50 cm)
  private static final double MAX_ROT_SPEED = 1.5; // Radians/sec
  private static final double MAX_X_DRIVE_SPEED = 0.7; // Meters/sec
  private static final double MAX_Y_DRIVE_SPEED = 0.6; // Meters/sec
  private static final double XDISTANCE_SETPOINT = 0.6; //Meters
  private static final double YDISTANCE_SETPOINT = 0.0; //Meters
  // private static final double VISION_DES_ANGLE_deg = 0.0;
  // private static final double VISION_DES_RANGE_m = 1.25;
  
  public AlignAndDriveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem vision, List<Integer> targetTags) {
      this.m_drive = driveSubsystem;
      this.m_vision = vision;
      this.targetTags = targetTags;
      addRequirements(driveSubsystem, vision);

      rotationController = new PIDController(0.035, 0.0, 0.0);
      XDriveController = new PIDController(0.7, 0.0, 0.0);
      YDriveController = new PIDController(0.5, 0, 0);

      rotationController.setTolerance(ROTATION_TOLERANCE);
      XDriveController.setTolerance(DISTANCE_TOLERANCE);
      YDriveController.setTolerance(DISTANCE_TOLERANCE);

      // rotationController.reset();
      // XDriveController.reset();
      // YDriveController.reset();
  }

  public AlignAndDriveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem vision, Integer targetTag) {
    this(driveSubsystem, vision, List.of(targetTag)); // Calls the primary constructor
  }
  
  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    targetVisible = false;

    if (!m_vision.getLatestResults().isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = m_vision.getLatestResults().get(m_vision.getLatestResults().size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        PhotonTrackedTarget target = result.getBestTarget();
        if (targetTags.contains(target.getFiducialId())) {
          targetVisible = true;
          targetYaw = target.getYaw();
          targetXRange = target.bestCameraToTarget.getMeasureX().in(Meters);
          targetYRange = target.bestCameraToTarget.getMeasureY().in(Meters);
          // PhotonUtils.calculateDistanceToTargetMeters(
          //           0.2, // Measured with a tape measure, or in CAD.
          //           AprilTagConstants.kID_HIGHTS.get( target.getFiducialId() ), 
          //           Units.degreesToRadians(30.0), // Measured with a protractor, or in CAD.
          //           Units.degreesToRadians( target.getPitch() ) );
        }
      }
    }
    //if there is an april tag then calculate and track
    if (targetVisible) {
      SmartDashboard.putNumber("XRange", targetXRange);
      SmartDashboard.putNumber("YRange", targetYRange);
      SmartDashboard.putNumber("Yaw", targetYaw);

      double XSpeed = XDriveController.calculate(targetXRange, XDISTANCE_SETPOINT); // Stop at 0.4m
      // double YSpeed = YDriveController.calculate(targetYRange, YDISTANCE_SETPOINT);
      double turnSpeed = rotationController.calculate(targetYaw, 0.0);

      turnSpeed = MathUtil.clamp(turnSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED); 
      XSpeed = MathUtil.clamp(XSpeed, MAX_X_DRIVE_SPEED, MAX_X_DRIVE_SPEED);
      // YSpeed = MathUtil.clamp(YSpeed, -MAX_Y_DRIVE_SPEED, MAX_Y_DRIVE_SPEED);

      ChassisSpeeds speeds = new ChassisSpeeds(XSpeed, 0, turnSpeed);
      m_drive.drive(speeds, false);
      
      // if (rotationController.atSetpoint() && YDriveController.atSetpoint()){
      //   speeds = new ChassisSpeeds(XSpeed, 0, 0);
      //   m_drive.drive(speeds, false);
      // }
      
    }
    else{
      ChassisSpeeds speeds = new ChassisSpeeds(0, 0.0, 0);
      m_drive.drive(speeds, false);
    }
  }

  @Override
  public boolean isFinished() {
      return rotationController.atSetpoint() && XDriveController.atSetpoint() && YDriveController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      System.out.println("FINISHED");
      m_drive.stop();

      rotationController.reset();
      XDriveController.reset();
  }
}
