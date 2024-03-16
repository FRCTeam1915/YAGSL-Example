package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.game.Shot;
import frc.robot.game.VisionShotLibrary;
import frc.robot.util.PolarCoordinate;
import frc.robot.util.fieldmirroring.BlueAllianceFieldPoseCollection;
import frc.robot.util.fieldmirroring.FlippableBlueAlliancePose;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.function.DoubleSupplier;

public class Vision extends SubsystemBase {

  private PhotonCamera frontCam;
  // mFrontRightCam, mBackLeftCam, mBackRightCam;
  private PhotonPoseEstimator mFrontCamEstimator;
  // mFrontRightEstimator, mBackLeftEstimator, mBackRightEstimator;
  private FlippableBlueAlliancePose mBlueSpeakerPose;
  private FlippableBlueAlliancePose mBlueAmpPose;
  private BlueAllianceFieldPoseCollection mBlueAllianceTrapPoses;
  private DoubleSupplier mVAngle;
  private DoubleSupplier mVSpeed;
  private Shot visionShot;
  private static Translation3d ORIGINTRANSLATION = new Translation3d();

  private AprilTagFieldLayout mFieldLayout;

  private final SwerveSubsystem SWERVE = RobotContainer.drivebase;

  public Vision() {
    frontCam = new PhotonCamera("Front Cam");
    // mFrontRightCam = new PhotonCamera("RightFront");

    mFrontCamEstimator = new PhotonPoseEstimator(mFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCam, Constants.kFrontLeftCamToCenter);
    // mFrontRightEstimator = new PhotonPoseEstimator(mFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // mFrontRightCam, Constants.kFrontRightCamToCenter);
    // mBackLeftEstimator = new PhotonPoseEstimator(mFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mBackLeftCam,
    // Constants.kBackLeftCamToCenter);
    // mBackRightEstimator = new PhotonPoseEstimator(mFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // mBackRightCam, Constants.kBackRightCamToCenter);

    mFrontCamEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // mFrontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // mBackLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // mBackRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    mBlueSpeakerPose = new FlippableBlueAlliancePose(new Translation2d(0.2, 5.55), Rotation2d.fromDegrees(180));
    mBlueAmpPose = new FlippableBlueAlliancePose(new Translation2d(1.84, 7.64), Rotation2d.fromDegrees(90.0));

    ArrayList<FlippableBlueAlliancePose> blueTrapPoses = new ArrayList<>();
    blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.36, 4.92), Rotation2d.fromDegrees(-60.0)));
    blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(4.41, 3.33), Rotation2d.fromDegrees(60.0)));
    blueTrapPoses.add(new FlippableBlueAlliancePose(new Translation2d(5.78, 4.1), Rotation2d.fromDegrees(180.0)));

    mBlueAllianceTrapPoses = new BlueAllianceFieldPoseCollection(blueTrapPoses);

    mFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    mFrontCamEstimator.setFieldTags(mFieldLayout);
    // mFrontRightEstimator.setFieldTags(mFieldLayout);
    // mBackLeftEstimator.setFieldTags(mFieldLayout);
    // mBackRightEstimator.setFieldTags(mFieldLayout);
  }

  public Rotation2d getYawToSpeaker() {
    Translation2d difference = getYAxisMovementCompensatedSpeakerPose()
        .minus(RobotContainer.drivebase.getPose().getTranslation());
    return Rotation2d
        .fromRadians(PolarCoordinate.toPolarCoordinate(() -> difference.getX(), () -> difference.getY())[1]);
  }

  public Translation2d getYAxisMovementCompensatedSpeakerPose() {
    return getSpeakerTranslation()
        .plus(new Translation2d(0.0, -(RobotContainer.drivebase.getFieldOrientedVelocity().vyMetersPerSecond * .3)));
  }

  public Translation2d getSpeakerTranslation() {
    return mBlueSpeakerPose.getTranslation(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
  }

  public Pose2d getAmpGoalPose2d() {
    return mBlueAmpPose.getPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance());
  }

  public Pose2d getNearestTrapPose() {
    return mBlueAllianceTrapPoses.getNearestPose(RobotContainer.S_DRIVERSTATIONCHECKER.getCurrentAlliance(),
        RobotContainer.drivebase.getPose());
  }

  public Double getDistanceToSpeaker() {
    return RobotContainer.drivebase.getPose().getTranslation().getDistance(getSpeakerTranslation());
  }

  public DoubleSupplier CalculateShotAngle() {
    visionShot = VisionShotLibrary.getShotForDistance(getDistanceToSpeaker());
    mVAngle = visionShot::getPivotAngle;
    return mVAngle;
  }

  public DoubleSupplier CalculateShotSpeed() {
    visionShot = VisionShotLibrary.getShotForDistance(getDistanceToSpeaker());
    mVSpeed = visionShot::getLauncherSpeed;
    return mVSpeed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);

    try {
      if (mFrontCamEstimator.update().isPresent() && updateIsValid(mFrontCamEstimator.update().get())) {
        EstimatedRobotPose update = mFrontCamEstimator.update().orElseThrow();
        if (update != null) {
          RobotContainer.drivebase.addVisionMeasurement(update.estimatedPose.toPose2d(), update.timestampSeconds);
        }
      }
    } catch (NoSuchElementException e) {
      // System.out.println("Vision.java: Front left estimator had no update to get");
    }

    // try {
    // if (mFrontRightEstimator.update().isPresent() &&
    // updateIsValid(mFrontRightEstimator.update().get())) {
    // EstimatedRobotPose update = mFrontRightEstimator.update().orElseThrow();
    // if (update != null) {
    // RobotContainer.drivebase.addVisionMeasurement(update.estimatedPose.toPose2d(),
    // update.timestampSeconds);
    // }
    // }
    // } catch (NoSuchElementException e) {
    // // System.out.println("Vision.java: Front right estimator had no update to
    // // get");
    // }

    // try {
    // if (mBackLeftEstimator.update().isPresent() &&
    // updateIsValid(mBackLeftEstimator.update().get())) {
    // EstimatedRobotPose update = mBackLeftEstimator.update().orElseThrow();
    // if (update != null) {
    // RobotContainer.drivebase.addVisionMeasurement(update.estimatedPose.toPose2d(),
    // update.timestampSeconds);
    // }
    // }
    // } catch (NoSuchElementException e) {
    // // System.out.println("Vision.java: Back left estimator had no update to
    // get");
    // }
    // try {
    // if (mBackRightEstimator.update().isPresent() &&
    // updateIsValid(mBackRightEstimator.update().get())) {
    // EstimatedRobotPose update = mBackRightEstimator.update().orElseThrow();
    // if (update != null) {
    // RobotContainer.drivebase.addVisionMeasurement(update.estimatedPose.toPose2d(),
    // update.timestampSeconds);
    // }
    // }
    // } catch (NoSuchElementException e) {
    // // System.out.println("Vision.java: Back right estimator had no update to
    // get");
    // }
  }

  public void resetRobotOdometryFromVision() {
    boolean hasBeenReset = false;
    try {
      if (mFrontCamEstimator.update().isPresent() && updateIsValid(mFrontCamEstimator.update().get())) {
        hasBeenReset = true;

        RobotContainer.drivebase.resetOdometry(mFrontCamEstimator.update().get().estimatedPose.toPose2d());
      }
    } catch (NoSuchElementException e) {
      // ignore
    }
    // try {
    // if (mFrontRightEstimator.update().isPresent() &&
    // updateIsValid(mFrontLeftEstimator.update().get())) {
    // hasBeenReset = true;
    // RobotContainer.drivebase.resetOdometry(mFrontRightEstimator.update().get().estimatedPose.toPose2d());
    // }
    // } catch (NoSuchElementException e) {
    // // ignore
    // }
    // try {
    // if (mBackLeftEstimator.update().isPresent() &&
    // updateIsValid(mBackLeftEstimator.update().get())) {
    // hasBeenReset = true;
    // RobotContainer.drivebase.resetOdometry(mBackLeftEstimator.update().get().estimatedPose.toPose2d());
    // }
    // } catch (NoSuchElementException e) {
    // // ignore
    // }
    // try {
    // if (mBackRightEstimator.update().isPresent() &&
    // updateIsValid(mBackRightEstimator.update().get())) {
    // hasBeenReset = true;
    // RobotContainer.drivebase.resetOdometry(mBackRightEstimator.update().get().estimatedPose.toPose2d());
    // }
    // } catch (NoSuchElementException e) {
    // // ignore
    // }
  }

  private boolean updateIsValid(EstimatedRobotPose estimatedRobotPose) {
    return updateHasMultipleTargets(estimatedRobotPose) && updateHasAllNearTargets(estimatedRobotPose);
  }

  private boolean updateHasMultipleTargets(EstimatedRobotPose estimatedRobotPose) {
    return estimatedRobotPose.targetsUsed.size() > 1.0;
  }

  private boolean updateHasAllNearTargets(EstimatedRobotPose estimatedRobotPose) {
    for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
      double distanceToTag = target.getBestCameraToTarget().getTranslation().getDistance(ORIGINTRANSLATION);
      if (distanceToTag > 6.0) {
        return false;
      }
    }
    return true;

  }

  @Override
  public void simulationPeriodic() {
  }

}