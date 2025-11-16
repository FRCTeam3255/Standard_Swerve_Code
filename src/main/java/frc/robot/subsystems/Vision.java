// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.frcteam3255.utils.LimelightHelpers;
import com.frcteam3255.utils.LimelightHelpers.PoseEstimate;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;

@Logged
public class Vision extends SubsystemBase {
  PoseEstimate lastEstimateFrontRight = new PoseEstimate();
  PoseEstimate lastEstimateFrontLeft = new PoseEstimate();
  PoseEstimate lastEstimateBackRight = new PoseEstimate();
  PoseEstimate lastEstimateBackLeft = new PoseEstimate();

  // Not logged, as they turn to false immediately after being read
  @NotLogged
  boolean newFrontRightEstimate = false;
  @NotLogged
  boolean newFrontLeftEstimate = false;
  @NotLogged
  boolean newBackRightEstimate = false;
  @NotLogged
  boolean newBackLeftEstimate = false;

  Pose2d frontRightPose = new Pose2d();
  Pose2d frontLeftPose = new Pose2d();
  Pose2d backRightPose = new Pose2d();
  Pose2d backLeftPose = new Pose2d();

  private boolean useMegaTag2 = true;

  public Vision() {
  }

  public PoseEstimate[] getLastPoseEstimates() {
    return new PoseEstimate[] {
        lastEstimateFrontRight,
        lastEstimateFrontLeft,
        lastEstimateBackRight,
        lastEstimateBackLeft
    };
  }

  public void setMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro.
   * 
   * @return True if the estimate should be rejected
   */

  public boolean rejectUpdate(PoseEstimate poseEstimate, AngularVelocity gyroRate, double areaThreshold) {
    // Angular velocity is too high to have accurate vision
    if (gyroRate.compareTo(ConstVision.MAX_ANGULAR_VELOCITY) > 0) {
      return true;
    }

    // No tags :<
    if (poseEstimate.tagCount == 0) {
      return true;
    }

    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && poseEstimate.avgTagArea > areaThreshold) {
      return false;
      // 2 tags or more
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  /**
   * Updates the current pose estimates for the front-right, front-left,
   * back-right,
   * and back-left of the robot using data from four Limelight cameras.
   *
   * @param gyroRate The current angular velocity of the robot, used to validate
   *                 the pose estimates.
   *
   *                 This method retrieves pose estimates from four Limelight
   *                 cameras (front-right, front-left, back-right, back-left) and
   *                 updates the
   *                 corresponding pose estimates if they are valid. The method
   *                 supports two modes of operation:
   *                 one using MegaTag2 and one without. The appropriate pose
   *                 estimate retrieval method is chosen
   *                 based on the value of the {@code useMegaTag2} flag.
   *
   *                 If the retrieved pose estimates are valid and not rejected
   *                 based on the current angular velocity,
   *                 the method updates the last known estimates and sets flags
   *                 indicating new estimates are available for each camera.
   */
  public void setCurrentEstimates(AngularVelocity gyroRate) {
    PoseEstimate currentEstimateFrontRight = new PoseEstimate();
    PoseEstimate currentEstimateFrontLeft = new PoseEstimate();
    PoseEstimate currentEstimateBackRight = new PoseEstimate();
    PoseEstimate currentEstimateBackLeft = new PoseEstimate();

    if (useMegaTag2) {
      currentEstimateFrontRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_NAMES[0]);
      currentEstimateFrontLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_NAMES[1]);
      currentEstimateBackRight = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_NAMES[2]);
      currentEstimateBackLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ConstVision.LIMELIGHT_NAMES[3]);
    } else {
      currentEstimateFrontRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_NAMES[0]);
      currentEstimateFrontLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_NAMES[1]);
      currentEstimateBackRight = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_NAMES[2]);
      currentEstimateBackLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue(ConstVision.LIMELIGHT_NAMES[3]);
    }

    if (currentEstimateFrontRight != null
        && !rejectUpdate(currentEstimateFrontRight, gyroRate, ConstVision.AREA_THRESHOLD_FRONT)) {
      lastEstimateFrontRight = currentEstimateFrontRight;
      frontRightPose = currentEstimateFrontRight.pose;
      newFrontRightEstimate = true;
    }
    if (currentEstimateFrontLeft != null
        && !rejectUpdate(currentEstimateFrontLeft, gyroRate, ConstVision.AREA_THRESHOLD_FRONT)) {
      lastEstimateFrontLeft = currentEstimateFrontLeft;
      frontLeftPose = currentEstimateFrontLeft.pose;
      newFrontLeftEstimate = true;
    }
    if (currentEstimateBackRight != null
        && !rejectUpdate(currentEstimateBackRight, gyroRate, ConstVision.AREA_THRESHOLD_BACK)) {
      lastEstimateBackRight = currentEstimateBackRight;
      backRightPose = currentEstimateBackRight.pose;
      newBackRightEstimate = true;
    }
    if (currentEstimateBackLeft != null
        && !rejectUpdate(currentEstimateBackLeft, gyroRate, ConstVision.AREA_THRESHOLD_BACK)) {
      lastEstimateBackLeft = currentEstimateBackLeft;
      backLeftPose = currentEstimateBackLeft.pose;
      newBackLeftEstimate = true;
    }
  }

  public Optional<PoseEstimate> determinePoseEstimate(AngularVelocity gyroRate) {
    setCurrentEstimates(gyroRate);

    // No valid pose estimates :(
    if (!newFrontRightEstimate && !newFrontLeftEstimate && !newBackRightEstimate && !newBackLeftEstimate) {
      return Optional.empty();

    } else if (newFrontRightEstimate && !newFrontLeftEstimate && !newBackRightEstimate && !newBackLeftEstimate) {
      // One valid pose estimate (FR)
      newFrontRightEstimate = false;
      return Optional.of(lastEstimateFrontRight);

    } else if (!newFrontRightEstimate && newFrontLeftEstimate && !newBackRightEstimate && !newBackLeftEstimate) {
      // One valid pose estimate (FL)
      newFrontLeftEstimate = false;
      return Optional.of(lastEstimateFrontLeft);

    } else if (!newFrontRightEstimate && !newFrontLeftEstimate && newBackRightEstimate && !newBackLeftEstimate) {
      // One valid pose estimate (BR)
      newBackRightEstimate = false;
      return Optional.of(lastEstimateBackRight);

    } else if (!newFrontRightEstimate && !newFrontLeftEstimate && !newBackRightEstimate && newBackLeftEstimate) {
      // One valid pose estimate (BL)
      newBackLeftEstimate = false;
      return Optional.of(lastEstimateBackLeft);

    } else if (newFrontRightEstimate && newFrontLeftEstimate && !newBackRightEstimate) {
      // Two valid pose estimates (right and left), average them
      newFrontRightEstimate = false;
      newFrontLeftEstimate = false;
      Pose2d avgPose = new Pose2d(
          (frontRightPose.getX() + frontLeftPose.getX()) / 2.0,
          (frontRightPose.getY() + frontLeftPose.getY()) / 2.0,
          frontRightPose.getRotation().interpolate(frontLeftPose.getRotation(), 0.5));
      PoseEstimate averagedEstimate = new PoseEstimate();
      averagedEstimate.pose = avgPose;
      averagedEstimate.tagCount = lastEstimateFrontRight.tagCount + lastEstimateFrontLeft.tagCount;
      averagedEstimate.avgTagDist = (lastEstimateFrontRight.avgTagDist + lastEstimateFrontLeft.avgTagDist) / 2.0;
      return Optional.of(averagedEstimate);

    } else {
      // More than one valid pose estimate, use the closest one
      newFrontRightEstimate = false;
      newFrontLeftEstimate = false;
      newBackRightEstimate = false;
      newBackLeftEstimate = false;
      if (lastEstimateFrontRight.avgTagDist < lastEstimateFrontLeft.avgTagDist
          && lastEstimateFrontRight.avgTagDist < lastEstimateBackRight.avgTagDist) {
        return Optional.of(lastEstimateFrontRight);
      } else if (lastEstimateFrontLeft.avgTagDist < lastEstimateFrontRight.avgTagDist
          && lastEstimateFrontLeft.avgTagDist < lastEstimateBackRight.avgTagDist) {
        return Optional.of(lastEstimateFrontLeft);
      } else if (lastEstimateBackRight.avgTagDist < lastEstimateFrontRight.avgTagDist
          && lastEstimateBackRight.avgTagDist < lastEstimateFrontLeft.avgTagDist) {
        return Optional.of(lastEstimateBackRight);
      } else if (lastEstimateBackLeft.avgTagDist < lastEstimateFrontRight.avgTagDist
          && lastEstimateBackLeft.avgTagDist < lastEstimateFrontLeft.avgTagDist
          && lastEstimateBackLeft.avgTagDist < lastEstimateBackRight.avgTagDist) {
        return Optional.of(lastEstimateBackLeft);
      } else {
        return Optional.empty();
      }
    }
  }

  @Override
  public void periodic() {
  }
}