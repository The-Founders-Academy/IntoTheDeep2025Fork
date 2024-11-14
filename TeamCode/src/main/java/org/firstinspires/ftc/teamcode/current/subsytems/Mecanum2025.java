package org.firstinspires.ftc.teamcode.current.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.current.util.HolonomicOdometry2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.MathUtil;

public class Mecanum2025 extends BaseMecanumDrive {

    @Config
    public static class Mecanum2025PARAMS {

        public static double TranslationP = 0.035;
        public static double TranslationI = 0.0175;
        public static double TranslationD = 0;
        public static double RotationP = 3;
        public static double RotationI = 0.0175;
        public static double RotationD = 0;

    }


    public double deadWheelRadiusCentimeters = 2.4;

    public double ticksPerRevolution = 2000.0;
    public double trackWidthCentimeters = 36.3;
    double perpendicularOffsetCentimeters = -20.32;

    public static double TranslationToleranceCentimeters = 0.5;
    public static double RotationToleranceRad = Math.toRadians(3); // 3 Deg

    private PIDController m_translationXController;
    private PIDController m_translationYController;
    private PIDController m_rotationController;

    private double targetXValue;
    private double targetYValue;
    private Pose2d m_robotPose;
    private Pose2d m_targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    private HolonomicOdometry2025 m_odo;

    private Rotation2d m_referenceRotation = Rotation2d.fromDegrees(0); // Used for heading

    /**
     * Defines the system to use for DriveToPosition commands, used for driving, and used for tuning PIDs and odometry. Connected with BaseMecanumDrive.
     *
     *
     * @param hardwareMap
     * @param mecanumConfigs
     * @param initialPose
     *
     */

    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose);

        m_alliance = alliance;
        m_translationYController = new PIDController(0,0,0);
        m_translationXController = new PIDController(0,0,0);
        m_rotationController = new PIDController(0,0,0);

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

        m_robotPose = initialPose;

        // Stored for telemetry purposes
        targetXValue = initialPose.getX();
        targetYValue = initialPose.getY();
        m_referenceRotation = initialPose.getRotation();

        if(m_alliance == Alliance.RED) {
            m_referenceRotation = Rotation2d.fromDegrees(90);
        } else {
            m_referenceRotation = Rotation2d.fromDegrees(-90);
        }

        double cm_per_tick = 2 * Math.PI * deadWheelRadiusCentimeters / ticksPerRevolution;
        Encoder left = m_frontRight.encoder.setDistancePerPulse(cm_per_tick);
        left.setDirection(Motor.Direction.REVERSE);
        Encoder right = m_frontLeft.encoder.setDistancePerPulse(cm_per_tick);
        right.setDirection(Motor.Direction.REVERSE);
        Encoder horizontal = m_backLeft.encoder.setDistancePerPulse(cm_per_tick);

        m_odo = new HolonomicOdometry2025(
                left::getDistance,
                right::getDistance,
                horizontal::getDistance,
                trackWidthCentimeters,
                perpendicularOffsetCentimeters
        );

        // m_odo is tracking heading / angle offset, so set its initial rotation to 0
        m_odo.updatePose(initialPose);
    }

    @Override
    public Pose2d getPose() {
        return m_robotPose;
    }

    /**
     * This function is never used and will interact poorly with getHeading(), giving weird results.
     * We will probably just remove it. - Kenny 11/13/2024
     * @param pose The new robot pose
     */
    @Deprecated
    @Override
    public void resetPose(Pose2d pose) {
        m_robotPose = pose;
    }

    @Override
    public Rotation2d getHeading() {
        return m_robotPose.getRotation().minus(m_referenceRotation);
    }

    /**
     * The robot heading is calculated as (CurrentAngle - ReferenceAngle) where ReferenceAngle
     * is arbitrary and initialized to zero.
     */
    @Override
    public void resetHeading() {
        m_referenceRotation = m_robotPose.getRotation();
    }

    public void setTargetPose(Pose2d targetPose) {
        m_targetPose = targetPose;
        m_translationXController.setSetPoint(m_targetPose.getX());
        m_translationYController.setSetPoint(m_targetPose.getY());

        double targetRotation = 0;
        if(m_targetPose.getHeading() < 0) {
            targetRotation = m_targetPose.getHeading() + 2 * Math.PI;
        } else {
            targetRotation = m_targetPose.getHeading();
        }

        m_rotationController.setSetPoint(targetRotation);

        TelemetryPacket targetvsOdoPose = new TelemetryPacket();

        targetvsOdoPose.put("Target X:", m_targetPose.getX());
        targetvsOdoPose.put("Target Y:", m_targetPose.getY());
        targetvsOdoPose.put("Target Rotation:", m_targetPose.getRotation());
        targetvsOdoPose.put("Real X:", m_odo.getPose().getX());
        targetvsOdoPose.put("Real Y:", m_odo.getPose().getY());
        targetvsOdoPose.put("Real Rotation:", m_odo.getPose().getRotation());

        FtcDashboard.getInstance().sendTelemetryPacket(targetvsOdoPose);
    }

    public boolean atTargetPose() {
        return (m_translationXController.atSetPoint() && m_translationYController.atSetPoint() && m_rotationController.atSetPoint());
    }

    public void resetPIDS() {
        m_translationXController.reset();
        m_translationYController.reset();
        m_rotationController.reset();
    }

    public void tunePIDS() {
        m_translationXController.setPID(Mecanum2025PARAMS.TranslationP, Mecanum2025PARAMS.TranslationI, Mecanum2025PARAMS.TranslationD);
        m_translationYController.setPID(Mecanum2025PARAMS.TranslationP, Mecanum2025PARAMS.TranslationI, Mecanum2025PARAMS.TranslationD);
        m_rotationController.setPID(Mecanum2025PARAMS.RotationP, Mecanum2025PARAMS.RotationI, Mecanum2025PARAMS.RotationD);

        m_translationXController.setTolerance(TranslationToleranceCentimeters);
        m_translationYController.setTolerance(TranslationToleranceCentimeters);
        m_rotationController.setTolerance(RotationToleranceRad);
    }


    /**
     * This function is used to move during autonomous. Most of the math is done with m_translation and m_rotation controllers,
     * through which a target posed is passed in earlier in the setTargetPose function. The calculate function then determines
     * the appropriate X, Y, and rotational velocities (forced to be in between max and negative max robot speed),
     * to move to the desired target position. This corrects for any error like overshoot and undershoot.
     * When the robot is in close enough proximity to the target position, which is determined by the tolerance values in the tunePIDS() function,
     * the robot stops. the move function converts these X, Y, and rotational speeds to individual wheel speeds, and is the last piece of code required
     * to make the robot move. It constantly generates new velocities based on the updating error from the target position.
     */
    public void moveFieldRelativeForPID() {
        // Uses the "error" to calculate appropriate speed, between min and max speed
        double vX = MathUtil.clamp(m_translationXController.calculate(m_robotPose.getX()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());

        double vY = MathUtil.clamp(m_translationYController.calculate(m_robotPose.getY()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());

        // Do some angle wrapping to ensure the shortest path is taken to get to the rotation target
        double normalizedRotationRad = m_robotPose.getHeading();
        if(normalizedRotationRad < 0) {
            normalizedRotationRad = m_robotPose.getHeading() + 2 * Math.PI; // Normalize to [0, 2PI], -PI becomes PI
        }

        double vOmega = MathUtil.clamp(m_rotationController.calculate(normalizedRotationRad),       // finds appropriate rotation velocity, again between min and max values
                -m_mecanumConfigs.getMaxRobotRotationRps(),
                m_mecanumConfigs.getMaxRobotRotationRps());

        TelemetryPacket motorVelocities = new TelemetryPacket();
        motorVelocities.put("vX: ", vX);
        motorVelocities.put("vY: ", vY);

        FtcDashboard motorVelocityPacket = FtcDashboard.getInstance();
        motorVelocityPacket.sendTelemetryPacket(motorVelocities);

            // Transform the x and y coordinates to account for differences between global field coordinates and driver field coordinates
        Translation2d transformedVelocities = fieldRelativeToAllianceRelative(new Translation2d(vX, vY));
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(transformedVelocities.getX(),
                transformedVelocities.getY(), vOmega, getHeading());
        move(speeds);

    }

    public void stop() {
        m_frontLeft.stopMotor();
        m_frontRight.stopMotor();
        m_backLeft.stopMotor();
        m_backRight.stopMotor();
    }


    // Run periodically, independent of other move functions, to generate Pose data. m_robotPose is used in FieldRelativeDrive in BaseMecanumDrive.
    @Override
    public void periodic() {
        tunePIDS();
        m_odo.updatePose();

        m_robotPose = m_odo.getPose();

        TelemetryPacket robotPose = new TelemetryPacket();
        robotPose.put("X value", m_robotPose.getX());
        robotPose.put("Y value", m_robotPose.getY());
        robotPose.put("Target X value", targetXValue);
        robotPose.put("Target Y value", targetYValue);

        FtcDashboard robotPosePacket = FtcDashboard.getInstance();
        robotPosePacket.sendTelemetryPacket(robotPose);
        

    }


}
