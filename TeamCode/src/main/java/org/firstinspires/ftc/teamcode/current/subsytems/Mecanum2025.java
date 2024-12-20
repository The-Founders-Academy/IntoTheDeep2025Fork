package org.firstinspires.ftc.teamcode.current.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
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
    double perpendicularOffsetCentimeters = 20.32;

    public static double TranslationToleranceCentimeters = 0.5;
    public static double RotationToleranceRad = Math.toRadians(3); // 3 Deg

    private PIDController m_translationXController;
    private PIDController m_translationYController;
    private PIDController m_rotationController;

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
        horizontal.setDirection(Motor.Direction.REVERSE);

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

        TelemetryPacket firstTestDebuggingInfo = new TelemetryPacket();

        /*
        TODO Test #1 (record results):
            1) Initiate a teleopmode with the initial pose at (0x, 0y, 0deg) (alliance does not matter)
            2) Instead of using the controller, physically move the robot forward while observing how its x coordinate changes
            3) Do the same thing for the y component
            4) Rotate the robot counterclockwise and observe how all three components change. Only the angle-related quantities should.
            5) Repeat with the initial pose at (0, 0, 90deg), (1,1,0deg), (1,1,90deg)
            You should expect the following results for each initial condition:
            (0, 0, 0deg): Pushing the robot forward while it is at 0deg increases only the x coordinate, while pushing it to the left only increases the y coordinate
            (0, 0, 90deg): Pushing the robot forward while it is at 90deg increases only the y coordinate, while pushing it to the left decreases the x coordinate
            You should expect exactly the same results for the (1,1) tests with corresponding angles.
         */
        firstTestDebuggingInfo.put("Recorded x", m_odo.getPose().getX());
        firstTestDebuggingInfo.put("Recorded y", m_odo.getPose().getY());
        firstTestDebuggingInfo.put("Recorded theta (abs coord)", m_odo.getPose().getRotation().getDegrees());

        // Comment me out when you're done with the first test
        FtcDashboard.getInstance().sendTelemetryPacket(firstTestDebuggingInfo);

        TelemetryPacket secondTestDebuggingInfo = new TelemetryPacket();


        /*
        TODO Test #2 (record results):
            In this test you will simply check that the alliance-relative heading works as expected
            1) Initiate a teleopmode with the intial pose at (0, 0, 90) and on the red alliance
            2) Check that "Recorded heading" is 0 degrees and check that when pushing the robot ccw, it increases heading
            3) Initiate the same teleopmode with the initial pose (0, 0, -90) and on the blue alliance
            4) Do the same checks
            5) If the above worked, try using the controller to drive the robot in both orientations and try to use resetHeading() and see if everything still works
         */
        secondTestDebuggingInfo.put("Recorded x", m_odo .getPose().getX());
        secondTestDebuggingInfo.put("Recorded y", m_odo .getPose().getX());
        secondTestDebuggingInfo.put("Recorded theta (abs coord)", m_odo .getPose().getX());
        secondTestDebuggingInfo.put("Recorded heading (alliance relative coordinates)", m_odo .getPose().getX());

        // FtcDashboard.getInstance().sendTelemetryPacket(secondTestDebuggingInfo);


        TelemetryPacket thirdTestDebuggingInfo = new TelemetryPacket();

        /**
         * TODO Test #3 (record results):
         *  This is the final test and will confirm whether auto works
         *  1) Run your normal auto tests, record videos and graphs
         *  2) If it works, then we're done worrying about this forever :)
         */
        thirdTestDebuggingInfo.put("Recorded x", m_odo .getPose().getX());
        thirdTestDebuggingInfo.put("Recorded y", m_odo .getPose().getX());
        thirdTestDebuggingInfo.put("Recorded theta", m_odo .getPose().getX());
        thirdTestDebuggingInfo.put("Target x", m_translationXController.getSetPoint());
        thirdTestDebuggingInfo.put("Target y", m_translationYController.getSetPoint());
        thirdTestDebuggingInfo.put("Target theta", m_rotationController.getSetPoint());

        // FtcDashboard.getInstance().sendTelemetryPacket(thirdTestDebuggingInfo);


    }


}
