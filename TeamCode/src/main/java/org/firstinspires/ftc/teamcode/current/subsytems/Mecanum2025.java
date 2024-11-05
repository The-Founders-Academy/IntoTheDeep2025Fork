package org.firstinspires.ftc.teamcode.current.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.shared.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.MathUtil;

public class Mecanum2025 extends BaseMecanumDrive {

    @Config
    public static class Mecanum2025PARAMS {

        public static double TranslationP = 0.035;
        public static double TranslationI = 0;
        public static double TranslationD = 0;
        public static double RotationP = 3;
        public static double RotationI = 0;
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


    private double m_initialAngleRad;
    private Pose2d m_robotPose;
    private Pose2d m_targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private IMU m_gyro;

    private HolonomicOdometry m_odo;


    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose, alliance);

        m_translationYController = new PIDController(0,0,0);
        m_translationXController = new PIDController(0,0,0);
        m_rotationController = new PIDController(0,0,0);

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

        m_robotPose = initialPose;
        m_initialAngleRad = m_robotPose.getRotation().getRadians();

        double cm_per_tick = 2 * Math.PI * deadWheelRadiusCentimeters / ticksPerRevolution;
        Encoder left = m_frontRight.encoder.setDistancePerPulse(cm_per_tick);
        left.setDirection(Motor.Direction.REVERSE);
        Encoder right = m_frontLeft.encoder.setDistancePerPulse(cm_per_tick);
        right.setDirection(Motor.Direction.REVERSE);
        Encoder horizontal = m_backLeft.encoder.setDistancePerPulse(cm_per_tick);
        m_odo = new HolonomicOdometry(
                left::getDistance,
                right::getDistance,
                horizontal::getDistance,
                trackWidthCentimeters,
                perpendicularOffsetCentimeters
        );

        m_gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        m_gyro.initialize(myIMUparameters);

        // m_odo is tracking heading / angle offset, so set its initial rotation to 0
        m_odo.updatePose(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(0)));

    }

    @Override
    public Rotation2d getHeading() {
        return m_robotPose.getRotation().minus(new Rotation2d(Math.PI / 2));
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(0,0,Rotation2d.fromDegrees(0));
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_robotPose = pose;
    }

    public void resetHeading() {
        m_odo.updatePose(new Pose2d(m_odo.getPose().getX(), m_odo.getPose().getY(), Rotation2d.fromDegrees(0)));
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

    public void moveFieldRelativeForPID() {
        double vX = MathUtil.clamp(m_translationXController.calculate(m_robotPose.getX()),      // Uses the "error" to calculate appropriate speed, between min and max speed
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());
        double vY = MathUtil.clamp(m_translationYController.calculate(m_robotPose.getY()),      // same for y
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

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vY, -vX, vOmega, getHeading()); // Transform the x and y coordinates to account for differences between global field coordinates and driver field coordinates
        move(speeds);
    }

    public void stop() {
        m_frontLeft.stopMotor();
        m_frontRight.stopMotor();
        m_backLeft.stopMotor();
        m_backRight.stopMotor();
    }

    @Override
    public void periodic() {
        tunePIDS();
        m_odo.updatePose();

        double currentAngleRad = m_initialAngleRad - m_odo.getPose().getHeading(); // Initial + Heading
        m_robotPose = new Pose2d(m_odo.getPose().getX(), -m_odo.getPose().getY(), new Rotation2d(currentAngleRad));
    }

}
