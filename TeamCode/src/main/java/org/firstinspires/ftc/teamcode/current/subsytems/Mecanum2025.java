package org.firstinspires.ftc.teamcode.current.subsytems;

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

    private PIDController m_translationXController;
    private PIDController m_translationYController;
    private PIDController m_rotationController;

    private double deadWheelRadiusCentimeters = 2.4;

    private double ticksPerRevolution = 2000.0;
    private double trackWidthCentimeters = 36;
    double perpendicularOffsetCentimeters = -20.32;
    private double m_initialAngleRad;
    private Pose2d m_robotPose;
    private IMU m_gyro;


    public Mecanum2025(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose, Alliance alliance) {
        super(hardwareMap, mecanumConfigs, initialPose, alliance);

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

        m_robotPose = initialPose;
        m_initialAngleRad = initialPose.getHeading();

        double cm_per_tick = 2 * Math.PI * deadWheelRadiusCentimeters / ticksPerRevolution;
        Encoder left = m_frontRight.encoder.setDistancePerPulse(cm_per_tick);
        left.setDirection(Motor.Direction.REVERSE);
        Encoder right = m_frontLeft.encoder.setDistancePerPulse(cm_per_tick);
        right.setDirection(Motor.Direction.REVERSE);
        Encoder horizontal = m_backLeft.encoder.setDistancePerPulse(cm_per_tick);

        HolonomicOdometry m_odo = new HolonomicOdometry(
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
        return Rotation2d.fromDegrees(0);
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(0,0,Rotation2d.fromDegrees(0));
    }

    @Override
    public void resetPose(Pose2d pose) {

    }

}