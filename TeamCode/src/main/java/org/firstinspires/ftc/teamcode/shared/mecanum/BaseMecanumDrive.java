package org.firstinspires.ftc.teamcode.shared.mecanum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;
import org.firstinspires.ftc.teamcode.shared.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.shared.util.MathUtil;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public abstract class BaseMecanumDrive extends SubsystemBase {

    public enum Alliance {
        RED, BLUE
    }


    protected MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    protected MecanumDriveKinematics m_kinematics;
    protected MecanumConfigs m_mecanumConfigs;
    protected Alliance m_alliance;

    public Mecanum2025 m_mecanumDrive;

    public abstract Rotation2d getHeading();
    public abstract Pose2d getPose();
    public abstract void resetPose(Pose2d pose);
    public abstract void resetHeading();
    private VoltageSensor myControlHubVoltageSensor;
    public double currentVoltage;


    public BaseMecanumDrive(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        m_mecanumConfigs = mecanumConfigs;
        m_frontLeft = new MotorEx(hardwareMap, m_mecanumConfigs.getFrontLeftName(), Motor.GoBILDA.RPM_312);
        m_frontRight = new MotorEx(hardwareMap, m_mecanumConfigs.getFrontRightName(), Motor.GoBILDA.RPM_312);
        m_backLeft = new MotorEx(hardwareMap, m_mecanumConfigs.getBackLeftName(), Motor.GoBILDA.RPM_312);
        m_backRight = new MotorEx(hardwareMap, m_mecanumConfigs.getBackRightName(), Motor.GoBILDA.RPM_312);
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        m_frontLeft.setRunMode(m_mecanumConfigs.getRunMode());
        m_frontRight.setRunMode(m_mecanumConfigs.getRunMode());
        m_backLeft.setRunMode(m_mecanumConfigs.getRunMode());
        m_backRight.setRunMode(m_mecanumConfigs.getRunMode());

        m_kinematics = new MecanumDriveKinematics(m_mecanumConfigs.getFrontLeftPosition(), m_mecanumConfigs.getFrontRightPosition(),
                m_mecanumConfigs.getBackLeftPosition(), m_mecanumConfigs.getBackRightPosition());


    }

    protected void move(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
        m_frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_backLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_backRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
    }


    /**
     * @param xPercentVelocity The forward velocity. Ranges from -1 to 1.
     * @param yPercentVelocity The leftward (from the driverstation) velocity. Ranges from -1 to 1.
     * @param omegaPercentVelocity The rotational velocity. Positive indicates cc rotation. Ranges from -1 to 1.
     */
    public void moveRobotRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double vXMps = xPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double vYMps = yPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double omegaRps = omegaPercentVelocity * m_mecanumConfigs.getMaxRobotRotationRps();
        ChassisSpeeds speeds = new ChassisSpeeds(vXMps, vYMps, omegaRps);
        move(speeds);
    }

    public Translation2d fieldRelativeToAllianceRelative(Translation2d translation) {
        Translation2d transformedTranslation;
        if(m_alliance == Alliance.RED) {
            transformedTranslation = new Translation2d(translation.getY(), -translation.getX());
        } else {
            transformedTranslation = new Translation2d(-translation.getY(), translation.getX());
        }
        return transformedTranslation;
    }

    /**
     * @param xPercentVelocity The forward velocity. Ranges from -1 to 1.
     * @param yPercentVelocity The leftward (from the driverstation) velocity. Ranges from -1 to 1.
     * @param omegaPercentVelocity The rotational velocity. Positive indicates cc rotation. Ranges from -1 to 1.
     */
    public void moveAllianceRelative(double xPercentVelocity, double yPercentVelocity, double omegaPercentVelocity) {
        double vXMps = xPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double vYMps = yPercentVelocity * m_mecanumConfigs.getMaxRobotSpeedMps();
        double omegaRps = omegaPercentVelocity * m_mecanumConfigs.getMaxRobotRotationRps();
        ChassisSpeeds speeds;

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getHeading());

        TelemetryPacket heading = new TelemetryPacket();
        heading.put("heading", getHeading());

        FtcDashboard dashboard = FtcDashboard.getInstance();

        dashboard.sendTelemetryPacket(heading);

        move(speeds);
    }

    public void periodic() {
        currentVoltage = myControlHubVoltageSensor.getVoltage();

        TelemetryPacket voltage = new TelemetryPacket();
        voltage.put("currentVoltage: ", currentVoltage);
        FtcDashboard.getInstance().sendTelemetryPacket(voltage);
    }
}
