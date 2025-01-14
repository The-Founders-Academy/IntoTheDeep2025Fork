package org.firstinspires.ftc.teamcode.current.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestLiftSubsystem extends SubsystemBase {
    @Config
    public static class LiftParams {
        public static double TestDistanceTicks = 800; // Vary this in your testing
    }
    private MotorEx m_lift;

    public TestLiftSubsystem(HardwareMap hardwareMap, String name) {
        m_lift = new MotorEx(hardwareMap, name);
        m_lift.setRunMode(Motor.RunMode.PositionControl);
        m_lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); // This should help prevent backsliding
    }

    public void setTargetExtension(double target) {
        m_lift.setTargetDistance(target); // You can choose the units here. Ticks are likely the easiest option.
    }

    public void move(double v) {
        m_lift.set(v);
    }

    public void stop() {
        m_lift.stopMotor();
    }

    public boolean atTargetExtension() {
        m_lift.atTargetPosition();
    }

    @Override
    public void periodic() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current Lift Extension:", m_lift.getDistance()); // Until you manually set encoder resolution, this returns ticks
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
