package org.firstinspires.ftc.teamcode.current.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift2025 extends SubsystemBase {
    public DcMotor liftMotor;


    @Config
    public static class Lift2025Params {
        public static int LIFT_MAX = 1220;
        public static int LIFT_COLLAPSED = 5;
        public static double Kp = 0.02;
        public static double smallKp = 0.015;

    }

    public Lift2025(final HardwareMap hardwaremap) {
        liftMotor = hardwaremap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // resets encoder
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveLift(int targetPosition) {
            liftMotor.setTargetPosition(targetPosition);
    }

    @Override
    public void periodic() {
        double error = (liftMotor.getTargetPosition() - liftMotor.getCurrentPosition());
        if(error > 500) {
            double power = (Lift2025Params.Kp * error);
            liftMotor.setPower(-power);
        }
        else {
            double power = (Lift2025Params.smallKp * error);
            liftMotor.setPower(-power);
        }



        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current Motor Position: ", liftMotor.getCurrentPosition());
        packet.put("error", error);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }



}
