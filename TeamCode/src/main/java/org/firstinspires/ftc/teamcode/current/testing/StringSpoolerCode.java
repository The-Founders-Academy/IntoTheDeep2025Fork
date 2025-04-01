package org.firstinspires.ftc.teamcode.current.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class StringSpoolerCode extends LinearOpMode {

    private DcMotor firstMotor;
    private DcMotor secondMotor;

    @Config
    public static class SpoolerParams   {
        public static double speed = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        firstMotor = hardwareMap.get(DcMotor.class, "testmotor");
        secondMotor = hardwareMap.get(DcMotor.class, "testmotor2");

        secondMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            firstMotor.setPower(SpoolerParams.speed);
            secondMotor.setPower(SpoolerParams.speed);


        }
    }
}
