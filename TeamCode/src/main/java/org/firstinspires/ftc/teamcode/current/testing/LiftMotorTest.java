package org.firstinspires.ftc.teamcode.current.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LiftMotorTest extends LinearOpMode {

    public DcMotor liftMotor = null; //the lift motor


    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.get(DcMotor.class, "lift");


        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {


            if (gamepad1.right_bumper) {
                liftMotor.setPower(0.2);

            } else if (gamepad1.left_bumper) {
                liftMotor.setPower(-0.2);
            }
            else {
                liftMotor.setPower(0);
            }

        }
    }
}
