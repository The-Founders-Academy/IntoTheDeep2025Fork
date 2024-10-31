package org.firstinspires.ftc.teamcode.current.opmodes;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp()
//@Disabled
public class newDrive2025 extends LinearOpMode {

    public MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;


    DcMotor armMotor = null; //the arm motor
    Servo wrist = null; //the wrist servo

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;


    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;


    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position in later functions*/
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    @Override
    public void runOpMode() {


        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        // Initializes the drive motors to the correct hardwaremap
        m_frontLeft = new MotorEx(hardwareMap,"fL", Motor.GoBILDA.RPM_312);
        m_frontRight = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_312);
        m_backLeft = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_312);
        m_backRight = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_312);



        m_frontLeft.setRunMode(Motor.RunMode.RawPower);
        m_frontRight.setRunMode(Motor.RunMode.RawPower);
        m_backLeft.setRunMode(Motor.RunMode.RawPower);
        m_backRight.setRunMode(Motor.RunMode.RawPower);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // resets field relative drive forward direction to where the robot is currently facing when pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            m_frontLeft.set(frontLeftPower);
            m_backLeft.set(backLeftPower);
            m_frontRight.set(frontRightPower);
            m_backRight.set(backRightPower);


            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/


//            if (gamepad1.left_bumper) {
//                robot.intake.setPower(robot.INTAKE_COLLECT);
//            }
//            else if (gamepad1.right_bumper) {
//                robot.intake.setPower(robot.INTAKE_OFF);
//            }
//            else if (gamepad1.y) {
//                robot.intake.setPower(robot.INTAKE_DEPOSIT);
//            }


            /* this "FUDGE_FACTOR" allows you to move +15 and -15 degrees outside of the target position set, by holding down
            the left or right trigger. So if your target position was 160 degrees, you could really be anywhere from 145 to
            175. This helps to fine tune arm position for things like hanging. May remove later.
             */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));


            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if(gamepad1.a){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
//                liftPosition = LIFT_COLLAPSED;
                wrist.setPosition(WRIST_FOLDED_OUT);
//                intake.setPower(robot.INTAKE_COLLECT);

            }

            else if (gamepad1.b){
                    /*This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.x){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                //liftPosition = LIFT_COLLAPSED;
//                intake.setPower(robot.INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                  armPosition = ARM_ATTACH_HANGING_HOOK;
//                robot.intake.setPower(robot.INTAKE_OFF);
                  wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                  armPosition = ARM_WINCH_ROBOT;
//                robot.intake.setPower(robot.INTAKE_OFF);
                  wrist.setPosition(WRIST_FOLDED_IN);
            }

//            else if (gamepad2.y) {
//                liftPosition = robot.LIFT_SCORING_IN_HIGH_BASKET;
//            }
//
//            else if (gamepad2.a) {
//                liftPosition = robot.LIFT_COLLAPSED;
//            }


//            if (armPosition < 45 * robot.ARM_TICKS_PER_DEGREE){
//                armLiftComp = (0.25568 * liftPosition);
//            }
//            else{
//                armLiftComp = 0;
//            }


           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}