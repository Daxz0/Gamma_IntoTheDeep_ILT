package org.firstinspires.ftc.teamcode.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.RobotConfig;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name = "OpMode3")
public class OpMode3 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    private boolean clawState = false;
    private boolean armState = false;
    private boolean intakeState = false;
    private boolean dumpState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        DcMotor rail = robot.motorlinearRail;

        Servo arm = robot.servoarm;
        Servo claw = robot.servoclaw;
        Servo misumi = robot.servomisumi;

        // intake servos
        CRServo intake = robot.servointake;
        Servo intake_rotate = robot.servorotateintake;

        rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setPosition(0);
        waitForStart();

        double linear_rail_counter = 0;

        final double LINEAR_RAIL_UPPER_LIMIT = -5000;
        final double LINEAR_RAIL_LOWER_LIMIT = 0;

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                double verticalMisumi = gamepad1.right_stick_y;
                double verticalLinear = gamepad1.left_stick_y * 0.6;

                linear_rail_counter += verticalLinear;

                if (linear_rail_counter > LINEAR_RAIL_LOWER_LIMIT) {
                    linear_rail_counter = LINEAR_RAIL_LOWER_LIMIT;
                    rail.setPower(0);
                } else if (linear_rail_counter < LINEAR_RAIL_UPPER_LIMIT) {
                    linear_rail_counter = LINEAR_RAIL_UPPER_LIMIT;
                    rail.setPower(0);
                } else {
                    rail.setPower(verticalLinear);
                }


                if (verticalMisumi > 0){

                    misumi.setPosition(0);

                }
                else if (verticalMisumi < 0){

                    misumi.setPosition(1);

                }
                else{
                    misumi.setPosition(0.5);
                }
            }

            else {
                // --- CONTROLS ---
                // A = CLAW toggle
                // X = CLAW dump toggle
                // Y = INTAKE activate toggle
                // B = INTAKE dump toggle

                // -- Ports --
                // 1 = servo_claw
                // 2 = servo_arm
                // 3 = servo_intake
                // 4 = servo_rotate_intake
                // 5 = servo_musumi

                // --- CLAW TOGGLE ---
                if (gamepad1.a) {
                    clawState = !clawState;
                    claw.setPosition(clawState ? RobotConfig.CLAW_CLOSE : RobotConfig.CLAW_OPEN);
                    sleep(200);
                }

                // --- ARM TOGGLE ---
                if (gamepad1.x) {
                    armState = !armState;
                    arm.setPosition(armState ? RobotConfig.ARM_PICK_POSITION : RobotConfig.ARM_RELEASE_POSITION);
                    sleep(200);
                }

                // --- INTAKE TOGGLE ---
                if (gamepad1.y) {
                    intakeState = !intakeState;
                    intake.setPower(intakeState ? -RobotConfig.INTAKE_POWER : 0);
                    sleep(200);
                }

                // --- DUMP TOGGLE ---
                if (gamepad1.b) {
                    dumpState = !dumpState;
                    intake_rotate.setPosition(dumpState ? RobotConfig.INTAKE_DUMP_OUT : RobotConfig.INTAKE_DUMP_IN);
                    sleep(200);
                }

                // --- DRIVE CONTROL ---
                double horizontal = -gamepad1.right_stick_x * 0.6;
                double vertical = gamepad1.right_stick_y * 0.6;
                double turn = -gamepad1.left_stick_x * 0.6;

                double flPower = vertical + turn + horizontal;
                double frPower = vertical - turn - horizontal;
                double blPower = vertical + turn - horizontal;
                double brPower = vertical - turn + horizontal;

                double max = Math.max(1.0, Math.max(Math.abs(flPower),
                        Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));

                robot.setDrivePower(flPower / max, frPower / max, blPower / max, brPower / max);
            }

            // --- TELEMETRY ---
            telemetry.addData("Claw", clawState ? "Closed" : "Open");
            telemetry.addData("Slide Count", linear_rail_counter);
            telemetry.addData("Misumi Position", misumi.getPosition());
            telemetry.update();
        }
    }
}
