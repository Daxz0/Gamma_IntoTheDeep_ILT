package org.firstinspires.ftc.teamcode.opmode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -1.0 * gamepad1.left_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            robot.setDrivePower(flPower / scaling, frPower / scaling, blPower / scaling, brPower / scaling);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));
        }
    }
}