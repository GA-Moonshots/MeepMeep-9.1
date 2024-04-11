package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.InputConfig;


@TeleOp(name="Toth")
public class Toth extends LinearOpMode {

    private MecanumDrive driveyMcDriveDriveDriverson;
    private final ElapsedTime timeyMcTimeTimerTimerson = new ElapsedTime();


    @Override
    public void runOpMode() {
        // Init (runs once)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveyMcDriveDriveDriverson = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        

        // Init Loop (runs until stop button or start button is pressed)
        while(opModeInInit()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("G2LS", "(%f, %f)", gamepad2.left_stick_x, gamepad2.left_stick_y);
            telemetry.addData("G2RS", "(%f, %f)", gamepad2.right_stick_x, gamepad2.right_stick_y);
            telemetry.update();
        }
        // Start (runs once)
        telemetry.addData("Status", "Started");
        telemetry.update();

        timeyMcTimeTimerTimerson.reset();

        // Main (runs until stop is pressed)
        while(opModeIsActive()) {
            telemetry.addData("G1LS", "(%f, %f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("G1RS", "(%f, %f)", gamepad1.right_stick_x, gamepad1.right_stick_y);

            //
            // Driver 1: Responsible for drivetrain and movement
            driver1Inputs();


            telemetry.update();
        }
        // Stop (runs once)
        driveyMcDriveDriveDriverson.stop();
    }


    private void driver1Inputs() {


        telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // DEADZONES
        if (Math.abs(forward) <= InputConfig.INPUT_THRESHOLD) forward = 0.0d;
        if (Math.abs(strafe) <= InputConfig.INPUT_THRESHOLD)  strafe = 0.0d;
        if (Math.abs(turn) <= InputConfig.INPUT_THRESHOLD) turn = 0.0d;

        driveyMcDriveDriveDriverson.setDrivePowers(new PoseVelocity2d(new Vector2d(strafe, forward), turn));
    }



}
    

