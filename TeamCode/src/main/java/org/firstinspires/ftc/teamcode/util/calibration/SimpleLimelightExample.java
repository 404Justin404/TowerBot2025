package org.firstinspires.ftc.teamcode.util.calibration;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.LimelightBallDetector.BallDetection;
import org.firstinspires.ftc.teamcode.subsystem.Robot;


@TeleOp(name = "Simple Limelight Example", group = "Examples")
public class SimpleLimelightExample extends LinearOpMode {

    private Robot robot;
    public static double calibrationConstant = 100.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);

        telemetry.addData("Status", "Initialized - Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.read();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.drive.frontLeftMotor.setPower(frontLeftPower);
            robot.drive.backLeftMotor.setPower(backLeftPower);
            robot.drive.frontRightMotor.setPower(frontRightPower);
            robot.drive.backRightMotor.setPower(backRightPower);

            //LIMELIGHT

            BallDetection closestBall = robot.limelight.getClosestBall();


            if (closestBall != null) {
                telemetry.addData("Ball Found", "Yes");
                telemetry.addData("  Distance", "%.1f inches", closestBall.estimatedDistance);
                telemetry.addData("  Angle", "%.1f degrees", closestBall.tx);
                telemetry.addData("  Confidence", "%.0f%%", closestBall.confidence * 100);

                int totalBalls = robot.limelight.getBallCount();
                telemetry.addData("Total Balls Visible", totalBalls);

                if (closestBall.tx > 5) {
                    telemetry.addData("  Direction", "Ball is to the RIGHT");
                } else if (closestBall.tx < -5) {
                    telemetry.addData("  Direction", "Ball is to the LEFT");
                } else {
                    telemetry.addData("  Direction", "Ball is CENTERED");
                }

            } else {
                telemetry.addData("Ball Found", "No balls detected");
            }



            java.util.List<BallDetection> allBalls = robot.limelight.getDetectedBalls();





            telemetry.update();
        }

        robot.shutdown();
    }
}