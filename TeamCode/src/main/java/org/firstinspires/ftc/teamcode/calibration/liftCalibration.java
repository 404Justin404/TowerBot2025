package org.firstinspires.ftc.teamcode.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subsystem.Lift;


@Config
@TeleOp(group = "Calibration")
public class liftCalibration extends LinearOpMode {

    public static double liftPower = 0.0;

    @Override
    public void runOpMode() {
        Lift lift = new Lift(this);

        waitForStart();

        while (opModeIsActive()) {
            lift.setPower(liftPower);


            telemetry.addData("Power", liftPower);
            telemetry.update();
        }
    }
}