package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

@TeleOp(group = "TeleOp")
public class RedDistTeleOp extends DistTeleOp{
    {
//        cornerCoordinate = new Pose2d(62,62, Math.toRadians(-45));
        cornerCoordinate = new Pose2d(60,63, Math.toRadians(-45));
        MecanumDrive.resetPose = new com.smartcluster.oracleftc.math.Pose2d(-64.3, -61, Math.toRadians(-90));


        isRed=true;
    }
}
