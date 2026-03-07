package com.example.meepmeep;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        final Pose2d startPose = new Pose2d(13,-59, Math.toRadians(90));
        final Vector2d shootPose = new Vector2d(15,-55);
        final Pose2d stack1 = new Pose2d(30,-33.5,Math.toRadians(180));
        final Pose2d stack2 = new Pose2d(30,-9.5,Math.toRadians(180));
        final Pose2d gate = new Pose2d(57,0,Math.toRadians(90));
        final Pose2d rotation = new Pose2d(28.6,-55,Math.toRadians(180));
        final Pose2d down = new Pose2d(56.7,-55,Math.toRadians(180));//67
        final Pose2d stack3 = new Pose2d(28,13.5,Math.toRadians(180));
        final Pose2d endPose = new Pose2d(35, -56, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(  300/25.4, 456/25.4)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(shootPose,70),Math.toRadians(70))
                        .setTangent(Math.toRadians(60))
                        .splineToLinearHeading(stack1, Math.toRadians(0))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(55,-33.5,Math.toRadians(180)), 0)
                        .setTangent(Math.toRadians(200))
                        .splineToLinearHeading(new Pose2d(shootPose,Math.toRadians(60)), Math.toRadians(220))
                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(stack2, Math.toRadians(0))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(55,-9.5,Math.toRadians(180)), 0)
                        .setTangent(Math.toRadians(100))
                        .splineToLinearHeading(gate, Math.toRadians(180))//slow
                        .setTangent(Math.toRadians(-160))
                        .splineToLinearHeading(new Pose2d(shootPose,Math.toRadians(60)),Math.toRadians(220))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(rotation, 0)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(down, 0)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(shootPose,Math.toRadians(60)),Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(rotation, 0)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(down, 0)
                        .build());

        Image img = null;
        try { img= ImageIO.read(new File("J:/chestii/imagine.meepmeep/field-2025-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}