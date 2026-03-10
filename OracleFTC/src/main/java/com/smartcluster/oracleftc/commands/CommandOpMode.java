package com.smartcluster.oracleftc.commands;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.smartcluster.oracleftc.commands.CommandScheduler;
import com.smartcluster.oracleftc.hardware.subsystem.Subsystem;

/**
 * As opposed to the general WPILib-style Robot paradigm, SolversLib also offers a command opmode
 * for individual opmodes.
 *
 * @author Jackson
 */
public abstract class CommandOpMode extends LinearOpMode {
    public CommandScheduler scheduler;

    protected final Pose startPose = new Pose(80.42992623814541, 34.80189673340359, Math.toRadians(-90));
    protected final Pose shootPose = new Pose(85.74077976817703, 14.908324552160169, Math.toRadians(-120));
    protected final Pose firstStack = new Pose(30, 121, Math.toRadians(0));
    protected final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    protected final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    protected final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));


}
