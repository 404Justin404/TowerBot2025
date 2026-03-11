package org.firstinspires.ftc.teamcode.subsystem;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.smartcluster.oracleftc.commands.Command;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


public class LimelightBallDetector {


    private Limelight3A limelight;
    private static final int BALL_PIPELINE = 3;
    private static final String GREEN_BALL = "green";
    private static final String PURPLE_BALL = "purple";



    public static class BallDetection {
        public final double tx;
        public final double ty;
        public final double targetArea;
        public final double estimatedDistance;
        public final double confidence;



        public BallDetection(double tx, double ty, double targetArea, double estimatedDistance, double confidence) {
            this.tx = tx;
            this.ty = ty;
            this.targetArea = targetArea;
            this.estimatedDistance = estimatedDistance;
            this.confidence = confidence;
        }

        @Override
        public String toString() {
            return String.format("Ball[tx=%.2f°, ty=%.2f°, dist=%.1f\", area=%.2f%%, conf=%.2f]",
                    tx, ty, estimatedDistance, targetArea, confidence);
        }
    }


    public LimelightBallDetector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(BALL_PIPELINE);
        limelight.start();
    }


    public List<BallDetection> getDetectedBalls() {
        List<BallDetection> balls = new ArrayList<>();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            for (LLResultTypes.DetectorResult detection : detectorResults) {
                if (detection.getClassName().equalsIgnoreCase(GREEN_BALL) || detection.getClassName().equalsIgnoreCase(PURPLE_BALL)) {
                    double tx = detection.getTargetXDegrees();
                    double ty = detection.getTargetYDegrees();
                    double area = detection.getTargetArea();
                    double confidence = detection.getConfidence();

                    double distance = estimateDistance(area);

                    balls.add(new BallDetection(tx, ty, area, distance, confidence));
                }
            }
        }

        return balls;
    }

    public BallDetection getClosestBall() {
        List<BallDetection> balls = getDetectedBalls();
        if (balls.isEmpty()) {
            return null;
        }

        return balls.stream()
                .min(Comparator.comparingDouble(b -> b.estimatedDistance))
                .orElse(null);
    }


    public boolean hasBallsDetected() {
        return !getDetectedBalls().isEmpty();
    }


    public int getBallCount() {
        return getDetectedBalls().size();
    }


    private double estimateDistance(double targetArea) {
        if (targetArea <= 0) {
            return Double.POSITIVE_INFINITY;
        }
        return  5/Math.sqrt(targetArea);
    }



    public LLResult getRawResult() {
        return limelight.getLatestResult();
    }


    public void stop() {
        limelight.stop();
    }



    public void updateTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        List<BallDetection> balls = getDetectedBalls();

        telemetry.addData("Balls Detected", balls.size());

        for (int i = 0; i < balls.size(); i++) {
            BallDetection ball = balls.get(i);
            telemetry.addData(String.format("Ball %d", i + 1), ball.toString());
        }

        BallDetection closest = getClosestBall();
        if (closest != null) {
            telemetry.addData("Closest Ball", String.format("%.1f\" away, %.2f° offset",
                    closest.estimatedDistance, closest.tx));
        }
    }


    public Command DriveToClosestBall(Follower follower, LimelightBallDetector detector, org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        final ElapsedTime timer = new ElapsedTime();
        final double interval = 500;

        return Command.builder()
                .init(() -> {
                    timer.reset();
                    LimelightBallDetector.BallDetection ball = detector.getClosestBall();
                    if (ball != null) {
                        catchMyBalls(follower, ball);
                    }
                })
                .update(() -> {
                    LimelightBallDetector.BallDetection ball = detector.getClosestBall();
                    if (ball != null) {
                        if (timer.milliseconds() >= interval) {
                            timer.reset();
                            catchMyBalls(follower, ball);
                        }
                        telemetry.addData("Driving to Ball", ball.toString());
                        telemetry.addData("Ball tx", ball.tx);
                        telemetry.addData("Ball dist (in)", ball.estimatedDistance);
                    } else {
                        telemetry.addData("Ball", "No longer visible — must be under camera");
                    }
                    follower.update();
                })
                .finished(() -> {
                    boolean noBalls = !detector.hasBallsDetected();
                    boolean pathDone = !follower.isBusy();
                    return noBalls || pathDone;
                })
                .build();
    }

    private void catchMyBalls(Follower follower, LimelightBallDetector.BallDetection ball) {
        Pose current = follower.getPose();
        double headingRad = current.getHeading();
        double distanceInches = ball.estimatedDistance;
        double lateralAngle = headingRad + Math.toRadians(ball.tx);

        double targetX = current.getX() + distanceInches * Math.cos(lateralAngle);
        double targetY = current.getY() + distanceInches * Math.sin(lateralAngle);

        Pose targetPose = new Pose(targetX, targetY, lateralAngle);

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(current, targetPose))
                .setLinearHeadingInterpolation(headingRad, lateralAngle)
                .build();
        follower.setMaxPower(1);
        follower.followPath(path, false);
    }

}