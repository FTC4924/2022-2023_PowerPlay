package com.team4924.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepInstall {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(/*45.5148*/ 25, 73.17330064499293, Math.toRadians(343.4181), Math.toRadians(262.03258124999996), 9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(90)))  // Write Path Here
                                .strafeLeft(3)
                                .forward(30)

                                /*.splineToConstantHeading(new Vector2d(-35,-45), Math.toRadians(90))
                                //down .splineToConstantHeading(new Vector2d(-25,-60), 0)
                                //top .splineToConstantHeading(new Vector2d(-25,-10), 0)
                               .splineToConstantHeading(new Vector2d(-35,-20),Math.toRadians(90))*/

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}