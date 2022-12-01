package org.firstinspires.ftc.teamcode.AutoDuck;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;

@Autonomous(name="RedDuck")
public class AutoRedDuck extends AutoDuck {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
