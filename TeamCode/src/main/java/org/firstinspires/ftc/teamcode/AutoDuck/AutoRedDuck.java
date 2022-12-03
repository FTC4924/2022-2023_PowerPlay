package org.firstinspires.ftc.teamcode.AutoDuck;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AllianceColor;

@Disabled
@Autonomous(name="RedDuck")
public class AutoRedDuck extends AutoDuck {
    @Override
    protected AllianceColor getAllianceColor()  {
        return AllianceColor.RED;
    }
}
