package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Rotatev", group="Pushbot")

public class AutoFramework extends LinearOpMode {
    ToolBox hardware;




    @Override
    public void runOpMode() throws InterruptedException {
        ToolBox hardware = new ToolBox();
        hardware.init(hardwareMap);

        waitForStart();



        while (opModeIsActive()){
            hardware.rotate(99, .5);
        }

    }

}
