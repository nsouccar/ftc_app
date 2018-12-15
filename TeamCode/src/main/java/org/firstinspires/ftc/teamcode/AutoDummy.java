package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Dummy", group = "Autonomous")
public class AutoDummy extends LinearOpMode {
    private ElapsedTime testTime = new ElapsedTime();

    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            testTime.reset();
            while (opModeIsActive() && (testTime.seconds() < 5)) {
                telemetry.addData("Time:", testTime.seconds());
                telemetry.update();
            }
            sleep(5000);
            telemetry.addData("Time:", testTime.seconds());
            telemetry.update();
        }
    }
}
