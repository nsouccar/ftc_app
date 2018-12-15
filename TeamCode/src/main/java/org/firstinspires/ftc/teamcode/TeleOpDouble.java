package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpDouble", group="TechnoChix")

public class TeleOpDouble extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot           = new Hardware();

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the latch
            if (gamepad1.right_bumper)
                robot.latch.setPower(robot.LATCH_OPEN_POWER);
            else if (gamepad1.left_bumper)
                robot.latch.setPower(robot.LATCH_CLOSE_POWER);
            else
                robot.latch.setPower(0.0);

            // Use gamepad dpad to move lift up (dpad_up) and down (dpad_down)
            if (gamepad2.dpad_up)
                robot.lift.setPower(robot.LIFT_UP_POWER);
            else if (gamepad2.dpad_down)
                robot.lift.setPower(robot.LIFT_DOWN_POWER);
            else
                robot.lift.setPower(0.0);

            // Use gamepad buttons to move collector arm up (b) and down (x)
            if (gamepad2.b)
                robot.collectorArm.setPower(robot.COLLECTOR_UP_POWER);
            else if (gamepad2.x)
                robot.collectorArm.setPower(robot.COLLECTOR_DOWN_POWER);
            else
                robot.collectorArm.setPower(0.0);

            // Use gamepad triggers to spin collector drum in (right_trigger) and out (left_trigger)
            if (gamepad2.right_trigger > 0.05)
                robot.collectorDrum.setPower(gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.05)
                robot.collectorDrum.setPower(-gamepad2.left_trigger);
            else
                robot.collectorDrum.setPower(0.0);

            // Use gamepad right stick to move bucket up and down
            double up = -gamepad2.right_stick_y;
            robot.bucketArm.setPower(0.5*up);

            // Use gamepad bumpers to open(right) and close(left) bucket door and gold(y)
            if (gamepad2.right_bumper)
                robot.bucketBox.setPosition(robot.BUCKET_DOOR_OPEN);
            else if (gamepad2.left_bumper)
                robot.bucketBox.setPosition(robot.BUCKET_DOOR_CLOSED);
            else if (gamepad2.y)
                robot.bucketBox.setPosition(robot.BUCKET_DOOR_GOLD);



            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so hook action is reasonable speed.
            sleep(50);
        }
    }
}
