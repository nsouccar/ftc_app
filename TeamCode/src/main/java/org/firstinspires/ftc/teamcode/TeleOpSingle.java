/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TeleOpSingle", group="TechnoChix")

public class TeleOpSingle extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private static long X_DEBOUNCE = 20;
    long lastX;
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
        lastX = runtime.now(TimeUnit.MILLISECONDS);

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
                robot.latch.setPower(0.);

            // Use gamepad buttons to move collector arm up (Y) and down (A)
            if (gamepad1.y)
                robot.collectorArm.setPower(robot.COLLECTOR_UP_POWER);
            else if (gamepad1.a)
                robot.collectorArm.setPower(robot.COLLECTOR_DOWN_POWER);
            else
                robot.collectorArm.setPower(0.0);


            // Use gamepad buttons to open/close scoop door in (X) and out (B)
            if (gamepad2.right_bumper)
                robot.bucketBox.setPosition(robot.BUCKET_DOOR_OPEN);
            else if (gamepad2.left_bumper)
                robot.bucketBox.setPosition(robot.BUCKET_DOOR_CLOSED);
            else if (gamepad2.y)
                robot.bucketBox.setPosition(robot.BUCKET_DOOR_GOLD);

            // Use gamepad triggers to spin collector drum in (left_trigger) and out (right_trigger)
            if (gamepad1.left_trigger > 0.05)
                robot.collectorDrum.setPower(-gamepad1.left_trigger);
            else if (gamepad1.right_trigger > 0.05)
                robot.collectorDrum.setPower(gamepad1.right_trigger);
            else
                robot.collectorDrum.setPower(0.0);

            // Use gamepad buttons to move lift up (dpad_up) and down (dpad_down)
            if (gamepad1.dpad_up)
                robot.lift.setPower(robot.LIFT_UP_POWER);
            else if (gamepad1.dpad_down)
                robot.lift.setPower(robot.LIFT_DOWN_POWER);
            else
                robot.lift.setPower(0.0);

            // Use gamepad buttons to move scoop up (dpad_right) and down (dpad_left)
            if (gamepad1.dpad_right)
                robot.bucketArm.setPower(robot.BUCKET_UP_POWER);
            else if (gamepad1.dpad_left)
                robot.bucketArm.setPower(robot.BUCKET_DOWN_POWER);
            else
                robot.bucketArm.setPower(0.0);

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so hook action is reasonable speed.
            sleep(50);
        }
    }
}
