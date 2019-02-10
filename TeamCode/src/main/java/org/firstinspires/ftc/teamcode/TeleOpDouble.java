package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpDouble", group="TechnoChix")

public class TeleOpDouble extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() {
        double left;
        double right;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            // Use gamepad trigger to move lift up and down
            if (gamepad1.right_trigger > 0.05)
                robot.lift.setPower(gamepad1.right_trigger);
            else if (gamepad1.left_trigger > 0.05)
                robot.lift.setPower(-gamepad1.left_trigger);
            else
                robot.lift.setPower(0.0);

            // Use gamepad buttons to move collector arm up (b) and down (x)
            /*if (gamepad2.b)
                robot.collectorArm.setPower(robot.COLLECTOR_UP_POWER);
            else if (gamepad2.x)
                robot.collectorArm.setPower(robot.COLLECTOR_DOWN_POWER);
            else
                robot.collectorArm.setPower(0.0);
*/
            // Use gamepad triggers to spin collector drum in (right_trigger) and out (left_trigger)
            /*if (gamepad2.right_trigger > 0.05)
                robot.collectorDrum.setPower(-gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.05)
                robot.collectorDrum.setPower(gamepad2.left_trigger);
            else
                robot.collectorDrum.setPower(0.0);
*/
            // Use gamepad right stick to move bucket up and down
            if (gamepad2.dpad_up) {
                robot.bucket.setPower(1);

            } else if (gamepad2.dpad_down) {
                robot.bucket.setPower(1);

            } else {
                robot.bucket.setPower(0);

            }

            if(gamepad2.a) {
                robot.transport.setPosition(1);
            } else {
                robot.transport.setPosition(0);
            }

            // Use gamepad bumpers to open(right) and close(left) bucket door and gold(y)

            if (gamepad2.right_trigger > 0.05) {
                robot.slide.setPower(1);
                robot.transport.setPosition(1);
            } else if (gamepad2.left_trigger > 0.05) {
                robot.slide.setPower(-.4);
                robot.transport.setPosition(0);
            }else {
                robot.slide.setPower(0.0);
            }
        //else if (gamepad2.y)
            //  robot.bucketBox.setPosition(robot.BUCKET_DOOR_GOLD);

            if (gamepad2.right_bumper) {
                robot.collector.setPower(-1);
            } else if (gamepad2.left_bumper) {
                robot.collector.setPower(1);
            } else {
                robot.collector.setPower(0);

            }

            if(gamepad2.b){
                robot.transport.setPosition(.5);
                encoderDrive(.8, 5, 4);
            }




            // Send telemetry message to signify robot running;


            // Pace this loop so hook action is reasonable speed.
            sleep(50);
        }
    }

    public void encoderDrive(double speed,
                             double leftInches,
                             double timeoutS) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
           target = robot.slide.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.slide.setTargetPosition(target);


            // Turn On RUN_TO_POSITION
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.slide.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.slide.isBusy() && robot.slide.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", target);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.slide.getCurrentPosition()
                      );
                telemetry.update();
            }

            // Stop all motion;
            robot.slide.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

}


