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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math.*;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name="Auto Base", group="Autonomous")
@Disabled
public class AutoBase extends LinearOpMode {

    //OpMode Members
    static final double lowerLift = Hardware.LIFT_HEIGHT * Hardware.pinion_CPI;
    static final double lowerCollector = ((Hardware.ENCODER_CPR_60 / (360 / Hardware.COLLECTOR_ANGLE)) / 3) * 2;
    Boolean crater = true;

    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime latchTime = new ElapsedTime();
    private ElapsedTime motorTime = new ElapsedTime();


    Hardware robot = new Hardware();
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    int color = 0;


    @Override
    public void runOpMode() {
        robot.autoInit(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            robot.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        latchTime.reset();
        motorTime.reset();


        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //land();

            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runtime.reset();
            //moveThatRobot(.5, 7, 7, 4);
            //moveThatRobot(.5, 7, 7, 8);

            //Sample();
            //moveThatRobot(.5, -5, -5, 4);
            //telemetry.addData("move", 1);
            //telemetry.update();

            //rotate(90, .25, 18);
            //telemetry.addData("rotate", 1);
            //moveThatRobot(.5, -5, -5, 4);
            //telemetry.addData("moveelse", 2);


            //robot.leftDrive.setPower(0);
            //robot.rightDrive.setPower(0);

            //rotate(112, .5, 3);
            land();
            runtime.reset();
            rotate(630 , 1, 3);
            Sample();
            //findLine();

        }


        /*motorTime.reset();
        if(crater true) {
            while (opModeIsActive() &&(motorTime.seconds() < 6.)) {
                robot.leftDrive.setPower(.4);
                robot.rightDrive.setPower(.4);
            }
            robot.leftDrive.setPower(0.);
            robot.rightDrive.setPower(0.);
         //this is depo
        } else if(crater == false) {
            while (opModeIsActive() && (motorTime.seconds() < 6.)) {
                robot.leftDrive.setPower(.4);
                robot.rightDrive.setPower(.4);
            }
            robot.leftDrive.setPower(0.);
            robot.rightDrive.setPower(0.);

            robot.collectorDrum.setPower(.4);
            sleep(500);
            robot.collectorDrum.setPower(0.);

*/


    }


    void land() {
        //robot.collectorArm.setTargetPosition((int) lowerCollector);
        //robot.collectorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.collectorArm.setPower(0.2);
        robot.lift.setTargetPosition((int) lowerLift);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.5);
        while (opModeIsActive() && robot.lift.isBusy()) {
        }
        robot.lift.setPower(0.0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latchTime.reset();
        while (opModeIsActive() && latchTime.seconds() < 0.25) {
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setPower(-0.5);
            robot.leftDrive.setPower(0.5);
        }

        robot.rightDrive.setPower(0.0);
        robot.leftDrive.setPower(0.0);

        latchTime.reset();
        while (opModeIsActive() && (latchTime.seconds() < 2.35)) {
            telemetry.addData("Latch Time:", latchTime.seconds());
            telemetry.update();
            robot.latch.setPower(Hardware.LATCH_OPEN_POWER);
        }
        robot.latch.setPower(0.0);

        latchTime.reset();
        while (opModeIsActive() && (latchTime.seconds() < 2)) {
            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(0.5);
        }
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);

/*        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setTargetPosition(55*(int)Hardware.WHEEL_CPI);
        robot.leftDrive.setTargetPosition(55*(int)Hardware.WHEEL_CPI);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setPower(0.3);
        robot.leftDrive.setPower(0.3);
        while (opModeIsActive() && robot.leftDrive.isBusy() && robot.rightDrive.isBusy()) {
            telemetry.addData("Encoder Right:", robot.rightDrive.getCurrentPosition());
            telemetry.addData("Encoder Left:", robot.leftDrive.getCurrentPosition());
            telemetry.update();

        }
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        robot.lift.setTargetPosition((int) Hardware.pinion_CPI);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.5);
        while (opModeIsActive() && robot.lift.isBusy()) {
        }
        robot.lift.setPower(0.0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveThatRobot(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        runtime.reset();
        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("update:", "about to turn on motors");
        telemetry.update();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }


        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void rotate(double degrees, double speed, double timeoutS) {
        //arc length/2pir = degrees
        //degrees/2pir = arc length
        double circumference = 2 * Math.PI * 6.625;
        double inches = degrees / circumference;

        runtime.reset();
        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);


            robot.leftDrive.setTargetPosition(-newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("update:", "about to turn on motors");
        telemetry.update();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy()) && robot.rightDrive.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();

        }
            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }


        public Integer Sample () {
        int position = 1;

            if (opModeIsActive()) {
                /** Activate Tensor Flow Object Detection. */
                if (robot.tfod != null) {
                    robot.tfod.activate();
                }

                while (opModeIsActive()) {
                    if (robot.tfod != null) {
                        telemetry.addData("fuck ya", "ha");
                        telemetry.update();

                        List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();

                        if (updatedRecognitions != null && updatedRecognitions.size() == 1) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL) == true) {
                                    moveThatRobot(1, -17, -17, 4);
                                    position = 1;

                                } else {
                                    rotate(135, .5, 4);
                                    List<Recognition> updatedRecognitions2 = robot.tfod.getUpdatedRecognitions();
                                    if (updatedRecognitions2 != null && updatedRecognitions2.size() == 1) {
                                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                                        for (Recognition recognition2 : updatedRecognitions2) {

                                            if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL) == true) {
                                                telemetry.addData("status", "turning");
                                                telemetry.update();

                                                moveThatRobot(.5, -17, -17, 6);
                                                position = 2;


                                            } else {
                                                telemetry.addData("hi", "hi");
                                                telemetry.update();
                                                rotate(-111, .5, 25);
                                                moveThatRobot(.5, -25, -25, 6);
                                                position = 3;


                                            }
                                        }


                                    }

                                }


                                //findGold(tfod.getUpdatedRecognitions(), 45, hardware);

                            }
                            telemetry.update();
                        }


                    }

                }



            }

                return position;


        }




    public String getColor () {
        String colorString = "None";
        //setting number values for each color
        if (hsvValues[0]<=360 && hsvValues[0]>=275) {
            color = 1;
            colorString = "Red";
            telemetry.addData("Color:", "Red");
        }
        else if (hsvValues[0]>=175 && hsvValues[0]<=250) {
            color = 2;
            colorString = "Blue";
            telemetry.addData("Color:", "Blue");
        }
        else if (hsvValues[0]<=125 && hsvValues[0]>=60) {
            color = 3;
            colorString = "Black";

            telemetry.addData("Color:", "Black");
        }
        else {
            color = 0;
            colorString = "None";

            telemetry.addData("Color:", "No color detected.");
        }
        return colorString;

    }


    }















