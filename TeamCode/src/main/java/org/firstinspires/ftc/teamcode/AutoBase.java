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

@Autonomous(name="Auto Base", group="Autonomous")
@Disabled
public class AutoBase extends LinearOpMode {

    //OpMode Members
    static final double lowerLift = Hardware.LIFT_HEIGHT*Hardware.pinion_CPI;
    static final double lowerCollector = ((Hardware.ENCODER_CPR_60/(360/Hardware.COLLECTOR_ANGLE))/3)*2;
    Boolean crater = true;

    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime latchTime = new ElapsedTime();
    private ElapsedTime motorTime = new ElapsedTime();

    Hardware robot = new Hardware();
    @Override
    public void runOpMode() {
        robot.autoInit(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        latchTime.reset();
        motorTime.reset();



        // run until the end of the match (driver presses STOP)
        if (opModeIsActive())
            land();

        motorTime.reset();
        if(crater == true) {
            while (opModeIsActive() &&(motorTime.seconds() < 4.)) {
                robot.leftDrive.setPower(.4);
                robot.rightDrive.setPower(.4);
            }
            robot.leftDrive.setPower(0.);
            robot.rightDrive.setPower(0.);
         //this is depo
        } else if(crater == false) {
            while (opModeIsActive() && (motorTime.seconds() < 4.)) {
                robot.leftDrive.setPower(.4);
                robot.rightDrive.setPower(.4);
            }
            robot.leftDrive.setPower(0.);
            robot.rightDrive.setPower(0.);

            robot.collectorDrum.setPower(.4);
            sleep(500);
            robot.collectorDrum.setPower(0.);




        }

    }
    void land(){
        robot.collectorArm.setTargetPosition((int)lowerCollector);
        robot.collectorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.collectorArm.setPower(0.2);
        robot.lift.setTargetPosition((int)lowerLift);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.5);
        while (opModeIsActive() && robot.lift.isBusy()) {}
        robot.lift.setPower(0.0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latchTime.reset();
        while (opModeIsActive() && latchTime.seconds() < 0.2)  {
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setPower(-0.32);
            robot.leftDrive.setPower(0.32);
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
            robot.leftDrive.setPower(0.3);
            robot.rightDrive.setPower(0.3);
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

        robot.lift.setTargetPosition((int)Hardware.pinion_CPI);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.5);
        while (opModeIsActive() && robot.lift.isBusy()) {}
        robot.lift.setPower(0.0);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
