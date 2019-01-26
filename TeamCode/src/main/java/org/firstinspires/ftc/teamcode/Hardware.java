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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    //public DcMotor  collectorArm     = null;
    //public DcMotor  collectorDrum    = null;
    public DcMotor  lift        = null;
    public DcMotor  bucketArm    = null;
    public CRServo  latch       = null;
    public Servo    bucketBox   = null;
    public DcMotor bucketArm2   = null;
    public DcMotor winch        = null;
    public DcMotor collector    = null;


    public static final double COLLECTOR_UP_POWER    =  1 ;
    public static final double COLLECTOR_DOWN_POWER  = -1 ;

    public static final double COLLECTOR_IN_POWER    =  0.75 ;
    public static final double COLLECTOR_OUT_POWER   =  -0.75 ;

    public static final double LIFT_UP_POWER    =  0.9 ;
    public static final double LIFT_DOWN_POWER  = -0.9 ;

    public static final double WINCH_UP_POWER   =  0.7;
    public static final double WINCH_DOWN_POWER   = -0.7;


    public static final double LATCH_OPEN_POWER  =  0.6 ;
    public static final double LATCH_CLOSE_POWER = -0.6 ;

    public static final double BUCKET_UP_POWER    =  0.8 ;
    public static final double BUCKET_DOWN_POWER  =  -0.4 ;

    public static final double BUCKET_DOOR_CLOSED =  0.55 ;
    public static final double BUCKET_DOOR_GOLD   =  0.88 ;
    public static final double BUCKET_DOOR_OPEN   =  1.0 ;

    //auto constants
    static final double ENCODER_CPR_60 = 1680;
    static final double ENCODER_CPR_40 = 1120;

    //auto lift constants
    static final double PINION_DIAMETER_Inches = 20.8/25.4;
    //static final double LIFT_HEIGHT = 8.875;
    static final double LIFT_HEIGHT = 9.4;
    static final double pinion_CPI = ENCODER_CPR_60/(PINION_DIAMETER_Inches*Math.PI);

    //auto collector constants
    static final double COLLECTOR_ANGLE = -90;

    //auto drive constants
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double WHEEL_CPI = ENCODER_CPR_40/(WHEEL_DIAMETER_INCHES*Math.PI);
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // vuforia constants
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public static final String VUFORIA_KEY = "ARYJT0b/////AAAAGYhN7cav+UUXqkMo7uS9Mswt0KxiQ3Sp/OVgoLfwHMP74uJpsnWLAXQLoXs0AIcpgC2IiJIov+JwDwrMwujShtlUastkjxWBAXLvJ6drxd811wEZGqBtBeOC6ObFPqG+W41u3D0fWJjsU4qG3S6NdgIAv6Q4T1OGH6Q6jOpatGlpEyhclM0Rk+vs77zaVzgBgZmcCa+tTqOpu0hhxqyxMvPv3Ehn0sgbF1KTfba/QQfxEjpsqJRyA5r7HfNNfg/31xdLLtzQXy28id0EXqPkB2iZ39fxsX0XcbKRWd7pq5uXqfvwJm4EvsKFLOz0eJhJBW+2vlCy5jrdehA7wH+pOnQTx3SQmbyqlr8KehWPWL1X";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        //collectorArm = hwMap.get(DcMotor.class, "collector_arm");
        //collectorDrum = hwMap.get(DcMotor.class, "collector_drum");
        lift = hwMap.get(DcMotor.class, "lift");
        bucketArm = hwMap.get(DcMotor.class, "bucket_arm");
        bucketArm2 = hwMap.get(DcMotor.class, "bucket_arm2");
        winch      = hwMap.get(DcMotor.class, "winch");
        collector  = hwMap.get(DcMotor.class, "collector");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        //collectorDrum.setDirection(DcMotor.Direction.REVERSE);

        //collectorArm.setDirection(DcMotor.Direction.REVERSE);
        //collectorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winch.setDirection(DcMotor.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketArm.setDirection(DcMotor.Direction.REVERSE);
        bucketArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketArm2.setDirection(DcMotor.Direction.FORWARD);
        bucketArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //collectorArm.setPower(0);
        //collectorDrum.setPower(0);
        lift.setPower(0);
        bucketArm.setPower(0);
        bucketArm2.setPower(0);
        winch.setPower(0);
        collector.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //collectorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //collectorDrum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucketArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        latch  = hwMap.get(CRServo.class, "latch");
        latch.setPower(0.0);
        bucketBox  = hwMap.get(Servo.class, "bucket_box");
        bucketBox.setDirection(Servo.Direction.REVERSE);
        bucketBox.setPosition(BUCKET_DOOR_CLOSED);

    }
    public void autoInit(HardwareMap otherHwMap) {
        init(otherHwMap);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //collectorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //collectorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //init vuforia localization engine

    public void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    //init Vuforia Tracking engine
    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.70;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


}

