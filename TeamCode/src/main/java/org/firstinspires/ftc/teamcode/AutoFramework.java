package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import java.util.List;


@Autonomous(name="Autonomous", group="Pushbot")


public class AutoFramework extends LinearOpMode {
    ToolBox hardware;
    ElapsedTime runtime;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ARYJT0b/////AAAAGYhN7cav+UUXqkMo7uS9Mswt0KxiQ3Sp/OVgoLfwHMP74uJpsnWLAXQLoXs0AIcpgC2IiJIov+JwDwrMwujShtlUastkjxWBAXLvJ6drxd811wEZGqBtBeOC6ObFPqG+W41u3D0fWJjsU4qG3S6NdgIAv6Q4T1OGH6Q6jOpatGlpEyhclM0Rk+vs77zaVzgBgZmcCa+tTqOpu0hhxqyxMvPv3Ehn0sgbF1KTfba/QQfxEjpsqJRyA5r7HfNNfg/31xdLLtzQXy28id0EXqPkB2iZ39fxsX0XcbKRWd7pq5uXqfvwJm4EvsKFLOz0eJhJBW+2vlCy5jrdehA7wH+pOnQTx3SQmbyqlr8KehWPWL1X";

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.70;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        ToolBox hardware = new ToolBox();
        ElapsedTime runtime = new ElapsedTime();
        hardware.init(hardwareMap);

        runtime.reset();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    telemetry.addData("fuck ya", "ha");
                    telemetry.update();

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    /*while(updatedRecognitions == null && updatedRecognitions.size() != 1){
                        hardware.rotate(45, .25);
                        hardware.rotate(-45, .25);


                    } while(updatedRecognitions.size() != 1) {
                        hardware.rotate(45, .25);
                        hardware.rotate(-45, .25);
                    }
                    while

                    */
                    if (updatedRecognitions != null && updatedRecognitions.size() == 1) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("fuck ya", recognition.getLabel());
                            telemetry.update();
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) == true) {
                                telemetry.addData("fuck ya", recognition.getLabel());
                                telemetry.update();
                                hardware.encoderDrive(1, -5, -5, 1);

                            } else {
                                hardware.rotate(35, .25);
                                hardware.rightDrive.setPower(0.0);
                                hardware.leftDrive.setPower(0.0);
                                sleep(100);

                                List<Recognition> updatedRecognitions2 = tfod.getUpdatedRecognitions();

                                if (updatedRecognitions2 != null && updatedRecognitions2.size() == 1) {
                                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                                    for (Recognition recognition2 : updatedRecognitions2) {
                                        telemetry.addData("fuck ya", recognition.getLabel());
                                        telemetry.update();
                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) == true) {
                                            telemetry.addData("fuck ya", recognition.getLabel());
                                            telemetry.update();
                                            hardware.encoderDrive(1, -6, -6, 1);
                                            hardware.encoderDrive(1, 1, 1, 1);

                                            hardware.rotate(45, .5);
                                            hardware.encoderDrive(1, -15, -15, 1);

                                        } else {
                                            hardware.rotate(-90, .25);
                                            hardware.encoderDrive(1, -6, -6, 1);
                                            hardware.rotate(45, .5);
                                            hardware.encoderDrive(1, -15, -15, 1);

                                            hardware.rightDrive.setPower(0.0);
                                            hardware.leftDrive.setPower(0.0);


                                        }

                                        telemetry.addData("fuck eys bitches", "h");
                                    }

                                    //findGold(tfod.getUpdatedRecognitions(), 45, hardware);


                                    telemetry.update();
                                }
                            }

                        }

                    }

                }

            }

        }

    }

}




















