/* Copyright (c) 2023 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="Team - Blue Left Auto", group = "Concept")
//@Disabled
public class TeamAutoDriveBlueAllianceLeft extends LinearOpMode
{
    private TfodProcessor tfod;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 500 ;    // eg: TETRIX Motor Encoder 1440
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    private static final String TFOD_MODEL_ASSET = "TeamPropAbs0.tflite";//"MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "TeamPropAbs0.tflite";//"/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            // "Pixel",
            "Abs0",
    };
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 0.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.05 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.008  ;   // 0.01  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {

        // initialize TFOD
        initTfod();

        // Initialize the Apriltag Detection process
        initAprilTag();



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor_fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor_fr");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motor_bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor_br");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //if (USE_WEBCAM)
          //  setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            //targetFound = false;
            desiredTag  = null;
            int obj_location = teamObjectDetectionTfod();
            teamAutoDrive(obj_location);
            //drive to April Tag
            driveToTeamAprilTag(obj_location);
            //sleep(1000);
            break;
        }
    }

    private void teamAutoDrive(int obj_location){

        //sleep(3000);
        // Apply desired axes motions to the drivetrain.
        //drive = 0.13;
        //moveRobot(drive, 0.0, 0.0);
        //moveForward(.13);
        // to to team object
        int team_object_position = obj_location; // 2 - Center, 1 left and 3 right
        float team_object_distance = 28;
        telemetry.addData("Auto - move to team object","Drive %5.2f inches ", team_object_distance);
        telemetry.update();
        //sleep(1000);
        encoderDrive(DRIVE_SPEED,  team_object_distance,  team_object_distance, 4.0);  // S1: Forward 24 Inches with 5 Sec timeout
        float turn_distance = 26;
        float forward_distance = 6;
        float reverse_distance = 10;
        // Decide what to do based on position
        // if center then put pixel next to team object, go back 2 inch and turn left
        // if left then turn left, move forward 2 inches, put pixel next to team object, move back 2 inches, move left 8 inches
        // if right then turn right, move forward 2 inches, put pixel next to team object, move back 2 inches, turn 180 degrees
        if (team_object_position == 2) {
            // if team object is in center
            encoderDrive(DRIVE_SPEED, forward_distance, forward_distance, 3.0);
            //move back and leave the pixel
            telemetry.addData("Auto - drop  team object", "Drive back %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, -reverse_distance, -reverse_distance, 3.0);  // S1: Forward 24 Inches with 5 Sec timeout
            // turn left towards the board
            telemetry.addData("Auto - turn left","turn left %5.2f inches ", turn_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(TURN_SPEED,   -turn_distance, turn_distance, 4.0);
        } else if (team_object_position == 1){
            // if team object position is left
            // turn left towards the board
            telemetry.addData("Auto - turn left","turn left %5.2f inches ", turn_distance);
            telemetry.update();
            moveParallelToRight(400);
            //sleep(400);
            //sleep(1000);
            encoderDrive(TURN_SPEED,   -turn_distance, turn_distance, 3.0);
            //move forward and drop the pixel
            telemetry.addData("Auto - drop  team object", "Drive forward %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, forward_distance+2, forward_distance+2, 3.0);
            //move back
            telemetry.addData("Auto - drive back", "Drive back %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, -reverse_distance+2, -reverse_distance+2, 2.0);
            //sleep(1000);
            moveParallelToLeft(1700);
            //sleep(500);
        } else if (team_object_position == 3){
            // if team object position is right
            // turn right towards the team object
            telemetry.addData("Auto - turn right","turn right %5.2f inches ", turn_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(TURN_SPEED,   turn_distance, -turn_distance, 3.0);
            //move forward and drop the pixel
            telemetry.addData("Auto - drop  team object", "Drive forward %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, forward_distance+1, forward_distance+1, 3.0);
            //move back
            telemetry.addData("Auto - drive back", "Drive back %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(DRIVE_SPEED, -reverse_distance-2, -reverse_distance-2, 3.0);
            // turn 180 degrees towards the team object
            telemetry.addData("Auto - turn 180 degrees","turn right %5.2f inches ", turn_distance);
            telemetry.update();
            //sleep(1000);
            encoderDrive(TURN_SPEED,   (turn_distance+1)*2, -(turn_distance+1)*2, 6.0);
        } else {
            telemetry.addData("Not able  to find object"," object position %d ", team_object_position);
            telemetry.update();
            sleep(5000);
            //moveParallelToLeft();
            //encoderDrive(DRIVE_SPEED,   10, 0, 6.0);
        }

        //drive to April Tag
        //driveToTeamAprilTag(team_object_position);

//            // turn left towards the board
//            float turn_distance = 26;
//            telemetry.addData("Auto - turn left","Drive back %5.2f inches ", turn_distance);
//            telemetry.update();
//            sleep(1000);
//            encoderDrive(TURN_SPEED,   -turn_distance, turn_distance, 3.0);



        //some additional experimentation
        //turnOff();
        //telemetry.addData("Auto - move back","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", .2, strafe, turn);
        //telemetry.update();
        //sleep(2000);
        //moveBackward(.05, 5);
        //sleep(3000);
        //moveLeft();
        //sleep(2000);
    }

    private void driveToTeamAprilTag(int team_object_position){

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        //set desired TAG ID depending on team object position
        DESIRED_TAG_ID = team_object_position;

        // drive towards April Tag - hardcoded distance
//        float april_tag_distance = 42;
//        telemetry.addData("Auto - drive to April Tag","Drive back %5.2f inches to %d ", april_tag_distance, DESIRED_TAG_ID);
//        telemetry.update();
//        sleep(1000);
//        encoderDrive(DRIVE_SPEED,  april_tag_distance,  april_tag_distance, 6.0);

        // drive using april tag code
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                telemetry.addData("Found Tag", "location %d", DESIRED_TAG_ID);
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData(">","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        //if (gamepad1.left_bumper && targetFound) {
        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        telemetry.update();
        //sleep(5000);
        moveRobot(drive, 0, 0);
        if (team_object_position == 1) {
            sleep(1700);
        } else  if (team_object_position == 3) {
            sleep(1600);
        } else {
            sleep(1600);
        }
        if (team_object_position == 1) {
            moveParallelToRight(1200);
            //sleep(1200);
        } else if (team_object_position == 3) {
            moveParallelToRight(1200);
            //sleep(1200);
        }
    }
    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int teamObjectDetectionTfod() {
        int location = -1; //  1 left and 2 - center and 3 - right
        double angle = 0;
        double x = -9999;
        double y = -9999;
        boolean found_pixel = false;
        String object_label = "";
        sleep(1000);
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.update();
        sleep(1000);

        // try again if we did not find object in first attempt
        if (currentRecognitions.size() == 0) {
            currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected on try 2", currentRecognitions.size());
            telemetry.update();
            sleep(1000);
        }
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            // double x = recognition.getLeft();
            //double y = recognition.getRight();
            angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
            object_label = recognition.getLabel();
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", object_label, recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Angle", ".0f", angle);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (object_label.equals("Abs0")){
                telemetry.addData("Found white pixel, breaking","%s", object_label);
                found_pixel = true;
                break;
            } else {
                telemetry.addData("Not Found white pixel, not breaking",  "%s", object_label);
            }

        }   // end for() loop
        if (found_pixel) {
            telemetry.addData(">","found pixel\n"); //enter 2 - Center, 1 left and 3 right
            if (x > 0 && x < 150){
                // if x between 150 to 350 assume it to be left
                location = 1;
            } else if (x > 150 && x < 350){
                // if x between 150 to 350 assume it to be center
                location = 2;
            } else  if (x > 400){
                // if x between 150 to 350 assume it to be right
                location = 3;
            }
            telemetry.addData("Team element is at location",  "%s", location);
            telemetry.update();
            sleep(2000);
        } // end of if
        telemetry.update();
        return location;
    }   // end method telemetryTfod()

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public void turnOff(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
    }


    public void moveForward(double power) {

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftBackDrive.setPower(power);
        double wait_time = 10000;
        while (wait_time > 0){
            int curr_power = leftFrontDrive.getCurrentPosition();
            int target_position = rightFrontDrive.getTargetPosition();
            telemetry.addData("current motor power is:", curr_power);
            telemetry.addData("current wait time is:", wait_time);
            telemetry.addData("target position is:", target_position);
            telemetry.update();
            wait_time = wait_time - 1000;
            sleep(1000);
        }
    }

    public void moveBackward(double power, long distance) {
        leftFrontDrive.setPower((-1) * power); //FL
        rightFrontDrive.setPower((-1) * power); //FR
        rightBackDrive.setPower((-1) * power); //BR
        leftBackDrive.setPower((-1) * power); //BL
    }

    public void moveLeft() {
        double power = .28;
        leftFrontDrive.setPower((-1)* power );
        rightFrontDrive.setPower((1) * power);
        rightBackDrive.setPower((1) * power);
        leftBackDrive.setPower((-1) *  power);
    }
    public void moveRight() {
        double power = .27;
        leftFrontDrive.setPower((1) * power);
        rightFrontDrive.setPower((-1) * power);
        rightBackDrive.setPower((-1)* power);
        leftBackDrive.setPower((1)* power);
    }

    public void moveParallelToLeft(int sl_sec) {
        double power = .20;
        leftFrontDrive.setPower((-1) * power);
        rightFrontDrive.setPower((1) * power);
        rightBackDrive.setPower((-1)* power);
        leftBackDrive.setPower((1)* power);
        sleep(sl_sec);
        turnOff();
    }

    public void moveParallelToRight(int sl_sec) {
        double power = .20;
        leftFrontDrive.setPower((1) * power);
        rightFrontDrive.setPower((-1) * power);
        rightBackDrive.setPower((1)* power);
        leftBackDrive.setPower((-1)* power);
        sleep(sl_sec);
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
            setManualExposure(11, 250);
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(aprilTag, tfod)
                    .build();
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)

                //set model file to trained model file
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();
// Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor_fl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor_fr");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "motor_bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "motor_br");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//            setManualExposure(6, 250);
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
//        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
  //      visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

//        telemetry.addData("Tryig to set exposure to",  "%7d :%7d",
//                exposureMS,
//                gain);
//        telemetry.update();
//        sleep(2000);

        if (visionPortal == null) {
            telemetry.addData("Vision Portal is null", "error");
            telemetry.update();
            sleep(500);
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Set exposure to",  "%7d :%7d",
                    exposureMS,
                    gain);
            telemetry.update();
        }
        sleep(200);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

//        leftFrontDrive.resetDeviceConfigurationForOpMode();
//        rightFrontDrive.resetDeviceConfigurationForOpMode();
//        leftBackDrive.resetDeviceConfigurationForOpMode();
//        rightBackDrive.resetDeviceConfigurationForOpMode();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();
        //sleep(250);
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
//            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = (int)(rightInches * COUNTS_PER_INCH);
            telemetry.addData("Heading to",  "%7d :%7d",
                    newLeftTarget,
                    newRightTarget);
            telemetry.update();
            //sleep(500);

            leftFrontDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
                //sleep(500);   // optional pause after each move.
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

}
