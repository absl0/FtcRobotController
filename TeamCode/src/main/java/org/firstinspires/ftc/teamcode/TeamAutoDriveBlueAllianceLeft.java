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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    private TeamAutoDrive tad ; //= new TeamAutoDrive(hardwareMap, telemetry);;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
    private static final String TFOD_MODEL_FILE = "TeamPropAbs0.tflite";//"/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {
        tad = new TeamAutoDrive(hardwareMap, telemetry, gamepad1, TFOD_MODEL_FILE);
        // Initialize the robot motors and settings
        // tad.initRobotSettings();
        //Robot robot = new Robot(hardwareMap, telemetry);
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
            int obj_location = 0;
            try {
                obj_location = tad.teamObjectDetectionTfod();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            //drive to team object
            driveToTeamObject(obj_location);
            //sleep for some time to give camera time to detect april tag
            sleep(1000);
            //drive to April Tag
            //obj_location = 2; // added temp code to test manually
            tad.driveToTeamAprilTag(obj_location);
            //sleep(1000);
            tad.dropPixel();
            //tad.robotInstance.rotateAntiClock(90, .60);
            break;
        }
    }

    private void driveToTeamObject(int obj_location){

        //sleep(3000);
        // Apply desired axes motions to the drivetrain.
        //drive = 0.13;
        //moveRobot(drive, 0.0, 0.0);
        //moveForward(.13);
        // to to team object
        int team_object_position = obj_location; // 2 - Center, 1 left and 3 right
        float team_object_distance = 28;
        telemetry.addData("Auto - move to team object","Drive %5.2f inches and object location %5d",
                team_object_distance, obj_location);
        telemetry.update();
        sleep(1000);
        tad.driveRobot(DRIVE_SPEED,  team_object_distance,  team_object_distance, 4.0);  // S1: Forward 24 Inches with 5 Sec timeout
        float turn_distance = 26;
        float forward_distance = 6;
        float reverse_distance = 10;
        int new_object_position = -1;
        if (obj_location == -1) {
            // if object not found at start then try again
            try {
                new_object_position = tad.teamObjectDetectionTfod();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            if (new_object_position != -1) {
                // object found at center place
                team_object_position = 2;
            } else {
                // even after getting close not found object then turn right and try to find object again
                tad.moveParallelToLeft(400);
                //turn right
                tad.driveRobot(TURN_SPEED, turn_distance, -turn_distance, 3.0);
                // if object not found then try again
                try {
                    new_object_position = tad.teamObjectDetectionTfod();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                if (new_object_position != -1) {
                    // object found at right place
                    team_object_position = 3;
                } else {
                    // even after getting close and turning right object not found found then turn 180 degrees and try to find object again
                    // it might be on left side
                    // if object not found then try again after turning 180 degrees
                    //tad.driveRobot(TURN_SPEED, turn_distance, -turn_distance, 3.0);
                    //tad.driveRobot(TURN_SPEED, turn_distance, -turn_distance, 3.0);
                    tad.driveRobot(TURN_SPEED, (turn_distance + 1) * 2, -(turn_distance + 1) * 2, 6.0);
                    try {
                        new_object_position = tad.teamObjectDetectionTfod();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    if (new_object_position != -1) {
                        team_object_position = 1;
                    } else {
                        telemetry.addData("Not able  to find object", " even from close position");
                        telemetry.update();
                        sleep(1000);
                    }
                }
            }
        }
        // Decide what to do based on position
        // if center then put pixel next to team object, go back 2 inch and turn left
        // if left then turn left, move forward 2 inches, put pixel next to team object, move back 2 inches, move left 8 inches
        // if right then turn right, move forward 2 inches, put pixel next to team object, move back 2 inches, turn 180 degrees
        if (team_object_position == 2) {
            // if team object is in center
            tad.driveRobot(DRIVE_SPEED, forward_distance+2, forward_distance+2, 3.0);
            //move back and leave the pixel
            telemetry.addData("Auto - drop  team object", "Drive back %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            tad.driveRobot(DRIVE_SPEED, -reverse_distance, -reverse_distance, 5.0);  // S1: Forward 24 Inches with 5 Sec timeout
            // turn left towards the board
            telemetry.addData("Auto - turn left","turn left %5.2f inches ", turn_distance);
            telemetry.update();
            //sleep(1000);
            tad.driveRobot(TURN_SPEED,   -turn_distance, turn_distance, 4.5);
        } else if (team_object_position == 1){
            // if team object position is left
            // turn left towards the board
            if (obj_location != -1) {
                telemetry.addData("Auto - turn left", "turn left %5.2f inches ", turn_distance);
                telemetry.update();
                tad.moveParallelToRight(400);
                //sleep(400);
                //sleep(1000);
                tad.driveRobot(TURN_SPEED, -turn_distance, turn_distance, 3.0);
            }
            //move forward and drop the pixel
            telemetry.addData("Auto - drop  team object", "Drive forward %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            tad.driveRobot(DRIVE_SPEED, forward_distance+4, forward_distance+4, 4.0);
            //move back
            telemetry.addData("Auto - drive back", "Drive back %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            tad.driveRobot(DRIVE_SPEED, -reverse_distance+2, -reverse_distance+2, 2.0);
            //sleep(1000);
            tad.moveParallelToLeft(1200);
            //sleep(1700);
        } else if (team_object_position == 3){
            if (obj_location != -1) {
                // if team object position is right and was found at start
                // turn right towards the team object
                telemetry.addData("Auto - turn right", "turn right %5.2f inches ", turn_distance);
                telemetry.update();
                // move little left before turning right
                tad.moveParallelToLeft(400);
                //sleep(1000);
                //turn right
                tad.driveRobot(TURN_SPEED, turn_distance, -turn_distance, 3.0);
            }
            //move forward and drop the pixel
            telemetry.addData("Auto - drop  team object", "Drive forward %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            tad.driveRobot(DRIVE_SPEED, forward_distance+1.5, forward_distance+1.5, 3.0);
            //move back
            telemetry.addData("Auto - drive back", "Drive back %5.2f inches ", reverse_distance);
            telemetry.update();
            //sleep(1000);
            tad.driveRobot(DRIVE_SPEED, -reverse_distance-2, -reverse_distance-2, 3.0);
            // turn 180 degrees towards the team object
            telemetry.addData("Auto - turn 180 degrees","turn right %5.2f inches ", turn_distance);
            telemetry.update();
            //sleep(1000);
            //tad.driveRobot(TURN_SPEED, turn_distance, -turn_distance, 3.0);
            //tad.driveRobot(TURN_SPEED, turn_distance, -turn_distance, 3.0);
            tad.driveRobot(TURN_SPEED,   (turn_distance+1)*2, -(turn_distance+1)*2, 6.0);
        } else {
            telemetry.addData("Not able  to find object"," object position %d ", team_object_position);
            telemetry.update();
            sleep(1000);
            //moveParallelToLeft();
            //driveRobot(DRIVE_SPEED,   10, 0, 6.0);
        }
    }
}
