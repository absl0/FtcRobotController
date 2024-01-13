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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This OpMode executes a Linear Actuator (LA) control TeleOp
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the Left Bumper button expands LinearActuator
 * and Right Bumper button contracts LinearActuator.
 */

@Disabled
@TeleOp(name="Robot: Teleop Linear Actuator", group="Robot")
public class LinearActuator extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  MotorLA   = null; // Linear Actuator Motor

    public static final double LA_EXPAND_POWER    =  0.80 ; // Run actuator motor up at 80% power
    public static final double LA_CONTRACT_POWER  = -0.80 ; // Run actuator motor down at -80% power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        MotorLA = hardwareMap.get(DcMotor.class, "LinearActuator");

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Use gamepad buttons to move the linear actuator
        // expand (Left Bumper) and contracts (Right Bumper)
        if (gamepad1.left_bumper) {
            MotorLA.setPower(LA_EXPAND_POWER);
            telemetry.addData("Linear Actuator: ", "Expanding");
        } else if (gamepad1.right_bumper) {
            MotorLA.setPower(LA_CONTRACT_POWER);
            telemetry.addData("Linear Actuator: ", "Contracting");
        } else {
            MotorLA.setPower(0.0);
            telemetry.addData("Linear Actuator: ", "Still");
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

