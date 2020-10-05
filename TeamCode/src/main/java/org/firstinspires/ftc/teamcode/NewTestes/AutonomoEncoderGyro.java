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

package org.firstinspires.ftc.teamcode.NewTestes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Autônomo Encoder Teste", group="Pushbot")
@Disabled
public class AutonomoEncoderGyro extends LinearOpMode {

    HardwareClass  robot   = new HardwareClass();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    Orientation angles;


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED            = 0.6;
    static final double     TURN_SPEED              = 0.5;
    double speedEsquerda;
    double speedDireita;
    @Override
    public void runOpMode() {
        robot.hardwareGeral(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.motorEsquerda.getCurrentPosition(),
                          robot.motorDireita.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        speedEsquerda = speed;
        speedDireita = speed;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorDireita.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorEsquerda.setTargetPosition(newLeftTarget);
            robot.motorDireita.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(gyroCalculate() > 0 && speed == 0.6) {
                speedEsquerda = speedEsquerda - 0.2;
            }
            if(gyroCalculate() < 0 && speed == 0.6) {
                speedDireita = speedDireita - 0.2;
            }

            runtime.reset();
            robot.motorEsquerda.setPower(Math.abs(speedEsquerda));
            robot.motorDireita.setPower(Math.abs(speedDireita));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.motorEsquerda.isBusy() && robot.motorDireita.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.motorEsquerda.getCurrentPosition(),
                                            robot.motorDireita.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorEsquerda.setPower(0);
            robot.motorDireita.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public double gyroCalculate(){
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
