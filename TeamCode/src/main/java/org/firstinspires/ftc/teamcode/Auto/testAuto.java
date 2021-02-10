package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;

@Deprecated
@Autonomous(name = "Test Auto", group = "Axel")
public class testAuto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private ArrayList<DcMotor> driveMotors = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "FR");
        rightFront = hardwareMap.get(DcMotor.class, "BR");



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        driveMotors.add(leftRear);
        driveMotors.add(leftFront);
        driveMotors.add(rightFront);
        driveMotors.add(rightRear);


        telemetry.addData("Ready.", 0);
        telemetry.update();

        waitForStart();

        sleep(500);

        double xPower = -0.5;
        double yPower = 0;
        double zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(1000);


    }
    public void driveMecanum(double xPower,double yPower,double  zPower) {
        yPower = -yPower;
        rightFront.setPower(1 * (((-yPower) + (xPower)) + -zPower));
        leftRear.setPower(1 * (((-yPower) + (-xPower)) + zPower));
        leftFront.setPower(1 * (((-yPower) + (xPower)) + zPower));
        rightRear.setPower(1 * (((-yPower) + (-xPower)) + -zPower));
    }


}