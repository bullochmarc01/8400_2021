package org.firstinspires.ftc.teamcode.teleop;

//import org.firstinspires.ftc.teamcode.helpers.IMUHelper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class RobotTeleop {
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;


    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotTeleop() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.dcMotor.get("leftFront");
        leftRear = hwMap.dcMotor.get("leftRear");

        rightFront = hwMap.dcMotor.get("rightFront");
        rightRear = hwMap.dcMotor.get("rightRear");

        //reverse one side of the drivetrain so the robot drives straight

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        // Turn on motor braking
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.pu
        period.reset();
    }


    public void drive(double left_front, double left_rear, double right_front, double right_rear) {
        leftFront.setPower(rangeclip(left_front));
        rightFront.setPower(rangeclip(right_front));
        leftRear.setPower(rangeclip(left_rear));
        rightRear.setPower(rangeclip(right_rear));

    }

    public void strafe(String dir, double power) {
        if (dir == "right") {
            drive(power, -power, -power, power);
        }

        if (dir == "left") {
            drive(-power, power, power, -power);
        }
    }


    public void mecDrive(float left_stick_x, float left_stick_y, float right_stick_x, float max_power_level) {
        //limit drive power to the max_power_level value
        left_stick_x = Range.clip(left_stick_x, -max_power_level, max_power_level);
        left_stick_y = Range.clip(left_stick_y, -max_power_level, max_power_level);
        right_stick_x = Range.clip(right_stick_x, -max_power_level, max_power_level);

        double robotAngle;
        double r = Math.hypot(-left_stick_x, left_stick_y);

        robotAngle = Math.atan2(-left_stick_y, left_stick_x) - Math.PI / 4;

        double rightX = right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        drive(v1, v3, v2, v4);
    }

    /***
     * @param v
     */
    public double rangeclip(double v) {
        return Range.clip(v, -1, 1);
    }

    public void setRuntoPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunUsingEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunWithoutEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAndResetEncoders() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean AreMotorsBusy() {
        return (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy());
    }

    public void EncoderDrive(int distance) {

        stopAndResetEncoders();

        setRuntoPosition();

        int startPosition = leftFront.getCurrentPosition();
        int targetValue = startPosition + distance;

        double maxDistance = distance + 765;
        double minSpeed = 0.2;
        double fullSpeed = 1;
        double distSpeed = fullSpeed-minSpeed;

        double scalar = distSpeed / Math.pow(maxDistance, 2); //1/5100d for linear 4/83436125d for expon at max of 4085 or 1 / 18336125d for no reason

        leftFront.setTargetPosition(targetValue);
        leftRear.setTargetPosition(targetValue);
        rightFront.setTargetPosition(targetValue);
        rightRear.setTargetPosition(targetValue);

        while (leftFront.isBusy()) {

            double distRemaining = targetValue - leftFront.getCurrentPosition();

            double ex_Var_Speed = Math.abs(scalar * Math.pow(distRemaining, 2)) + minSpeed;

            leftFront.setPower(ex_Var_Speed);
            leftRear.setPower(ex_Var_Speed);
            rightRear.setPower(ex_Var_Speed);
            rightFront.setPower(ex_Var_Speed);

        }

        stopAndResetEncoders();

    }

}