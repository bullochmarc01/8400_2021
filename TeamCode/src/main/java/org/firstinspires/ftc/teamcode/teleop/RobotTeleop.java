package org.firstinspires.ftc.teamcode.helpers;

//import org.firstinspires.ftc.teamcode.helpers.IMUHelper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class DriveTrainHelper {
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightRearMotor = null;


    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public DriveTrainHelper() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("left front motor");
        leftRearMotor = hwMap.dcMotor.get("left rear motor");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor = hwMap.dcMotor.get("right front motor");
        rightRearMotor = hwMap.dcMotor.get("right rear motor");

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        leftFrontMotor.setPower(rangeclip(left_front));
        rightFrontMotor.setPower(rangeclip(right_front));
        leftRearMotor.setPower(rangeclip(left_rear));
        rightRearMotor.setPower(rangeclip(right_rear));

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
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunUsingEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunWithoutEncoder() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAndResetEncoders() {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        rightFrontMotor.setPower(0);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean AreMotorsBusy() {
        return (leftFrontMotor.isBusy() && leftRearMotor.isBusy() && rightFrontMotor.isBusy() && rightRearMotor.isBusy());
    }

    public void EncoderDrive(int distance) {

        stopAndResetEncoders();

        setRuntoPosition();

        int startPosition = leftFrontMotor.getCurrentPosition();
        int targetValue = startPosition + distance;

        double maxDistance = distance + 765;
        double minSpeed = 0.2;
        double fullSpeed = 1;
        double distSpeed = fullSpeed-minSpeed;

        double scalar = distSpeed / Math.pow(maxDistance, 2); //1/5100d for linear 4/83436125d for expon at max of 4085 or 1 / 18336125d for no reason

        leftFrontMotor.setTargetPosition(targetValue);
        leftRearMotor.setTargetPosition(targetValue);
        rightFrontMotor.setTargetPosition(targetValue);
        rightRearMotor.setTargetPosition(targetValue);

        while (leftFrontMotor.isBusy()) {

            double distRemaining = targetValue - leftFrontMotor.getCurrentPosition();

            double ex_Var_Speed = Math.abs(scalar * Math.pow(distRemaining, 2)) + minSpeed;

            leftFrontMotor.setPower(ex_Var_Speed);
            leftRearMotor.setPower(ex_Var_Speed);
            rightRearMotor.setPower(ex_Var_Speed);
            rightFrontMotor.setPower(ex_Var_Speed);

        }

        stopAndResetEncoders();

    }

}