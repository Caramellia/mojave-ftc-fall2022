package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Fast AutoOp Red, Right", group = "Linear Opmode")
public class PowerPlaySuperAutoOp_R extends PowerPlaySuperAutoOp {

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        this.coneColor = new double[]{255, 0, 0};
        super.runOpMode();
    }

}
