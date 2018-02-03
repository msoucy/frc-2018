/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import frc.team166.robot.vision.LuminousPipeline;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Vision extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private final int IMG_WIDTH = 340;
    private final int IMG_HEIGHT = 240;

    UsbCamera camera;
    VisionThread thread;
    LuminousPipeline pipeline;

    double centerX = Double.NaN;

    public Vision() {
        camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        pipeline = new LuminousPipeline();
        thread = new VisionThread(camera, pipeline, pipeline -> {
            if (pipeline.filterContoursOutput().isEmpty()) {
                Rect bounds = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
                synchronized (this) {
                    centerX = bounds.x + (bounds.width / 2.0);
                }
            } else {
                synchronized (this) {
                    centerX = Double.NaN;
                }
            }
        });
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
