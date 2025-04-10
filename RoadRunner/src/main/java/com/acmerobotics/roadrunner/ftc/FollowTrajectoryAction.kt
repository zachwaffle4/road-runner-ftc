package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.Drive
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory
import java.lang.Math.toDegrees
import kotlin.math.ceil
import kotlin.math.max
import kotlin.properties.Delegates
import kotlin.text.get

class FollowTrajectoryAction(
    @JvmField val traj: TimeTrajectory,
    @JvmField val drive: Drive<*, *>
) : InitLoopAction() {
    var startTime by Delegates.notNull<Double>()
    val points = range(
        0.0,
        traj.path.length(),
        max(2, ceil(traj.path.length() / 2 ).toInt())
    ).let {
        List<Vector2d>(it.size) { i ->
            traj.path[it[i], 1].value().position
        }
    }

    val xPoints = points.map { it.x }.toDoubleArray()
    val yPoints = points.map { it.y }.toDoubleArray()

    override fun init(p: TelemetryPacket) {
        startTime = now()
    }

    override fun loop(p: TelemetryPacket): Boolean {
        val t = now() - startTime

        if (t >= traj.duration) {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            return false
        }

        val target = traj[t]
        val robotVel = drive.updatePoseEstimate()

        val command = drive.controller.compute(
            target,
            drive.localizer.pose,
            robotVel
        )

        drive.setDrivePowersWithFF(command)

        p.put("x", drive.localizer.pose.position.x)
        p.put("y", drive.localizer.pose.position.y)
        p.put("heading (deg)", toDegrees(drive.localizer.pose.heading.toDouble()))

        val error = target.value().minusExp(drive.localizer.pose)
        p.put("xError", error.position.x)
        p.put("yError", error.position.y)
        p.put("headingError (deg)", toDegrees(error.heading.toDouble()))

        // only draw when active; only one drive action should be active at a time
        val c = p.fieldOverlay()
        drive.drawPoseHistory(c)

        c.setStroke("#4CAF50")
        drawRobot(c, target.value())

        c.setStroke("#3F51B5")
        drawRobot(c, drive.localizer.pose)

        c.setStroke("#4CAF50FF")
        c.setStrokeWidth(1)
        c.strokePolyline(xPoints, yPoints)

        return true
    }
}