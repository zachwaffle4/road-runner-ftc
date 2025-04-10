package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.actions.InitLoopAction
import com.acmerobotics.roadrunner.actions.now
import com.acmerobotics.roadrunner.control.Drive
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectories.TimeTurn
import kotlin.properties.Delegates
import kotlin.text.get

class TurnAction(
    @JvmField
    val turn: TimeTurn,
    @JvmField
    val drive: Drive<*, *>
) : InitLoopAction() {
    var startTime by Delegates.notNull<Double>()

    override fun init(p: TelemetryPacket) {
        startTime = now()
    }

    override fun loop(p: TelemetryPacket): Boolean {
        val t = now() - startTime

        if (t > turn.duration) {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            return false
        }

        val target = turn[t]
        val robotVel = drive.localizer.update()

        val command = drive.controller.compute(
            target,
            drive.localizer.pose,
            robotVel
        )

        drive.setDrivePowersWithFF(command)

        val c = p.fieldOverlay()
        drive.drawPoseHistory(c)

        c.setStroke("#4CAF50")
        drawRobot(c, target.value())

        c.setStroke("#3F51B5")
        drawRobot(c, drive.localizer.pose)

        c.setStroke("#7C4DFFFF")
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

        return true
    }
}