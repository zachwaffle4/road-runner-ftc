@file:JvmName("Actions")

package com.acmerobotics.roadrunner.actions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket

/**
 * Run [a] to completion in a blocking loop.
 */
fun runBlocking(a: Action) {
    val dash = FtcDashboard.getInstance()
    val c = Canvas()
    a.preview(c)

    var b = true
    while (b && !Thread.currentThread().isInterrupted) {
        val p = TelemetryPacket()
        p.fieldOverlay().operations.addAll(c.operations)

        b = a.run(p)

        dash.sendTelemetryPacket(p)
    }
}
