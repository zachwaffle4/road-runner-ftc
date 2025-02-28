package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import kotlin.math.roundToInt

interface PinpointView {
    var parDirection: DcMotorSimple.Direction
    var perpDirection: DcMotorSimple.Direction

    var pose: Pose2d
    var vel: PoseVelocity2d

    fun update()
}

class PinpointParEncoder(val pinpoint: PinpointView) : Encoder {
    override var direction by pinpoint::parDirection
    override fun getPositionAndVelocity() = PositionVelocityPair(
        pinpoint.pose.position.x.roundToInt(),
        pinpoint.vel.linearVel.x.roundToInt()
    )
}

class PinpointPerpEncoder(val pinpoint: PinpointView) : Encoder {
    override var direction by pinpoint::perpDirection
    override fun getPositionAndVelocity() = PositionVelocityPair(
        pinpoint.pose.position.y.roundToInt(),
        pinpoint.vel.linearVel.y.roundToInt()
    )
}

class PinpointEncoderGroup(
    val pinpoint: PinpointView,
) : EncoderGroup {
    override val encoders = listOf(
        PinpointParEncoder(pinpoint),
        PinpointPerpEncoder(pinpoint),
    )
    override val unwrappedEncoders = encoders

    override fun bulkRead() {
        pinpoint.update()
    }
}

// Only the methods used by tuning routines are implemented
class PinpointIMU(val pinpoint: PinpointView) : IMU, LazyImu {
    override fun getManufacturer() = HardwareDevice.Manufacturer.Other
    override fun getDeviceName() = ""
    override fun getConnectionInfo() = ""
    override fun getVersion() = 0

    override fun resetDeviceConfigurationForOpMode() {}

    override fun close() {}

    override fun initialize(parameters: IMU.Parameters?) = true

    override fun resetYaw() {
        throw NotImplementedError()
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        throw NotImplementedError()
    }

    override fun getRobotOrientation(
        reference: AxesReference?,
        order: AxesOrder?,
        angleUnit: AngleUnit?
    ): Orientation {
        throw NotImplementedError()
    }

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        throw NotImplementedError()
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity {
        pinpoint.update()
        return AngularVelocity(angleUnit, 0.0f, 0.0f,
            pinpoint.vel.angVel.toFloat(), 0L)
    }

    override fun get() = this
}
