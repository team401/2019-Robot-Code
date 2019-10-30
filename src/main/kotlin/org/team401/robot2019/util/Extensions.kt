package org.team401.robot2019.util

import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

fun Pose2d.fieldMirror(): Pose2d {
    return Pose2d(this.translation.x(), 322.0 - this.translation.y(), Rotation2d(this.rotation.inverse()))
}

fun Pose2d.withHeading(heading: Rotation2d): Pose2d {
    return Pose2d(this.translation, heading)
}