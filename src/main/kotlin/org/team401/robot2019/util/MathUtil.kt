package org.team401.robot2019.util

object MathUtil {
    fun offsetEncoder12B(value: Int, zeroPoint: Int, negWrapOffset: Int): Int {
        val rotated = Math.floorMod(value - zeroPoint, 4096)
        if (rotated >= negWrapOffset) {
            return rotated - 4096
        }
        return rotated
    }
}