package org.team401.robot2019.control.superstructure.geometry

/**
 * @author Cameron Earle
 * @version 3/27/2019
 *
 */
enum class VisionHeightMode(val pipeline: Int) {
    NONE(1),
    HATCH_INTAKE(1),
    HATCH_SCORE(1),
    CARGO_SCORE_ROCKET(1)
}