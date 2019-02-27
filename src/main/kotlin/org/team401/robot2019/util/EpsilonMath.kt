package org.team401.robot2019.util

import org.snakeskin.measure.MeasureUnitless

/**
 * @author Cameron Earle
 * @version 2/27/2019
 *
 */
const val EPSILON = 1e-6

infix fun Double.epsEq(that: Double): Boolean {
    return this == that || (Math.abs(this - that) < EPSILON)
}

infix fun Double.epsGt(that: Double): Boolean {
    return (this - that) > EPSILON

}

infix fun Double.epsLt(that: Double): Boolean {
    return (that - this) > EPSILON
}

infix fun Double.epsGe(that: Double): Boolean {
    return this.epsGt(that) || this.epsEq(that)
}

infix fun Double.epsLe(that: Double): Boolean {
    return this.epsLt(that) || this.epsEq(that)
}