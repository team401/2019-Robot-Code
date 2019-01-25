package org.team401.robot2019

import com.ctre.phoenix.sensors.PigeonIMU
import com.ctre.phoenix.sensors.PigeonImuJNI

/**
 * @author Cameron Earle
 * @version 1/15/2019
 *
 */
object SketchyPigeonFactory {

    fun getPigeon(talonId: Int): PigeonIMU {
        val handle = PigeonImuJNI.JNI_new_PigeonImu_Talon(talonId)
        val instance = PigeonIMU(0)
        val field1 = instance.javaClass.getDeclaredField("m_deviceNumber")
        val field2 = instance.javaClass.getDeclaredField("m_handle")
        field1.isAccessible = true
        field2.isAccessible = true
        field1.set(instance, talonId)
        field2.set(instance, handle)
        return instance
    }
}