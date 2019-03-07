package org.team401.robot2019.util

import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import java.io.File

data class TrajectoryPath(val reverse: Boolean,
                          val waypoints: ArrayList<Pose2d>,
                          val settings: DoubleArray)

object PathReader{

    fun readTrajectory(file: File): TrajectoryPath{
        val gson = Gson()
        val parser = JsonParser()

        val json = file.readText()

        val jsonArray = parser.parse(json).asJsonArray
        val path = ArrayList<Pose2d>()

        for (i in 0 until jsonArray.size() -1){
            val jsonObj = gson.fromJson(jsonArray.get(i), JsonObject::class.java)
            val x = jsonObj.get("x").asDouble
            val y = jsonObj.get("y").asDouble
            val angle = jsonObj.get("angle").asDouble

            path.add(Pose2d(x, y, Rotation2d.fromDegrees(angle)))
        }

        val jsonSettings = gson.fromJson(jsonArray.last(), JsonObject::class.java)
        val maxVelocity = jsonSettings.get("maxVelocity").asDouble
        val maxAcceleration = jsonSettings.get("maxAcceleration").asDouble
        val maxVoltage = jsonSettings.get("maxVoltage").asDouble
        val reverse = jsonSettings.get("reverse").asBoolean

        val settings = doubleArrayOf(maxVelocity, maxAcceleration, maxVoltage)

        val output = TrajectoryPath(reverse, path, settings)
        println(output)

        return output
    }
}

