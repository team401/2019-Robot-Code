package org.team401.robot2019.util

import com.google.gson.Gson
import com.google.gson.GsonBuilder
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import java.io.File
import java.io.FileWriter

data class TrajectoryPath(val reverse: Boolean,
                          val waypoints: List<Pose2d>,
                          val maxVel: Double,
                          val maxAccel: Double,
                          val maxVoltage: Double){
}

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

        val output = TrajectoryPath(reverse, path, maxVelocity, maxAcceleration, maxVoltage)
        //println(output)

        return output
    }

    fun outputToPathViewer(trajectory: TrajectoryPath, fileName: String){
        val gson = GsonBuilder().create()
        val path = trajectory.waypoints
        val waypoints = ArrayList<JsonObject>()

        // get the path's waypoints
        path.forEach{
            val waypoint = JsonObject()
            waypoint.addProperty("x", it.translation.x())
            waypoint.addProperty("y", it.translation.y())
            waypoint.addProperty("angle", it.translation.direction().degrees)
            waypoints.add(waypoint)
        }
        // add the path settings
        val settings = JsonObject()
        settings.addProperty("maxVoltage", trajectory.maxVoltage)
        settings.addProperty("maxVelocity", trajectory.maxVel)
        settings.addProperty("maxAcceleration", trajectory.maxAccel)
        settings.addProperty("maxCentripetalAcceleration", 130.0)//TODO update this
        settings.addProperty("reverse", trajectory.reverse)

        waypoints.add(settings)
        val writer = FileWriter("paths\\$fileName.json")
        val json = gson.toJson(waypoints)

        writer.write(json)
        writer.flush()
        writer.close()

        //println(json)
    }

}

