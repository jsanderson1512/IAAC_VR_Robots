using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using System;

public static class RosHelper
{

    public static string SpecialToString(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Point point)
    {
        return "("
            + point.x.ToString("F6") + ", "
            + point.y.ToString("F6") + ", "
            + point.z.ToString("F6")
            + ")";
    }
    public static string SpecialToString(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 point)
    {
        return "("
            + point.x.ToString("F6") + ", "
            + point.y.ToString("F6") + ", "
            + point.z.ToString("F6")
            + ")";
    }
    public static string SpecialToString(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion quaternion)
    {
        return "("
            + quaternion.x.ToString("F6") + ", "
            + quaternion.y.ToString("F6") + ", "
            + quaternion.z.ToString("F6") + ", "
            + quaternion.w.ToString("F6")
            + ")";
    }
    // Unity Vec3 to Ros Point
    public static RosSharp.RosBridgeClient.MessageTypes.Geometry.Point UnityVector3ToRosPoint(this Vector3 unityPoint)
    {
        return new RosSharp.RosBridgeClient.MessageTypes.Geometry.Point(
            Convert.ToDouble(-unityPoint.z),
            Convert.ToDouble(unityPoint.x),
            Convert.ToDouble(unityPoint.y)
        );
    }
    public static Vector3 RosPointToUnityVector3(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Point rosPoint)
    {
        return new Vector3(
            Convert.ToSingle(rosPoint.y),
            Convert.ToSingle(rosPoint.z),
            Convert.ToSingle(-rosPoint.x)
        );
    }
    // Unity Vec3 to Ros Vec3
    public static RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 UnityVector3ToRosVector3(this Vector3 unityPoint)
    {
        return new RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3(
            Convert.ToDouble(-unityPoint.z),
            Convert.ToDouble(unityPoint.x),
            Convert.ToDouble(unityPoint.y)
        );
    }
    public static Vector3 RosVector3ToUnityVector3(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 rosPoint)
    {
        return new Vector3(
            Convert.ToSingle(rosPoint.y),
            Convert.ToSingle(rosPoint.z),
            Convert.ToSingle(-rosPoint.x)
        );
    }
    // Unity Euler to Ros Quaternion
    public static RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion UnityEulerToRosQuaternion(this Vector3 unityEulerOrientation) {
        Quaternion unityOrientation = Quaternion.Euler(unityEulerOrientation.x, unityEulerOrientation.z, unityEulerOrientation.y);

        return new RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion(
            Convert.ToDouble(unityOrientation.x),
            Convert.ToDouble(unityOrientation.y),
            Convert.ToDouble(unityOrientation.z),
            Convert.ToDouble(unityOrientation.w));
    }
    public static Vector3 RosQuaternionToUnityEuler(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion rosOrientation)
    {
        Quaternion unityOrientation = new Quaternion(
            Convert.ToSingle(rosOrientation.x),
            Convert.ToSingle(rosOrientation.z),
            Convert.ToSingle(rosOrientation.y),
            Convert.ToSingle(rosOrientation.w));
        Vector3 rosEulerOrientation = unityOrientation.eulerAngles;

        return new Vector3(
            rosEulerOrientation.x, 
            rosEulerOrientation.z, 
            rosEulerOrientation.y);
    }
    // Unity Euler to Ros Vec3
    public static RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 UnityEulerToRosEuler(this Vector3 unityEulerOrientation)
    {
        return new RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3(
            Convert.ToDouble(-unityEulerOrientation.y),
            Convert.ToDouble(-unityEulerOrientation.x),
            Convert.ToDouble(unityEulerOrientation.z)
        );
    }
    public static Vector3 RosEulerToUnityEuler(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Vector3 rosEulerOrientation)
    {
        return new Vector3(
            Convert.ToSingle(-rosEulerOrientation.y),
            Convert.ToSingle(-rosEulerOrientation.x),
            Convert.ToSingle(rosEulerOrientation.z)
        );
    }
}
