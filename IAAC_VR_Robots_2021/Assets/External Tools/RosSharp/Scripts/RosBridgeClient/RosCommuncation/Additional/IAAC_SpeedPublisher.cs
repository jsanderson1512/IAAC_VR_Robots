using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;


public class IAAC_SpeedPublisher : UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Std.Float32>
{
    private RosSharp.RosBridgeClient.MessageTypes.Std.Float32 message;

    //publish message when funciton is called
    public void SetSpeed(float speed)
    {
        message.data = speed;
        Publish(message);
    }
}
