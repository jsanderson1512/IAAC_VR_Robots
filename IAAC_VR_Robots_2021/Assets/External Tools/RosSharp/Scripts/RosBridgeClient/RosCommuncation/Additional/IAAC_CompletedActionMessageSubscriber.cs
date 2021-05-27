using UnityEngine;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;

public class IAAC_CompletedActionMessageSubscriber : UnitySubscriber<RosSharp.RosBridgeClient.MessageTypes.Std.String>
{
    public RosSharp_Kinematics ikModel;
    private bool newMessage = false;
    protected override void ReceiveMessage(RosSharp.RosBridgeClient.MessageTypes.Std.String message)
    {
        Debug.Log(message.data);
        //do the thing
        newMessage = true;
     }

    private void Update()
    {
        if(newMessage)
        {

            ikModel.SendPointsToROSOneAtATime();
            newMessage = false;
        }


    }
}

