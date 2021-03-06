using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//this class transforms the wheel models to match the wheelcoliders so you can see the wheels spinning and turning
public class WheelMovement : MonoBehaviour
{
    public WheelCollider targetWheel;

    private Vector3 wheelPos = new Vector3();
    private Quaternion wheelRot = new Quaternion();

    // Update is called once per frame
    void Update()
    {
        targetWheel.GetWorldPose(out wheelPos, out wheelRot);
        transform.position = wheelPos;
        transform.rotation = wheelRot;
    }
}
