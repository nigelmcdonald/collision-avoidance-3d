using UnityEngine;
using System.Collections.Generic;
using System.Collections;


public class CarEngine : MonoBehaviour {

    //references
    [Header("References")]
    public Transform path;
    private List<Transform> nodes;
    private int currentNodeIndex = 0;
    public WheelCollider wheelFrontLeft;
    public WheelCollider wheelFrontRight;
    public WheelCollider wheelRearLeft;
    public WheelCollider wheelRearRight;
    public Texture2D notBreakingTex;
    public Texture2D breakingTex;
    public Renderer carRenderer;

    //turning
    [Header("Turning")]
    public float maxSteeringAngle = 45f;
    public float steeringLerpSteps = 0.5f;
    float steeringNew;
    public float maxSpeedInCorner = 8f;
    private bool activeAvoidance = false; // for sensors to turn away from obstacles
    private Vector3 reletiveVector;

    //thrust/breaking
    [Header("Thust/Breaking")]
    public float motorToque = 50f;
    public float currentSpeed;
    public float MaxSpeed = 15f;
    public float CurrentAllowedSpeed = 15f;
    private float OldMaxSpeed;    
    public float slowingDistanceToNode = 10f;
    public float maxBreakingTorque = 150f;
    public float currentBreakingTorque = 150f;
    public float ActualBreakTorque;
    public float MinSafeBrakeDist = 0.5f;
    public bool comeToStop = false;

    //sensors
    [Header("Sensors")]
    private Vector3 oldPos;
    public float sensorLength = 10f;
    public Vector3 frontSensorPos = new Vector3(0.3f,0.07f, 0);
    public float sensorSpacing = 0.35f;
    public float frontAngleSensorDeg = 30f;
    private float avoidMultiplier = 0f;

    private Rigidbody rb;
    public Vector3 centerOfMass; // change to help stabalise the vehicle

	void Start () {        
        GetComponent<Rigidbody>().centerOfMass = centerOfMass; // adjust center of mass for stability
        oldPos = transform.position; // for calculating speed when compared against transform.pos each frame
        OldMaxSpeed = CurrentAllowedSpeed;// save max speed
        rb = GetComponent<Rigidbody>();
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();// find all the path nodes
        nodes = new List<Transform>();
        //copy over transforms that arent the parents, only the nodes
        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }
    }

    //method called 50 times per seconds
    private void FixedUpdate()
    {
        sensorLength = 3f + currentSpeed; // adjust sensor responce based on current speed, min length of 3        
        MinSafeBrakeDist = currentSpeed / 2.5f; // min safe stopping distance depends on current speed
        //ActualBreakTorque = wheelRearLeft.brakeTorque; // debugging
        CheckClosestNodeDistance();
        Sensors();
        Breaking();        
        ApplySteering();
        AutoDrive();               
    }

    // this function fires raycasts to detect Collisions with things tagged as obstacles
    // this function modifies the allowable speed based on the conditions
    // LIMITATION: limited sensor resolution leads to blind spots so would need to be improved to be used in the real world
    private void Sensors()
    {
        avoidMultiplier = 0; // add to this to increase the strength of turning
        RaycastHit hit;

        Vector3 sensorStartPos = transform.position + frontSensorPos;
        sensorStartPos += transform.forward * frontSensorPos.x;
        sensorStartPos += transform.up * frontSensorPos.y;

        //front center sensor
        if (avoidMultiplier == 0)
        {
            if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength)) // center sensors
            {
                if (hit.transform.tag == "Obstacle") // only store obstacle type objects
                {                    
                    Debug.DrawLine(sensorStartPos, hit.point);                    
                    if (hit.distance < MinSafeBrakeDist && currentSpeed > 5) // if too close to the front then max breaking applied
                    {
                        comeToStop = false;
                        activeAvoidance = true;                        
                        if (hit.normal.x < 0) // if the obstacle is angled then turn towards the least steep angle to avoid
                        {
                            avoidMultiplier = -1;
                        }
                        else
                        {
                            avoidMultiplier = 1;
                        }
                    }
                    else
                    {
                        CurrentAllowedSpeed = 0f;
                        comeToStop = true;                     
                    }
                    if (currentSpeed < 5)
                    {
                        CurrentAllowedSpeed = 0f;
                        comeToStop = true;
                    }
                }
            }
        }


        //front right sensor
        sensorStartPos += transform.right * sensorSpacing;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only compare obstacle type objects
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                if (hit.distance < MinSafeBrakeDist && currentSpeed > 5) // if too close to the front then max breaking applied
                {
                    activeAvoidance = true;
                    comeToStop = false;
                    avoidMultiplier -= 1f;
                }
                else
                {
                    comeToStop = true;
                    CurrentAllowedSpeed = 0f;
                }
                if (currentSpeed < 5)
                {
                    CurrentAllowedSpeed = 0f;
                    comeToStop = true;
                }
            }            
        }
        //front right angle sensor                                                               gives vector 3 of the direction       shorten side sensors
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(frontAngleSensorDeg, transform.up) * transform.forward , out hit, sensorLength/3)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only compare obstacle type objects
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                comeToStop = false;
                activeAvoidance = true;                
                avoidMultiplier -= 0.75f;
            }
        }

        //front left sensor                                                                    
        sensorStartPos -= transform.right * (sensorSpacing * 2);        
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only compare obstacle type objects
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                if (hit.distance < MinSafeBrakeDist && currentSpeed > 5) // if too close to the front then max breaking applied
                {
                    comeToStop = false;
                    activeAvoidance = true;
                    avoidMultiplier += 1f;
                }
                else
                {
                    CurrentAllowedSpeed = 0f;
                    comeToStop = true;
                }
                if (currentSpeed < 5)
                {
                    CurrentAllowedSpeed = 0f;
                    comeToStop = true;
                }
            }
        }
        //front left angle sensor                                                                 gives vector 3 of the direction      shorten side sensors
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontAngleSensorDeg, transform.up) * transform.forward, out hit, sensorLength/3)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only compare obstacle type objects
            {
                Debug.DrawLine(sensorStartPos, hit.point);
                comeToStop = false;
                activeAvoidance = true;
                avoidMultiplier += 0.75f;           
            }
        }

        if (hit.transform == null)
        {
            comeToStop = false;
        }
        
             
        
    }

    // define best steering angle towards the vector possible
    private float CalculateNewTurnVect(Vector3 reletiveVec , float maxSteer)
    {
        return (reletiveVec.x / reletiveVec.magnitude) * maxSteer;
    }

    //function applys steering forces
    private void ApplySteering()
    {
        if (!activeAvoidance)
        {
            reletiveVector = transform.InverseTransformPoint(nodes[currentNodeIndex].position);
            reletiveVector = reletiveVector / reletiveVector.magnitude; // clamp between -1 and 1  
            steeringNew = CalculateNewTurnVect(reletiveVector, maxSteeringAngle);

            wheelFrontLeft.steerAngle = Mathf.Lerp(wheelFrontLeft.steerAngle, steeringNew, Time.deltaTime * steeringLerpSteps);
            wheelFrontRight.steerAngle = wheelFrontLeft.steerAngle; // both wheels align
        }
        else // steer away from obstacle
        {
            wheelFrontLeft.steerAngle = Mathf.Lerp(wheelFrontLeft.steerAngle, maxSteeringAngle * avoidMultiplier, Time.deltaTime * steeringLerpSteps);
            wheelFrontRight.steerAngle = wheelFrontLeft.steerAngle; // both wheels align
            activeAvoidance = false;
        }       
    }


    // this function checks how close you are to the current waypoint and when that distance
    // is below a set value it will move the target to the next node
    private void CheckClosestNodeDistance()
    {        
        if (Vector3.Distance(transform.position, nodes[currentNodeIndex].position) < 1) // if we are very close increase nodeindex
        {
            if (currentNodeIndex == nodes.Count - 1) // if last node then move back to the first
            {
                currentNodeIndex = 0;
            }
            else
            {
                currentNodeIndex++;
            }            
        }

        //reduce speed if within slowingDistanceToNode from next node so that the turn can be made safely
        if (Vector3.Distance(transform.position, nodes[currentNodeIndex].position) < slowingDistanceToNode) 
        {
            CurrentAllowedSpeed = maxSpeedInCorner; //slow
        }
        else
        {
            CurrentAllowedSpeed = OldMaxSpeed; // speed back up
        }
    }

    // this funciton applies torque to the wheels on if the current speed < max speed
    private void AutoDrive()
    {
        currentSpeed = Vector3.Distance(oldPos, transform.position) * 100; // calculate speed comparing the distance traveled in 1/60th of a second
        oldPos = transform.position; // update old position 
        if (currentSpeed < CurrentAllowedSpeed && !comeToStop)
        {
            wheelFrontLeft.motorTorque = motorToque;
            wheelFrontRight.motorTorque = motorToque;
        }
        else // dont accelerate
        {
            wheelFrontLeft.motorTorque = 0;
            wheelFrontRight.motorTorque = 0;
        }        
    }

    // this funtion handles changing the texture of the car to show a break light and also
    // to apply a breaking force to the rear wheels
    // breaking occurs if over allowed speed
    private void Breaking()
    {
        if (comeToStop)
        {
            currentBreakingTorque = maxBreakingTorque;
            wheelRearLeft.brakeTorque = currentBreakingTorque;
            wheelRearRight.brakeTorque = currentBreakingTorque;
            wheelFrontLeft.brakeTorque = currentBreakingTorque / 2; // reduce front wheel break force to prevent spinouts
            wheelFrontRight.brakeTorque = currentBreakingTorque / 2; 
        }
        else if (currentSpeed > CurrentAllowedSpeed)
        {            
            wheelRearLeft.brakeTorque = currentBreakingTorque;
            wheelRearRight.brakeTorque = currentBreakingTorque;
            wheelFrontLeft.brakeTorque = currentBreakingTorque / 2; // reduce front wheel break force to prevent spinouts 
            wheelFrontRight.brakeTorque = currentBreakingTorque / 2;
        }
        else // no breaking needed to reduce break toque to 0
        {            
            carRenderer.material.mainTexture = notBreakingTex;
            wheelRearLeft.brakeTorque = 0;
            wheelRearRight.brakeTorque = 0;
            wheelFrontLeft.brakeTorque = 0;
            wheelFrontRight.brakeTorque = 0;
        }        
        if (currentSpeed - CurrentAllowedSpeed > 0.8 || comeToStop) // only show brakes if heavy over the breaking threshold so the light doesnt flicker
        {
            carRenderer.material.mainTexture = breakingTex;
        }
    }
}
