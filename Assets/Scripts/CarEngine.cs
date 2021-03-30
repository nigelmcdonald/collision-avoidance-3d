using UnityEngine;
using System.Collections.Generic;
using System.Collections;
using C5;


// this class compares store game objects based on distance from the vehicle and prioritise
// shortest distance first
//ref https://csharp.hotexamples.com/examples/-/TreeBag/-/php-treebag-class-examples.html
class ObstacleComparer : IComparer<GameObject>
{
    GameObject self;
    public ObstacleComparer(GameObject _self)
    {
        self = _self;
    }

    public int Compare(GameObject x, GameObject y)
    {
        int xVal = CalculatePriority(x);
        int yVal = CalculatePriority(y);
        return yVal - xVal;
    }

    private int CalculatePriority(GameObject obstacle)
    {
        return Mathf.RoundToInt(Vector3.Distance(self.transform.position, obstacle.transform.position));
    }
}


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

    //statuses
    public bool breaking = false;

    //turning
    [Header("Turning")]
    public float maxSteeringAngle = 45f;
    public float steeringLerpSteps = 0.5f;
    float steeringNew;
    bool isTurning = false;
    private bool activeAvoidance = false; // for sensors to turn away from obstacles
    
    //thrust/breaking
    [Header("Thust/Breaking")]
    public float motorToque = 50f;
    public float currentSpeed;
    public float maxSpeed = 30f;
    private float OldMaxSpeed;
    public float turningSpeed = 5f;
    public float slowingDistanceToNode = 15f;
    public float maxBreakingTorque = 150f;

    //sensors
    [Header("Sensors")]
    public float sensorLength = 10f;
    public Vector3 frontSensorPos = new Vector3(0.3f,0.07f, 0);
    public float sensorSpacing = 0.35f;
    public float frontAngleSensorDeg = 30f;
    TreeBag<GameObject> priorityQue;

    private Rigidbody rb;
    public Vector3 centerOfMass;

	void Start () {
        priorityQue = new TreeBag<GameObject>(new ObstacleComparer(this.gameObject), C5.EqualityComparer<GameObject>.Default);//define obstacle storage

        GetComponent<Rigidbody>().centerOfMass = centerOfMass;
        OldMaxSpeed = maxSpeed;// save max speed
        rb = GetComponent<Rigidbody>();
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
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

    //method called 60 times per seconds
    private void FixedUpdate()
    {
        Sensors();
        ApplySteering();
        AutoDrive();
        CheckClosestNodeDistance();
        Breaking();
    }

    // this function fires raycasts to detect Collisions with things tagged as obstacles
    // LIMITATION: limited sensor resolution leads to blind spots so would need to be improved to be used in the real world
    private void Sensors()
    {
        float avoidMultiplier = 0; // add to this to increase the strength of turning
        RaycastHit hit;

        Vector3 sensorStartPos = transform.position + frontSensorPos;
        sensorStartPos += transform.forward * frontSensorPos.x;
        sensorStartPos += transform.up * frontSensorPos.y;
                  

        //front right sensor
        sensorStartPos += transform.right * sensorSpacing;
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only store obstacle type objects
            {
                priorityQue.Add(hit.transform.gameObject);
                Debug.DrawLine(sensorStartPos, hit.point);
                activeAvoidance = true;
                avoidMultiplier -= 1f;
            }            
        }
        //front right angle sensor                                                               gives vector 3 of the direction
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(frontAngleSensorDeg, transform.up) * transform.forward, out hit, sensorLength)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only store obstacle type objects
            {
                priorityQue.Add(hit.transform.gameObject);
                Debug.DrawLine(sensorStartPos, hit.point);
                activeAvoidance = true;
                avoidMultiplier -= 0.5f;
            }
        }

        //front left sensor                                                                     gives vector 3 of the direction
        sensorStartPos -= transform.right * (sensorSpacing * 2);        
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only store obstacle type objects
            {
                priorityQue.Add(hit.transform.gameObject);
                Debug.DrawLine(sensorStartPos, hit.point);
                activeAvoidance = true;
                avoidMultiplier += 1f;
            }
        }
        //front left angle sensor
        else if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontAngleSensorDeg, transform.up) * transform.forward, out hit, sensorLength)) // center sensors
        {
            if (hit.transform.tag == "Obstacle") // only store obstacle type objects
            {
                priorityQue.Add(hit.transform.gameObject);
                Debug.DrawLine(sensorStartPos, hit.point);
                activeAvoidance = true;
                avoidMultiplier += 0.5f;
            }
        }

        //front center sensor only test if the other sensors arent acting or ar canceling each other out
        if (avoidMultiplier == 0)
        {
            if (Physics.Raycast(sensorStartPos, transform.forward, out hit, sensorLength)) // center sensors
            {
                if (hit.transform.tag == "Obstacle") // only store obstacle type objects
                {
                    priorityQue.Add(hit.transform.gameObject);
                    Debug.DrawLine(sensorStartPos, hit.point);
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
            }
        }
        

        if (activeAvoidance)
        {
            wheelFrontLeft.steerAngle = Mathf.Lerp(wheelFrontLeft.steerAngle, maxSteeringAngle * avoidMultiplier, Time.deltaTime * steeringLerpSteps);
            wheelFrontRight.steerAngle = wheelFrontLeft.steerAngle; // both wheels align
            activeAvoidance = false;
        };


        priorityQue.Clear();// clear right after I have intereacted with Obstacle[0] to avoice ect.        
        
    }

    

    //function applys steering forces
    private void ApplySteering()
    {
        if (!isTurning && !activeAvoidance)
        {
            Vector3 reletiveVector = transform.InverseTransformPoint(nodes[currentNodeIndex].position);
            //Debug.Log(reletiveVector);
            reletiveVector = reletiveVector / reletiveVector.magnitude; // clamp between -1 and 1

            steeringNew = (reletiveVector.x / reletiveVector.magnitude) * maxSteeringAngle; // define best steering angle towards the vector possible

            wheelFrontLeft.steerAngle = Mathf.Lerp(wheelFrontLeft.steerAngle, steeringNew, Time.deltaTime * steeringLerpSteps);
            wheelFrontRight.steerAngle = wheelFrontLeft.steerAngle; // both wheels align
            //StartCoroutine(WheelLerp()); 
        }                    
    }

    // not using coroutine currently as the steering becomes jerky
    // this coroutine run the lerp between the current vector and the vector of the next node in relation to wheel turning angle
    // it runs for lerpDuration seconds;
    //IEnumerator WheelLerp()
    //{
    //    isTurning = true;
    //    float timeElapsed = 0; // reset timer

    //    // set wheel colliders to the found angle but turn smoothly using a lerp between the two vectors
    //    while (timeElapsed < steeringLerpSteps)
    //    {
    //        //Debug.Log(steeringNew - wheelFrontLeft.steerAngle);
    //        wheelFrontLeft.steerAngle = Mathf.Lerp(wheelFrontLeft.steerAngle, steeringNew, timeElapsed / steeringLerpSteps);
    //        wheelFrontRight.steerAngle = Mathf.Lerp(wheelFrontRight.steerAngle, steeringNew, timeElapsed / steeringLerpSteps);

    //        timeElapsed += Time.deltaTime; // timer count up

    //        yield return null;
    //    }
    //    wheelFrontLeft.steerAngle = steeringNew;
    //    wheelFrontRight.steerAngle = steeringNew;
    //    isTurning = false;
    //}

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
            maxSpeed = turningSpeed; //slow
        }
        else
        {
            maxSpeed = OldMaxSpeed; // speed back up
        }
    }

    // this funciton applies torque to the wheels on if the current speed < max speed
    private void AutoDrive()
    {
        currentSpeed = 2 * Mathf.PI * wheelFrontLeft.radius * wheelFrontLeft.rpm * 60 / 1000; // calculates speed based on wheel radius and rotation speed
        if (currentSpeed < maxSpeed && !breaking)
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

    // this funtion handles changing the texture of hte car to show a break light and also
    // to apply a breaking force to the rear wheels
    private void Breaking()
    {
        if (breaking)
        {
            carRenderer.material.mainTexture = breakingTex;
            wheelRearLeft.brakeTorque = maxBreakingTorque;
            wheelRearRight.brakeTorque = maxBreakingTorque;
        }
        else
        {
            carRenderer.material.mainTexture = notBreakingTex;
            wheelRearLeft.brakeTorque = 0;
            wheelRearRight.brakeTorque = 0;
        }
    }
}
