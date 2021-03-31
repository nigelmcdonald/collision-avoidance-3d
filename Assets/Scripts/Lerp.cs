using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//this class moves and object smoothly between two vector3 positions
// this also stops the movement when the car is within half a body legth of the obstacle
// size, travel speed, and travel distance are all randomized between a range
//ref: https://gamedevbeginner.com/the-right-way-to-lerp-in-unity-with-examples/
public class Lerp : MonoBehaviour
{
    private Vector3 startpoint;
    private Vector3 endPoint;
    private float timeElapsed = 0f;
    private float LerpDuration;
    private float travelDistance;
    public bool carInFront = true;

    private void Start()
    {
        transform.localScale = new Vector3(1, 1, Random.Range(1f, 4f));// random length of the obstacle
        travelDistance = Random.Range(8f, 15f); // this is randomise the timing of the obstacles each time the program is run
        LerpDuration = Random.Range(3f, 12f); // as above for travel speed

        startpoint = transform.position;
        endPoint = transform.position + transform.forward * travelDistance;
    }
    void Update()
    {
        if (!carInFront)
        {
            if (timeElapsed < LerpDuration)
            {
                transform.position = Vector3.Lerp(startpoint, endPoint, timeElapsed / LerpDuration);
                timeElapsed += Time.deltaTime;
            }
            else
            {
                timeElapsed = 0f;
            }
        }        
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Car"))
        {
            carInFront = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.CompareTag("Car"))
        {
            carInFront = false;
        }
    }
}
