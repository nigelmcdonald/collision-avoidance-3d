using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using C5;// treebag dotnet 4.5 win8


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

public class UNUSED : MonoBehaviour
{
    TreeBag<GameObject> priorityQue;
    // Start is called before the first frame update
    void Start()
    {
        priorityQue = new TreeBag<GameObject>(new ObstacleComparer(this.gameObject), C5.EqualityComparer<GameObject>.Default);//define obstacle storage
        //priorityQue.Add(hit.transform.gameObject);
        priorityQue.Clear();// clear right after I have intereacted with Obstacle[0] to avoice ect.   
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
