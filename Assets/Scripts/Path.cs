using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Path : MonoBehaviour
{
    public Color lineColor;
    private List<Transform> nodes = new List<Transform>();

    //draw the path the vehicle follows in editor
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = lineColor;

        Transform[] pathTransforms = GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();
        //copy over transforms that arent the parents, only the nodes
        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }
        for (int i = 1; i < nodes.Count; i++) // draw sphere gizmos at each node
        {
            Vector3 currentNode = nodes[i].position;
            Vector3 previousNode = nodes[i - 1].position;
            Gizmos.DrawLine(previousNode, currentNode);
            Gizmos.DrawWireSphere(currentNode, 0.3f);
        }
        Gizmos.DrawLine(nodes[0].position, nodes[nodes.Count - 1].position);
        Gizmos.DrawWireSphere(nodes[0].position, 0.3f);
    }
}
