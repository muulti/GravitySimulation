using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;

public class Planet : MonoBehaviour
{
    public GameObject thisBody;
    public Vector2 currentVelocity;

    [Range(1, 5000000)]
    public float mass;
    [Range(.1f, 1000)]
    public float radius;
    // Start is called before the first frame update
    void Start()
    {
        thisBody.AddComponent<LineRenderer>().widthMultiplier=.005f;
        //thisBody.GetComponent<LineRenderer>().useWorldSpace=false;
        FixMassAndScale(mass,radius);       //Fixes mass in "RigidBody" and radius in "localScale"
    }



    public void UpdateVel(Vector2 forceVector)
    {
        currentVelocity += forceVector;
    }
    public Vector2 GetVel()
    {
        return currentVelocity;
    }

    public void UpdatePosition(float t)
    {
        thisBody.GetComponent<Rigidbody2D>().position += currentVelocity*t;    //So nothing breaks..
    }
    public void SetPosition(Vector2 p)
    {
        thisBody.GetComponent<Rigidbody2D>().position = p;   
    }
    public void SetDimension()
    {FixMassAndScale(mass,radius);}
    public void SetVelocity(Vector2 v)
    {
        currentVelocity = v; 
    }


    // Update is called once per frame
    void Update()
    {
    }

    void FixMassAndScale(float m, float r)
    {
        float s = r/2;
        thisBody.GetComponent<Rigidbody2D>().mass = m;
        thisBody.transform.localScale = new Vector3(s, s, s);
    }
}
