using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;

public class SpaceTimeManager : MonoBehaviour               //Basically the script of the whole simulation
{
    GameObject[] bodies;
    public CameraFocus mainCamera;
    [Range(.1f, 10)]
    public float zoom;
    List<List<Vector2>> points;
    List<List<Vector2>> pointsVel;
     [Range(.0001f, .5f)]
    public float gConstant;
    public bool start;
    [Range(0, 10000)]
    public int numberOfIterations;
    //[Range(.1f, 3)]
    //public float timeScale;
    [Range(.001f, .5f)]
    public float timeStep;
    public bool showLines;
    public bool relativeToBody;
    int selectedBody;
    
    void Start()
    {
        StateBools();
        StateNumbers();
        StateOthers();        
    }
    void Update()
    {
        UpdateLineRendererOrigin();
        if (relativeToBody)
        {
            UpdateTrajectoryLines(selectedBody);
        }
        else
        {
            UpdateTrajectoryLines();
        }
        UpdateSelectedBodyVar();
        mainCamera.SetCameraSize(zoom);
        if (start==true)
        {
            UpdateBodyVelocities(bodies);
            UpdateBodyPositions(bodies);
        }
        FocusCamera(relativeToBody);

        if (showLines) 
        {EnableTrajectories();}
        else
        {DisableTrajectories();}
    }
    void EnableTrajectories()
    {
        foreach (GameObject b in bodies)
        {
            b.GetComponent<LineRenderer>().enabled=true;
        }
    }
    void DisableTrajectories()
    {
        foreach (GameObject b in bodies)
        {
            b.GetComponent<LineRenderer>().enabled=false;
        }
    }

    public float NewtonForceEquation(float force, float mass)
    {
        return (force/mass);
    }
    public float NewtonGravitatoryEquation(float mass1,float mass2,float distance)
    {
        float denominator=distance*distance;
        float numerator =gConstant*mass2*mass1;
        return (numerator/denominator);

    }
    public float QuickGravitatoryForce(float mass2,float distance)
    {
        float denominator=distance*distance;
        float numerator =gConstant*mass2;   //*mass1* missing
        return numerator/denominator;
    }
    public Vector2 AddForceToVector(float force, Vector2 vectorD)
    {
        return vectorD*force;
    }

    void UpdateBodyVelocities(GameObject[] otherBodies)
    {
        foreach(GameObject body in otherBodies)
        {
            foreach(GameObject otherBody in otherBodies)
            {
                if (body!=otherBody)
                {
                    //Getting parameters
                    //float bodyMass = body.GetComponent<Rigidbody2D>().mass;
                    float otherBodyMass =otherBody.GetComponent<Rigidbody2D>().mass;
                    Vector2 bodyPos = body.GetComponent<Rigidbody2D>().position;
                    Vector2 otherBodyPos = otherBody.GetComponent<Rigidbody2D>().position;
                    float distance = Vector2.Distance(bodyPos,otherBodyPos);

                    //Computing force vector and updating position
                    Vector2 direction =(otherBodyPos-bodyPos).normalized;
                    float force = QuickGravitatoryForce(otherBodyMass, distance);
                    //print(body.name +" force: "+ force);
                    body.GetComponent<Planet>().UpdateVel(AddForceToVector(force,direction));
                }
            }
        }
    }
    void UpdateLineRendererOrigin()
    {
        int bodyCount = bodies.Length;
        for (int i = 0; i < bodyCount; i++)
        {
            bodies[i].GetComponent<Planet>().SetDimension();
            points[0][i] = bodies[i].GetComponent<Rigidbody2D>().position;
            pointsVel[0][i] = bodies[i].GetComponent<Planet>().GetVel();
        }
    }
    void AddIteration(int n)    //Adds a list of n vectors2D to Points and PointsVel 
    {
        List<Vector2> auxVectorList1 = new List<Vector2>(); 
        List<Vector2> auxVectorList2 = new List<Vector2>(); 
        for (int k = 0; k < n; k++)
        {
            auxVectorList1.Add(Vector2.zero);
            auxVectorList2.Add(Vector2.zero);
        }
        points.Add(auxVectorList1);
        pointsVel.Add(auxVectorList2);
    }
    void SubstractIteration()   //Removes the last elements of both lists
    {
        points.RemoveAt(points.Count-1);
        pointsVel.RemoveAt(points.Count-1);
    }
    void UpdateTrajectoryLines()
    {
        int bodyCount=bodies.Length;
        while (points.Count-1>numberOfIterations){SubstractIteration();}
        while (points.Count-1<numberOfIterations){AddIteration(bodyCount);}
        for (int it = 1; it < numberOfIterations; it++)                            //Each iteration
        {
            for (int i = 0; i < bodyCount; i++)                            //Computing body movement from other bodies influence
            {
                bodies[i].GetComponent<LineRenderer>().positionCount=numberOfIterations-1;
                //bodies[i].GetComponent<LineRenderer>().GetInde
                Vector2 newVel = pointsVel[it-1][i];
                Vector2 thisPos = points[it-1][i];
                for (int j = 0; j < bodyCount; j++)                         //Influence of n bodies over specific
                {
                    //print("[ i:"+i+" // j:"+j+" ]");
                    if (i!=j)
                    {
                        float mass = bodies[j].GetComponent<Rigidbody2D>().mass;
                        Vector2 otherPos = points[it-1][j];

                        Vector2 dir = (otherPos-thisPos).normalized;
                        Vector2 force = dir*QuickGravitatoryForce(mass, Vector2.Distance(thisPos,otherPos));
                        
                        newVel +=force;     
                    }
                }
                pointsVel[it][i] = newVel;
                Vector2 newPos = thisPos+newVel*timeStep;
                points[it][i] = newPos;
                bodies[i].GetComponent<LineRenderer>().SetPosition(it-1,newPos);
            }
        }
    }  

    void UpdateTrajectoryLines(int index)
    {
        int bodyCount=bodies.Length;
        while (points.Count-1>numberOfIterations){SubstractIteration();}
        while (points.Count-1<numberOfIterations){AddIteration(bodyCount);}
        for (int it = 1; it < numberOfIterations; it++)                            //Each iteration
        {
            for (int i = 0; i < bodyCount; i++)                            //Computing body movement from other bodies influence
            {
                bodies[i].GetComponent<LineRenderer>().positionCount=numberOfIterations-1;
                //bodies[i].GetComponent<LineRenderer>().GetInde
                Vector2 newVel = pointsVel[it-1][i];
                Vector2 thisPos = points[it-1][i];
                for (int j = 0; j < bodyCount; j++)                         //Influence of n bodies over specific
                {
                    //print("[ i:"+i+" // j:"+j+" ]");
                    if (i!=j)
                    {
                        float mass = bodies[j].GetComponent<Rigidbody2D>().mass;
                        Vector2 otherPos = points[it-1][j];

                        Vector2 dir = (otherPos-thisPos).normalized;
                        Vector2 force = dir*QuickGravitatoryForce(mass, Vector2.Distance(thisPos,otherPos));
                        
                        newVel +=force;     
                    }
                }
                pointsVel[it][i] = newVel;
                Vector2 newPos = thisPos+newVel*timeStep;
                points[it][i] = newPos;
            }
        }
        for (int it = 1; it < numberOfIterations; it++)
        {
            for (int i = 0; i < bodyCount; i++)                            //FUCK newton
            {
                Vector2 movementSubstracted = (points[it][index]+points[0][index])-points[it][i] ;
                bodies[i].GetComponent<LineRenderer>().SetPosition(it-1,movementSubstracted);
            }
        }
        
    } 

         
    void UpdateBodyPositions(GameObject[] bodies)
    {
        foreach (GameObject body in bodies)
        {
            body.GetComponent<Planet>().UpdatePosition(timeStep);
        }
    }
    void FocusCamera(bool relative)
    {
        if (relative)
        {
            mainCamera.SetCameraFocus(bodies[selectedBody].transform.position);
        }
        else
        {
            mainCamera.SetCameraFocus();
        }
    }
    void StateBools()
    {
        relativeToBody=false;
        showLines=true;
        start=false;
    }
    void StateNumbers()                                
    {
        zoom=1;
        timeStep=.005f;
        selectedBody=0;
        //camera.orthographicSize=zoom;
        gConstant=.005f;                                 //Sorry Newton        
    }
    void StateOthers()
    {
        //Array of bodies
        bodies = GameObject.FindGameObjectsWithTag("Body");

        //First iteration of trajectory lines
        InitialConditions();
    }
    void InitialConditions()
    {
        points=new List<List<Vector2>>();
        pointsVel=new List<List<Vector2>>();
        int count = bodies.Length;
        AddIteration(count);
        FixedInitalConditions();
    }
    void LineRendererSetUp(int n)
    {
        foreach (GameObject b in bodies)
        {
            //b.GetComponent<LineRenderer>().positionCount=numberOfIterations-2;
        }
    }

    void UpdateSelectedBodyVar()
    {
        if (Input.GetAxis("Horizontal")>0)
        {
            if (selectedBody>=bodies.Length)
            {
                selectedBody=0;
            }
            else{selectedBody++;}
        }
    }
    void FixedInitalConditions()                                //Initial conditions "Periodic solution to 3-body problem computed by Carles Simó"
    {
        Vector2 velocity = new Vector2(-0.93240737f, -0.86473146f);
        Vector2 position = new Vector2(0.93240737f, -0.24308753f);

        bodies[2].GetComponent<Planet>().SetPosition(position);
        bodies[2].GetComponent<Planet>().SetVelocity(velocity/-2);

        bodies[1].GetComponent<Planet>().SetPosition(position*-1);
        bodies[1].GetComponent<Planet>().SetVelocity(velocity/-2);

        bodies[0].GetComponent<Planet>().SetPosition(Vector2.zero);
        bodies[0].GetComponent<Planet>().SetVelocity(velocity);
    }
}
