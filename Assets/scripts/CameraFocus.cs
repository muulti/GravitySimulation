using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFocus : MonoBehaviour
{
    // Start is called before the first frame update
    public Camera mainCamera;
    public void SetCameraSize(float z)
    {
        mainCamera.orthographicSize=z;
    }
    public void SetCameraFocus(Vector2 focus)
    {
        transform.position=new Vector3(focus.x,focus.y,-11);
    }
    public void SetCameraFocus()
    {
        transform.position=new Vector3(0,0,-10);
    }
}
