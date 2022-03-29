using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraSwitch : MonoBehaviour
{
    public GameObject cam1;
    public GameObject cam2;
    public GameObject cam3;
    public GameObject cam4;
    public GameObject cam5;

    // Update is called once per frame
    void Update() {
        if (Input.GetButtonDown("1key"))
        {   
            cam1.SetActive(true);
            cam2.SetActive(false);
            cam3.SetActive(false);
            cam4.SetActive(false);        
            cam5.SetActive(false);        
        }
        if (Input.GetButtonDown("2key"))
        {   
            cam2.SetActive(true);
            cam1.SetActive(false);
            cam3.SetActive(false);
            cam4.SetActive(false);        
            cam5.SetActive(false);        
        }
        if (Input.GetButtonDown("3key"))
        {   
            cam3.SetActive(true);
            cam1.SetActive(false);
            cam2.SetActive(false);
            cam4.SetActive(false);        
            cam5.SetActive(false);        
        }
        if (Input.GetButtonDown("4key"))
        {   
            cam4.SetActive(true);
            cam1.SetActive(false);
            cam2.SetActive(false);
            cam3.SetActive(false);
            cam5.SetActive(false);        
        }
        if (Input.GetButtonDown("5key"))
        {   
            cam5.SetActive(true);
            cam1.SetActive(false);
            cam2.SetActive(false);
            cam3.SetActive(false);
            cam4.SetActive(false);        
        }
    }
}
