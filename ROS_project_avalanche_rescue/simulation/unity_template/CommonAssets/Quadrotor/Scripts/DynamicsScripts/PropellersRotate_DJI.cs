using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PropellersRotate_DJI : MonoBehaviour
{
float rotationAmount = 1000.0f;
	void Update() {
		//transform.Rotate(Time.deltaTime, 0, 0);
		//Vector3 RotationAxis = new Vector3 [0, 1, 0];
		transform.Rotate(Vector3.up, rotationAmount*Time.deltaTime);
		//transform.Rotate(0, rotationAmount*Time.deltaTime, 0, Space.Self);
		//transform.Rotate(0, rotationAmount*Time.deltaTime,0, Space.Self);
	}
}

