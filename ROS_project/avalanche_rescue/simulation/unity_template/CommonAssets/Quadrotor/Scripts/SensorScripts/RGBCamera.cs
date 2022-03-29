﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;



public class RGBCamera : ICommandable {
	RenderTexture cameraImage;
	public RenderTexture outputImage;
	Texture2D myTexture2D;
	public Camera camera;
	
	public int width = 320;
	public int height = 240;
	public float fieldOfView = 90.0f;
	public float farClipPlane = 65.535f;
	public float targetFrameRate = 30.0f;
	public bool monochrome = false;
	const uint type = 1;
	float time_last_image_sent = 0.0f;
	protected bool send_image = false;
	
	protected RenderTextureReadWrite render_texture_read_write = RenderTextureReadWrite.Default;

	void Initialize() {
        cameraImage = new RenderTexture(width, height, 24, RenderTextureFormat.DefaultHDR, render_texture_read_write);
        outputImage = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, render_texture_read_write);

		if(camera == null) {
			camera = GetComponent<Camera>();
		}
		camera.farClipPlane = farClipPlane;
		camera.depthTextureMode = DepthTextureMode.Depth;
		camera.targetTexture = cameraImage;

		myTexture2D = new Texture2D (cameraImage.width, cameraImage.height, TextureFormat.RGB24, false);
	}

	void OnPreRender() {
		camera.farClipPlane = farClipPlane;
		camera.fieldOfView = fieldOfView;
	}	

	void OnRenderImage(RenderTexture src, RenderTexture dest) {
		Graphics.Blit(src, outputImage);
		if (send_image) {
            server.SendHeader(type, full_name, time_server.GetFrameTicks());
			SendImage(outputImage);
			send_image = false;
		}
	}

	protected void SendImage(RenderTexture tex) {
		RenderTexture.active = tex;		                                      
		myTexture2D.ReadPixels (new Rect (0, 0, tex.width, tex.height), 0, 0, false);
		byte[] imageBytes = myTexture2D.GetRawTextureData ();
		server.SendData(monochrome ? 1 : 0);
        server.SendData(fieldOfView);
        server.SendData(farClipPlane);
        server.SendData(tex.width);
        server.SendData(tex.height);
    	server.SendData(imageBytes);
	}

	void Awake () {
		Initialize();
	}

	void Update () {
		float time_since_last_image_sent = Time.time - time_last_image_sent;
		if (time_since_last_image_sent > (1/targetFrameRate - Time.fixedDeltaTime * 0.5f)) {
			send_image = true;
			time_last_image_sent = Time.time;
		}
	}	
}
