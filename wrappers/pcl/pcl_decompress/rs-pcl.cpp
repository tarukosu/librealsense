// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.



#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering



#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


#include <pcl/io/openni2_grabber.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/compression/octree_pointcloud_compression.h>


// socket connection

#include <stdio.h>
// socket
#define FD_SETSIZE 32
#define BUFFER 512

#include <windows.h>
#include <winsock2.h>


SYSTEMTIME operator-(const SYSTEMTIME& pSr, const SYSTEMTIME& pSl);

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; 
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pcl_rgb_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);
void draw_pointcloud_rgb(window& app, state& app_state, const std::vector<pcl_rgb_ptr>& points);


float3 colors[] { { 0.8f, 0.1f, 0.3f }, 
                  { 0.1f, 0.9f, 0.5f },
                };

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	//rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	//rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	//rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	//pipe.start();

	/*

	rs2::frameset frames;
	rs2::video_frame color = nullptr;
	rs2::depth_frame depth = nullptr;
	pcl_ptr pcl_points;
	*/

	//compression
	bool showStatistics = true;
//	bool showStatistics = false;

	// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
	//pcl::io::compression_Profiles_e compressionProfileRGB = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	pcl::io::compression_Profiles_e compressionProfileRGB = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

	// instantiate point cloud compression for encoding and decoding
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfileRGB, showStatistics);
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfileRGB, showStatistics, 0.001, 0.01, false, 30U);
    //auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfileRGB, showStatistics, 0.001, 0.01, true, 1U);
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfileRGB, showStatistics);
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.001, 0.01, false, 1U);
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.001, 0.01, false, 0U);
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.001, 0.01, true, 10U);x
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.0005, 0.003, true, 10U);
	///auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.0005, 0.003, true, 1U);
	auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, showStatistics, 0.0002, 0.003, true, 0U); //GOOD!

//	auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(compressionProfileRGB, showStatistics);

	auto PointCloudDecoderRGB = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
	//auto 	PointCloudEncoderRGB = new pcl::io::OctreePointCloudCompression<pcl_rgb_ptr>(compressionProfileRGB, showStatistics);
//	auto 	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile, showStatistics);
//	auto PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();


	int filtered_size = 0;
	int cloudSize = 0;
	int cumulativeDataLength = 0;


	int calc_compression_index = 0;
	int calc_compression_rate = 30;
	int compression_data_size = 0;


	int switch_buffer_counter = 0;
	int switch_buffer_rate = 5;

	int frameCounter = 0;

	SYSTEMTIME startTime;
	SYSTEMTIME endTime;
	GetLocalTime(&startTime);
	GetLocalTime(&endTime);
//	int cumulativeDataLength = 0;


	/*
	// socket connection
	WSADATA wsaData;
	SOCKET sock;
	struct sockaddr_in addr;
	struct timeval t_val = { 0, 1000 };
	int select_ret;
	char buf[BUFFER];
	fd_set fds, readfds;
	int accept_list[FD_SETSIZE];
	int accept_num = 0;

	// accept_list 初期化
	for (int i = 0; i<FD_SETSIZE; i++) {
		accept_list[i] = INVALID_SOCKET;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(48003);
	addr.sin_addr.S_un.S_addr = INADDR_ANY;

	WSAStartup(MAKEWORD(2, 0), &wsaData);

	if (
		(sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET ||
		::bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR ||
		listen(sock, FD_SETSIZE) == SOCKET_ERROR
		) {
		printf("socket error\n");
		return 1;
	}
	FD_ZERO(&readfds);
	FD_SET(sock, &readfds);
	// end socket connection
	*/

	struct sockaddr_in address;
	int sock = 0, valread;
	struct sockaddr_in serv_addr;
	char *hello = " ";

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	int PORT = 8080;
	//PORT = 48003;

	serv_addr.sin_port = htons(PORT);
	//inet_aton("127.0.0.1", &addr.sin_addr);
	serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

	/*
	// Convert IPv4 and IPv6 addresses from text to binary form
	if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}
	*/

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}
	/*
	valread = recv(sock, buffer, 1024, 0);
	printf("%s\n", buffer);
	return 0;
	*/

	char buffer[1024 * 100] = { 0 };

    while (app) // Application still alive?
    {
		send(sock, hello, strlen(hello), 0);
		printf("Hello message sent\n");

		valread = recv(sock, buffer, 4, 0);

		/*
		printf("%s\n", &buffer[4]);
		printf("%c\n", buffer[3]);
		printf("%c\n", buffer[2]);
		printf("%c\n", buffer[1]);
		*/
		int data_size = 0;
		data_size += ((unsigned char)buffer[3] << 24);
		data_size += ((unsigned char)buffer[2] << 16);
		data_size += ((unsigned char)buffer[1] << 8);
		data_size += (unsigned char)buffer[0];

		printf("datasize: %d\n", data_size);

		std::stringstream compressedData;
		int read_data = 0;
		int read_size = data_size;
		while (true) {
			read_size = std::min(data_size - read_data, 1024 * 100);
			valread = recv(sock, buffer, read_size, 0);
			compressedData.write(buffer, valread);

			read_data += valread;
			if (read_data >= data_size) {
				break;
			}
		}

		pcl_rgb_ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		PointCloudDecoderRGB->decodePointCloud(compressedData, decompressed_cloud);

		// draw point cloud
		std::vector<pcl_rgb_ptr> layers_rgb;
		//layers_rgb.push_back(pcl_rgb_points);
		//layers_rgb.push_back(cloud_filtered);
		layers_rgb.push_back(decompressed_cloud);

		draw_pointcloud_rgb(app, app_state, layers_rgb);

		/*
		//byte[] compressed_data;
		// calculate data size
		//std::string compressedDataString = compressedData.str();
		std::string compressedDataString = "";
		int frameDataLength = compressedDataString.length() / 1024; //get the datalength in kilobytes
																	//cout << "compresseddata offset: " << frameDataLength << "\n";
		cumulativeDataLength += frameDataLength;
		GetLocalTime(&endTime);

		if ((endTime - startTime).wSecond > 0)
		{
			startTime = endTime;
			cout << "Total data transfer over last second was " << cumulativeDataLength << "KB\n";
			cumulativeDataLength = 0;
		}


		pcl_rgb_ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		PointCloudDecoderRGB->decodePointCloud(compressedData, decompressed_cloud);

		// draw point cloud
		std::vector<pcl_rgb_ptr> layers_rgb;
		//layers_rgb.push_back(pcl_rgb_points);
		//layers_rgb.push_back(cloud_filtered);
		layers_rgb.push_back(decompressed_cloud);

		draw_pointcloud_rgb(app, app_state, layers_rgb);
		*/
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)
            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}


void draw_pointcloud_rgb(window& app, state& app_state, const std::vector<pcl_rgb_ptr>& points)
{
	// OpenGL commands that prep screen for the pointcloud
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(), height = app.height();

	glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60, width / height, 0.01f, 10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

	glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
	glRotated(app_state.pitch, 1, 0, 0);
	glRotated(app_state.yaw, 0, 1, 0);
	glTranslatef(0, 0, -0.5f);

	glPointSize(width / 640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;

	for (auto&& pc : points)
	{
		//auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

		glBegin(GL_POINTS);
		//glColor3f(c.x, c.y, c.z);

		/* this segment actually prints the pointcloud */
		for (int i = 0; i < pc->points.size(); i++)
		{
			auto&& p = pc->points[i];
			if (p.z)
			{
				// upload the point and texture coordinates only for points we have depth data for
				glVertex3f(p.x, p.y, p.z);

				glColor3f(p.r / 255.0, p.g/ 255.0, p.b/255.0);
			}
		}

		glEnd();
	}

	// OpenGL cleanup
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

/*
void send_point_cloud() {
	int i;
	// ポート番号，ソケット
	int srcSocket;  // 自分
	int dstSocket;  // 相手

	// sockaddr_in 構造体
	struct sockaddr_in srcAddr;
	struct sockaddr_in dstAddr;
	int dstAddrSize = sizeof(dstAddr);
	int status;
	// 各種パラメータ
	int numrcv;
	char buffer[1024];

	// Windows の場合
	WSADATA data;
	WSAStartup(MAKEWORD(2,0), &data);
	// sockaddr_in 構造体のセット
	memset(&srcAddr, 0, sizeof(srcAddr));
	srcAddr.sin_port = htons(PORT);
	srcAddr.sin_family = AF_INET;
	srcAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	// ソケットの生成（ストリーム型）
	srcSocket = socket(AF_INET, SOCK_STREAM, 0);
  	// ソケットのバインド
	bind(srcSocket, (struct sockaddr *) &srcAddr, sizeof(srcAddr));
  	// 接続の許可
	listen(srcSocket, 1);

}*/

SYSTEMTIME operator-(const SYSTEMTIME& pSr, const SYSTEMTIME& pSl)
{
	SYSTEMTIME t_res;
	FILETIME v_ftime;
	ULARGE_INTEGER v_ui;
	__int64 v_right, v_left, v_res;
	SystemTimeToFileTime(&pSr, &v_ftime);
	v_ui.LowPart = v_ftime.dwLowDateTime;
	v_ui.HighPart = v_ftime.dwHighDateTime;
	v_right = v_ui.QuadPart;

	SystemTimeToFileTime(&pSl, &v_ftime);
	v_ui.LowPart = v_ftime.dwLowDateTime;
	v_ui.HighPart = v_ftime.dwHighDateTime;
	v_left = v_ui.QuadPart;

	v_res = v_right - v_left;

	v_ui.QuadPart = v_res;
	v_ftime.dwLowDateTime = v_ui.LowPart;
	v_ftime.dwHighDateTime = v_ui.HighPart;
	FileTimeToSystemTime(&v_ftime, &t_res);
	return t_res;
}


//d_intrinsics = depth_stream.asrs2::video_stream_profile().get_intrinsics();
//c_intrinsics = color_stream.asrs2::video_stream_profile().get_intrinsics();
//d_to_c_extrinsics = depth_stream.get_extrinsics_to(color_stream);

