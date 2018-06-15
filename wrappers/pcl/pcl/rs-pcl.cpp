// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>


#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>



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
void draw_pointcloud_rgb(window& app, state& app_state, const std::vector<pcl_rgb_ptr>& points);
SYSTEMTIME operator-(const SYSTEMTIME& pSr, const SYSTEMTIME& pSl);

pcl_rgb_ptr points_to_pcl(const rs2::points& points, rs2::video_frame frame)
{
    pcl_rgb_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());


	auto vertices = points.get_vertices();              // get vertices
	auto tex_coords = points.get_texture_coordinates(); // and texture coordinates


	auto color = (uint8_t *)frame.get_data();			//get the video frame texture

	auto width = frame.get_width();
	auto height = frame.get_height();
	

#pragma omp parallel for 
	for (int i = 0; i < points.size(); i++)
	{
		if (vertices[i].z)
		{
			// upload the point and texture coordinates only for points we have depth data for
			cloud->points[i].x = vertices[i].x;
			cloud->points[i].y = vertices[i].y;

			cloud->points[i].z = vertices[i].z;

			auto coords = tex_coords[i];
			auto color_x = (int)((coords.u ) * frame.get_width());
			auto color_y = (int)((coords.v)  * frame.get_height());


			if (color_x < 0 || color_x > width
				|| color_y < 0 || color_y > height) {
				//this point falls outside the video frame, so we set the z to zero. We'll filter out all z= 0 later on.
				cloud->points[i].z = 0;
				continue;

			}
			
			cloud->points[i].r = *(color + color_x * 3 + color_y * width * 3 + 0);
			cloud->points[i].g = *(color + color_x * 3 + color_y * width * 3 + 1);
			cloud->points[i].b = *(color + color_x * 3 + color_y * width * 3 + 2);


		}
	}

    return cloud;
}


float3 colors[] { { 0.8f, 0.1f, 0.3f }, 
                  { 0.1f, 0.9f, 0.5f },
                };

int frameCounter = 0;
int numberOfFramesToSave = 100;
string saveCloudLocation = ".\\TestData\\";
bool loadCloudFrames = true;

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense PCL Pointcloud Example");
	// Construct an object to manage view state
	state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	if (!loadCloudFrames)
	{
		// Start streaming with default recommended configuration
		pipe.start();
	}

	rs2::frameset frames;
	rs2::video_frame color = nullptr;
	rs2::depth_frame depth = nullptr;
	pcl_ptr pcl_points;

	//compressionstatistics
	bool showStatistics = true;

	// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
	//pcl::io::compression_Profiles_e compressionProfileRGB = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	pcl::io::compression_Profiles_e compressionProfileRGB = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;

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

	

	SYSTEMTIME startTime;
	SYSTEMTIME endTime;
	GetLocalTime(&startTime);
	GetLocalTime(&endTime);
//	int cumulativeDataLength = 0;

    while (app) // Application still alive?
    {
		string fileName = boost::filesystem::canonical(saveCloudLocation, boost::filesystem::current_path()).string() + "\\PointCloudFrame" + boost::lexical_cast<string>(frameCounter % numberOfFramesToSave) + ".pcd";
		
		pcl_rgb_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (loadCloudFrames)
		{
			if (!boost::filesystem::exists(fileName))
			{
				cerr << "Can't find cloud file " << fileName << endl;
				getline(cin, string());
				return EXIT_FAILURE;
			}
			pcl::io::loadPCDFile(fileName, *cloud_filtered);
		}
		else
		{
			// Wait for the next set of frames from the camera
			frames = pipe.wait_for_frames();
			depth = frames.get_depth_frame();

			// Generate the pointcloud and texture mappings
			points = pc.calculate(depth);

			color = frames.get_color_frame();

			// Tell pointcloud object to map to this color frame
			pc.map_to(color);
			auto pcl_rgb_points = points_to_pcl(points, color);

			// filter point cloud
			pcl::PassThrough<pcl::PointXYZRGB> pass;
			pass.setInputCloud(pcl_rgb_points);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.2, 1.0);
			pass.filter(*cloud_filtered);
		}
#if TRUE
		PointCloudEncoderRGB->switchBuffers();
#else
		int filteredSize = cloud_filtered->size();
		if (cloudSize == 0)
			cloudSize = filteredSize;
		else
		{
			float differenceFactor = std::abs((float)(filteredSize - cloudSize) / cloudSize);
			//cout << "differenceFactor: " << differenceFactor << "\n";
			if (differenceFactor > 0.1 || (frameCounter % 5) == 0)
			{
				cloudSize = filteredSize;
				PointCloudEncoderRGB->switchBuffers();
				cout << "switched cloudsize, pointcloud size difference factor is " << differenceFactor << "\n";
			}
			else
			{
				cloud_filtered->resize(cloudSize);
			}
		}
#endif

		// compress 
		//std::stringstream compressedData;
		//PointCloudEncoderRGB->encodePointCloud(cloud_filtered, compressedData);

		//// calculate data size
		//std::string compressedDataString = compressedData.str();
		//int frameDataLength = compressedDataString.length() / 1024; //get the datalength in kilobytes
		//															//cout << "compresseddata offset: " << frameDataLength << "\n";
		//cumulativeDataLength += frameDataLength;
		//GetLocalTime(&endTime);

		//if ((endTime - startTime).wSecond > 0)
		//{
		//	startTime = endTime;
		//	cout << "Total data transfer over last second was " << cumulativeDataLength << "KB\n";
		//	cumulativeDataLength = 0;
		//}


		//pcl_rgb_ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		//PointCloudDecoderRGB->decodePointCloud(compressedData, decompressed_cloud);

		/*
		 compressedData.seekp(0, ios::end);
		 stringstream::pos_type offset = compressedData.tellp();
		 std::cout << offset << std::endl;


		 
		 calc_compression_index++;
		 if (calc_compression_index == calc_compression_rate) {
			 compression_data_size += (int)offset;
			 std::cout << compression_data_size / calc_compression_rate << "Bytes" << std::endl;
			 compression_data_size = 0;
			 calc_compression_index = 0;
		 }
		 */

		//don't save files to disk if we're loading them from disk, unneccesary.
		if (!loadCloudFrames && frameCounter < numberOfFramesToSave)
		{
			pcl::io::savePCDFileBinaryCompressed(fileName, *cloud_filtered);
		}
		// draw point cloud
		std::vector<pcl_rgb_ptr> layers_rgb;
		//layers_rgb.push_back(pcl_rgb_points);
		layers_rgb.push_back(cloud_filtered);
		//layers_rgb.push_back(decompressed_cloud);
		draw_pointcloud_rgb(app, app_state, layers_rgb);
		frameCounter++;
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

