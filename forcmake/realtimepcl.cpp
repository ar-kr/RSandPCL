// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;

// Prototypes
//void Load_PCDFile(void);
//bool userInput(void);
//void cloudViewer(void);

// Global Variables
string cloudFile; // .pcd file name
string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

		   //======================================================
		   // RGB Texture
		   // - Function is utilized to extract the RGB data from
		   // a single point return R, G, and B values. 
		   // Normals are stored as RGB components and
		   // correspond to the specific depth (XYZ) coordinate.
		   // By taking these normals and converting them to
		   // texture coordinates, the RGB components can be
		   // "mapped" to each individual point (XYZ).
		   //======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
	// Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels

									   // Normals to Texture Coordinates conversion
	int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

	// RGB components to save in tuple
	int NT1 = New_Texture[Text_Index];
	int NT2 = New_Texture[Text_Index + 1];
	int NT3 = New_Texture[Text_Index + 2];

	return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color) {

	// Object Declaration (Point Cloud)
	cloud_pointer cloud(new point_cloud);

	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

	//================================
	// PCL Cloud Object Configuration
	//================================
	// Convert data captured from Realsense camera to Point Cloud
	auto sp = points.get_profile().as<rs2::video_stream_profile>();

	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); i++)
	{
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

		// Obtain color texture for specific point
		RGB_Color = RGB_Texture(color, Texture_Coord[i]);

		// Mapping Color (BGR due to Camera Model)
		cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
		cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
		cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

	}

	return cloud; // PCL RGB Point Cloud generated
}

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense Pointcloud Example");
	// Construct an object to manage view state
	glfw_state app_state;
	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, app_state);

	// Colorizer is used to visualize depth data
	rs2::colorizer color_map;
	// Use black to white color map
	color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
	// Decimation filter reduces the amount of data (while preserving best samples)
	rs2::decimation_filter dec;
	// If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
	// but you can also increase the following parameter to decimate depth more (reducing quality)
	dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	// Define transformations from and to Disparity domain
	rs2::disparity_transform depth2disparity;
	rs2::disparity_transform disparity2depth(false);
	// Define spatial filter (edge-preserving)
	rs2::spatial_filter spat;
	// Enable hole-filling
	// Hole filling is an agressive heuristic and it gets the depth wrong many times
	// However, this demo is not built to handle holes
	// (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
	spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
											   // Define temporal filter
	rs2::temporal_filter temp;
	// Spatially align all streams to depth viewport
	// We do this because:
	//   a. Usually depth has wider FOV, and we only really need depth for this demo
	//   b. We don't want to introduce new holes
	rs2::align align_to(RS2_STREAM_DEPTH);


	/*--pcl--*/

	//======================
	// Variable Declaration
	//======================
	bool captureLoop = true; // Loop control for generating point clouds

	//====================
	// Object Declaration						 
	//====================						 						 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;

	/*--pcl--*/


	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	// Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;

	//======================
	// Stream configuration
	//======================
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

	rs2::pipeline_profile selection = pipe.start(cfg);

	rs2::device selected_device = selection.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
	{
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
	}
	if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
	{
		// Query min and max values:
		auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
	}


	// After initial post-processing, frames will flow into this queue:
	rs2::frame_queue postprocessed_frames;

	// Alive boolean will signal the worker threads to finish-up
	std::atomic_bool alive{ true };

	// Video-processing thread will fetch frames from the camera,
	// apply post-processing and send the result to the main thread for rendering
	// It recieves synchronized (but not spatially aligned) pairs
	// and outputs synchronized and aligned pairs
	std::thread video_processing_thread([&]() {
		while (alive)
		{
			// rs-measure
			/*
			// Fetch frames from the pipeline and send them for processing
			rs2::frameset data;
			if (pipe.poll_for_frames(&data))
			{
				// First make the frames spatially aligned
				data = data.apply_filter(align_to);

				// Decimation will reduce the resultion of the depth image,
				// closing small holes and speeding-up the algorithm
				data = data.apply_filter(dec);

				// To make sure far-away objects are filtered proportionally
				// we try to switch to disparity domain
				data = data.apply_filter(depth2disparity);

				// Apply spatial filtering
				data = data.apply_filter(spat);

				// Apply temporal filtering
				data = data.apply_filter(temp);

				// If we are in disparity domain, switch back to depth
				data = data.apply_filter(disparity2depth);

				//// Apply color map for visualization of depth
				data = data.apply_filter(color_map);

				// Send resulting frames for visualization in the main thread
				postprocessed_frames.enqueue(data);
			}
			*/

			// Wait for frames from the camera to settle
			//for (int i = 0; i < 30; i++) {
			//	auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
			//}

			// Capture a single frame and obtain depth + RGB values from it    
			auto frames = pipe.wait_for_frames();
			auto depth = frames.get_depth_frame();
			auto RGB = frames.get_color_frame();

			// Map Color texture to each point
			pc.map_to(RGB);

			// Generate Point Cloud
			auto points = pc.calculate(depth);

			// Convert generated Point Cloud to PCL Formatting
			cloud_pointer cloud = PCL_Conversion(points, RGB);

			//========================================
			// Filter PointCloud (PassThrough Method)
			//========================================
			//pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
			//Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
			//Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
			//Cloud_Filter.setFilterLimits(0.0, 1.0);      // Set accepted interval values
			//Cloud_Filter.filter(*newCloud);              // Filtered Cloud Outputted

			cloudFile = "CF" + to_string(i) + ".pcd";

			//==============================
			// Write PC to .pcd File Format
			//==============================
			// Take Cloud Data and write to .PCD File Format
			//cout << "Generating PCD Point Cloud File... " << endl;
			pcl::io::savePCDFileBinary(cloudFile, *cloud); // Input cloud to be saved to .pcd
			cout << cloudFile << " successfully generated. " << endl;

			//Load generated PCD file for viewing
			//Load_PCDFile();
			i++; // Increment File Name
		}
	});
	
	video_processing_thread.join();

	rs2::frameset current_frameset;
	while (app) // Application still alive?
	{
		// rs-pointcloud
		
		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();

		auto color = frames.get_color_frame();

		// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
		if (!color)
			color = frames.get_infrared_frame();

		// Tell pointcloud object to map to this color frame
		pc.map_to(color);

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		// Upload the color frame to OpenGL
		app_state.tex.upload(color);

		// Draw the pointcloud
		draw_pointcloud(app.width(), app.height(), app_state, points);
		

		// Fetch the latest available post-processed frameset
		postprocessed_frames.poll_for_frame(&current_frameset);

		// rs-measure
		/*
		if (current_frameset)
		{
			
			auto depth = current_frameset.get_depth_frame();
			auto color = current_frameset.get_color_frame();
			auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

			glEnable(GL_BLEND);
			// Use the Alpha channel for blending
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			// First render the colorized depth image
			depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });

			// Render the color frame (since we have selected RGBA format
			// pixels out of FOV will appear transparent)
			color_image.render(color, { 0, 0, app.width(), app.height() });

			// Render the simple pythagorean distance
			render_simple_distance(depth, app_state, app);

			// Render the ruler
			app_state.ruler_start.render(app);
			app_state.ruler_end.render(app);

			glColor3f(1.f, 1.f, 1.f);
			glDisable(GL_BLEND);
			
		}
		*/
	}

	// Signal threads to finish and wait until they do
	alive = false;
	video_processing_thread.join();

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