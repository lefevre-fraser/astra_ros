#pragma once

#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <pcl/pcl_macros.h>
#include <Eigen/Core>
#include <astra/astra.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <vector>

#include "common.h"

class AstraListener : public astra::FrameListener
{
	C_PTR(AstraListener)
	SHARED_PTR(AstraListener)
	UNIQUE_PTR(AstraListener)

	PUBLIC AstraListener() {	}

#define SET_LWH_RESIZE(frame, pcloud) \
length = (frame).length(); \
width = (frame).width(); \
height = (frame).height(); \
(pcloud).resize(length);

	PUBLIC void getWorldDepthData(astra::StreamReader& reader, astra::Frame& frame) {
		const astra::CoordinateMapper cMapper = reader.stream<astra::DepthStream>().coordinateMapper();
		const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
		const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
		uint length, height, width;

		if (depthFrame.is_valid()) {
			SET_LWH_RESIZE(depthFrame, this->depthData)

			for (uint h = 0; h < height; ++h) {
				for (uint w = 0; w < width; ++w) {
					int index = (h*width)+w;
					this->depthData[index] = cMapper.convert_depth_to_world(astra::Vector3f(w, h, depthFrame.data()[index]));
				}
			}
		}

		// else if (pointFrame.is_valid()) {
		// 	SET_LWH_RESIZE(pointFrame, this->depthData)
		// 	astra::Vector3f* data = new astra::Vector3f[length];

		// 	for (uint h = 0; h < height; ++h) {
		// 		for (uint w = 0; w < width; ++w) {
		// 			uint index = (h*width)+w;
		// 			this->depthData[index] = pointFrame.data()[index];
		// 		}
		// 	}
		// }

		else {
			this->depthData.resize(0);
		}
	}

#define SET_LWH_RESIZE(frame, pcloud) \
length = (frame).length(); \
(pcloud).width = width = (frame).width(); \
(pcloud).height = height = (frame).height(); \
(pcloud).resize(length); \
(pcloud).header.frame_id = (frame).frame_index();

#define SET_XYZ(w, h, index, point, depthData) \
if (depthData.size() > 0) { \
	point.x = depthData[index].x; \
	point.y = depthData[index].y; \
	point.z = depthData[index].z; \
} \
else { \
	point.x = w; \
	point.y = h; \
	point.z = 0x0; \
}

#define SET_RGBA(point, rColor, gColor, bColor, aColor) \
point.r = (rColor); \
point.g = (gColor); \
point.b = (bColor); \
point.a = (aColor);

	PUBLIC template<typename FrameType> void frameToPointCloud(astra::Frame& frame) {
		FrameType f = frame.get<FrameType>();
		uint length, height, width;

		if (std::is_base_of<astra::DepthFrame, FrameType>()) {
			astra::DepthFrame* framePtr = (astra::DepthFrame*)(&f);
			if (framePtr->is_valid()) {
				SET_LWH_RESIZE(*framePtr, this->depthCloud)
				for (uint h = 0; h < height; ++h) {
					for (uint w = 0; w < width; ++w) {
						uint index = (h*width)+w;
						SET_XYZ(w, h, index, depthCloud.operator[](index), this->depthData)
						SET_RGBA(depthCloud.operator[](index), 0xFF, 0xFF, 0xFF, 0xFF)
					}
				}
			}
			
			else {
				this->depthCloud.resize(0);
			}
		}

		// else if (std::is_base_of<astra::PointFrame, FrameType>()) {
		// 	astra::PointFrame* framePtr = (astra::PointFrame*)(&f);
		// 	if (framePtr->is_valid()) {
		// 		SET_LWH_RESIZE(*framePtr, this->depthCloud)
		// 		for (uint h = 0; h < height; ++h) {
		// 			for (uint w = 0; w < width; ++w) {
		// 				uint index = (h*width)+w;
		// 				pointCloud.operator[](index).x = framePtr->data()[index].x;
		// 				pointCloud.operator[](index).y = framePtr->data()[index].y;
		// 				pointCloud.operator[](index).z = framePtr->data()[index].z;
		// 				SET_RGBA(pointCloud.operator[](index), 0xFF, 0xFF, 0xFF, 0xFF)
		// 			}
		// 		}
		// 	}
			
		// 	else {
		// 		this->pointCloud.resize(0);
		// 	}
		// }

#define SET_RGBA_2(point, pixel, aColor) \
point.r = pixel.r; \
point.g = pixel.g; \
point.b = pixel.b; \
point.a = (aColor);

		else if (std::is_base_of<astra::ColorFrame, FrameType>()) {
			astra::ColorFrame* framePtr = (astra::ColorFrame*)(&f);
			if (framePtr->is_valid()) {
				SET_LWH_RESIZE(*framePtr, this->depthCloud)
				for (uint h = 0; h < height; ++h) {
					for (uint w = 0; w < width; ++w) {
						uint index = (h*width)+w;
						SET_XYZ(w, h, index, colorCloud.operator[](index), this->depthData)
						SET_RGBA_2(colorCloud.operator[](index), framePtr->data()[index], 0xFF)
					}
				}
			}
			
			else {
				this->colorCloud.resize(0);
			}
		}

		else if (std::is_base_of<astra::InfraredFrame16, FrameType>()) {
			astra::InfraredFrame16* framePtr = (astra::InfraredFrame16*)(&f);
			if (framePtr->is_valid()) {
				SET_LWH_RESIZE(*framePtr, this->depthCloud)
				for (uint h = 0; h < height; ++h) {
					for (uint w = 0; w < width; ++w) {
						uint index = (h*width)+w;
						SET_XYZ(w, h, index, ir16Cloud.operator[](index), this->depthData)
						uint16_t color = framePtr->data()[index];
						uint8_t r = static_cast<uint8_t>(color >> 2);
						uint8_t b = 0x66 - r / 2;
						SET_RGBA(ir16Cloud.operator[](index), r, 0, b, 0xFF)
					}
				}
			}
			
			else {
				this->ir16Cloud.resize(0);
			}
		}

		else if (std::is_base_of<astra::InfraredFrameRgb, FrameType>()) {
			astra::InfraredFrameRgb* framePtr = (astra::InfraredFrameRgb*)(&f);
			if (framePtr->is_valid()) {
				SET_LWH_RESIZE(*framePtr, this->depthCloud)
				for (uint h = 0; h < height; ++h) {
					for (uint w = 0; w < width; ++w) {
						uint index = (h*width)+w;
						SET_XYZ(w, h, index, irRgbCloud.operator[](index), this->depthData)
						SET_RGBA_2(irRgbCloud.operator[](index), framePtr->data()[index], 0xFF)
					}
				}
			}
			
			else {
				this->irRgbCloud.resize(0);
			}
		}
	}

	PUBLIC virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
	{
		std::cout << "Frame is ready" << std::endl;
		this->getWorldDepthData(reader, frame);
		this->frameToPointCloud<astra::DepthFrame>(frame);
		this->frameToPointCloud<astra::PointFrame>(frame);
		this->frameToPointCloud<astra::ColorFrame>(frame);
		this->frameToPointCloud<astra::InfraredFrame16>(frame);
		this->frameToPointCloud<astra::InfraredFrameRgb>(frame);
		this->time = ros::Time::now();

		// const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();
		// const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

		// const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
		// const astra::InfraredFrame16 ir16 = frame.get<astra::InfraredFrame16>();
		// const astra::InfraredFrameRgb irRgb = frame.get<astra::InfraredFrameRgb>();


		// unsigned int width = colorFrame.width();
		// unsigned int height = colorFrame.height();
		// unsigned int length = colorFrame.length();
		
		// std::cout << "Width: " << width << std::endl;
		// std::cout << "Height: " << height << std::endl;
		// std::cout << "Length: " << length << std::endl;

		// sensor_msgs::PointCloud2 pcloud;
		// pcloud.header.stamp = ros::Time::now();
		// pcloud.header.frame_id = colorFrame.frame_index();
		// pcloud.height = height;
		// pcloud.width = width;
		// pcloud.point_step = sizeof(pcl::_PointXYZ) + sizeof(pcl::RGB);
		// pcloud.row_step = pcloud.width * pcloud.point_step;

		// pcloud.fields.resize(4);
		// pcloud.fields[0].name		= "x";
		// pcloud.fields[0].offset		= 0;
		// pcloud.fields[0].datatype	= sensor_msgs::PointField::FLOAT32;
		// pcloud.fields[0].count		= 1;
		// pcloud.fields[1].name		= "y";
		// pcloud.fields[1].offset		= 4;
		// pcloud.fields[1].datatype	= sensor_msgs::PointField::FLOAT32;
		// pcloud.fields[1].count		= 1;
		// pcloud.fields[2].name		= "z";
		// pcloud.fields[2].offset		= 8;
		// pcloud.fields[2].datatype	= sensor_msgs::PointField::FLOAT32;
		// pcloud.fields[2].count		= 1;
		// pcloud.fields[3].name		= "rgba";
		// pcloud.fields[3].offset		= 16;
		// pcloud.fields[3].datatype	= sensor_msgs::PointField::UINT32;
		// pcloud.fields[3].count		= 1;


		// int16_t * depths = new int16_t[width * height];
		// depthFrame.copy_to(depths);
		// unsigned int datasize = pcloud.height * pcloud.row_step;
		// uint8_t* data = new uint8_t[datasize];
		// astra::RgbPixel* pixels = new astra::RgbPixel[length];
		
		// colorFrame.copy_to(pixels);
		// for (unsigned int i = 0; i < datasize; i += pcloud.point_step)
		// {
		// 	unsigned int x = (i / pcloud.point_step) % width;
		// 	unsigned int y = (i / pcloud.point_step) / width;
		// 	astra::RgbPixel* pixel = &pixels[y * width + x];
		// 	*((float*)(&data[i])) = x;
		// 	*((float*)(&data[i + 4])) = y;
		// 	*((float*)(&data[i + 8])) = depths[y * width + x];
		// 	data[i + 16] = pixel->r;
		// 	data[i + 20] = pixel->b;
		// 	data[i + 24] = pixel->g;
		// 	data[i + 28] = 0xFF;
		// }

		// //pub.publish(pcloud);

		// delete[] depths;
		// delete[] pixels;
	}

	typedef pcl::PointCloud<pcl::PointXYZRGBA> FrameData;
	typedef std::vector<astra::Vector3f> DepthData;
	PUBLIC DepthData depthData;
	PUBLIC FrameData depthCloud;
	PUBLIC FrameData pointCloud;
	PUBLIC FrameData colorCloud;
	PUBLIC FrameData ir16Cloud;
	PUBLIC FrameData irRgbCloud;
	PUBLIC ros::Time time;
};