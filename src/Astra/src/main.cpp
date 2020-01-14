// External Library Includes
#include <astra_core/astra_core.hpp>
#include <astra/astra.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

// Project File Inclides
#include "frameListener.h"
#include "publisher.h"
#include "key_handler.h"

void callBack(const std_msgs::String::ConstPtr& msg)
{
	std::cout << msg->data.c_str() << std::endl;
}

int main(int argc, char** argv)
{
	try {
		set_key_handler();
		std::cout << "Initializing ROS" << std::endl;
		ros::init(argc, argv, "astra_camera");
		ros::NodeHandle nh("~");

		std::cout << "Initialized Astra with error code: "
			<< astra::initialize() << std::endl;

		std::cout << "Initializing Reader" << std::endl;
		astra::StreamSet streamSet;
		astra::StreamReader reader = streamSet.create_reader();
		// reader.stream<astra::ColorStream>().start();
		// reader.stream<astra::DepthStream>().start();

		std::cout << "Creating Listener" << std::endl;
		AstraListener::Shared_Ptr listener(new AstraListener());
		reader.add_listener(*listener);

		std::cout << "Creating Publisher" << std::endl;
		Publisher publisher(reader);

		std::cout << "Updating Astra Frame" << std::endl;
		ros::Rate rate(500);
		while(ros::ok() && shouldContinue) {
			astra_update();
			publisher.publishCameraFeed(listener);
			rate.sleep();
		}
		
		reader.remove_listener(*listener);
	}
	catch (const char* message) {
		std::cout << message << std::endl;
	}

	std::cout << "Terminated with error code: "
		<< astra::terminate() << std::endl;
}