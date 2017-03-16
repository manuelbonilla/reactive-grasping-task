#include <ros/ros.h>
#include <reactive_grasping/GloveIMU.h>
#include <reactive_grasping/GloveIMUArray.h>
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

ros::Publisher pub_glove;
ros::Subscriber sub_acc, sub_gyro;
std::vector<double> acc, gyro;
std::vector<int> imu_ids;
int num_imo;

void cb_acc(const qb_interface::inertialSensorArray::ConstPtr& msg )
{
	int j = 0;
	for (int i = 0; i < num_imo; ++i)
	{
		imu_ids[i] = msg->m[i].id;
		acc[j] = (msg->m[i].x - msg->m[num_imo - 1].x);
		acc[j + 1] = (msg->m[i].y - msg->m[num_imo - 1].y);
		acc[j + 2] = (msg->m[i].z - msg->m[num_imo - 1].z);
		j += 3;
	}
}

void cb_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg )
{
	int j = 0;
	for (int i = 0; i < num_imo; ++i)
	{
		gyro[j] = msg->m[i].x  - msg->m[num_imo - 1].x;
		gyro[j + 1] = msg->m[i].y  - msg->m[num_imo - 1].y;
		gyro[j + 2] = msg->m[i].z  - msg->m[num_imo - 1].z;
		j += 3;
	}
}

void publish(ros::Time t)
{
	int j = 0;
	reactive_grasping::GloveIMU imu;
	reactive_grasping::GloveIMUArray imus;
	for (int i = 0; i < num_imo - 1; i++)
	{
		imu.id = imu_ids[i];
		imu.linear_acceleration.x = acc[j] * 4096;
		imu.linear_acceleration.y = acc[j + 1] * 4096;
		imu.linear_acceleration.z = acc[j + 2] * 4096;
		imu.angular_velocity.x = gyro[j];
		imu.angular_velocity.y = gyro[j + 1];
		imu.angular_velocity.z = gyro[j + 2];
		j += 3;

		imus.data.push_back(imu);
	}

	imus.header.stamp = t;
	imus.header.frame_id = std::string("frame_ee_glove");
	imus.acquisition_time = ros::Duration();
	pub_glove.publish(imus);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "glove_relative");
	ROS_INFO("glove_relative started");
	ros::NodeHandle n;
	float f;
	std::string acc_topic_name, gyro_topic_name, glove_topic_name;
	n.param<int>("n_imo", num_imo, 6);
	n.param<float>("speed_rate", f, 100);
	n.param<std::string>("acc_topic", acc_topic_name, "/qb_class_imu/acc");
	n.param<std::string>("gyro_topic", gyro_topic_name, "/qb_class_imu/gyro");
	n.param<std::string>("glove_topic", glove_topic_name, "/glove_data");

	acc = std::vector<double>(num_imo * 3, 0.);
	gyro = std::vector<double>(num_imo * 3, 0.);
	imu_ids = std::vector<int>(num_imo, 0);
	ros::Rate r(f);

	pub_glove = n.advertise<reactive_grasping::GloveIMUArray>(glove_topic_name, 1);
	sub_acc = n.subscribe(acc_topic_name, 1, cb_acc);
	sub_gyro = n.subscribe(gyro_topic_name, 1, cb_gyro);

	while (ros::ok())
	{
		ros::spinOnce();
		publish( ros::Time::now() );
		r.sleep();
	}

	return 0;
}