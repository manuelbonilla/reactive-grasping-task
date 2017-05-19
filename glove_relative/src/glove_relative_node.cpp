#include <ros/ros.h>
#include <reactive_grasping/GloveIMU.h>
#include <reactive_grasping/GloveIMUArray.h>
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>

ros::Publisher pub_glove, pub_glove_local;
ros::Subscriber sub_acc, sub_gyro;
std::vector<double> acc, gyro, acc_relative, gyro_relative;
std::vector<int> imu_ids;
int num_imu;
double sampleFreq_  =  80;
double beta_        =  0.4;
std::vector<Eigen::Vector4d> QL_vector;

/*This package publish the relative acceleration of num_imu-1 imu's relative to the num_imu_imo*/

Eigen::Vector4d ConjQ(Eigen::Vector4d Q_in)
{
	Eigen::Vector4d Q_out;
	Q_out(0) =  Q_in(0);
	Q_out(1) = -Q_in(1);
	Q_out(2) = -Q_in(2);
	Q_out(3) = -Q_in(3);
	return Q_out;
}

Eigen::Vector4d QxQ(Eigen::Vector4d Q_1, Eigen::Vector4d Q_2)
{
	Eigen::Vector4d Q_out;
	Q_out(0) = Q_1(0) * Q_2(0) - (Q_1(1) * Q_2(1) + Q_1(2) * Q_2(2) + Q_1(3) * Q_2(3));
	Q_out(1) = Q_1(0) * Q_2(1) + Q_1(1) * Q_2(0) + (Q_1(2) * Q_2(3) - Q_1(3) * Q_2(2));
	Q_out(2) = Q_1(0) * Q_2(2) + Q_1(2) * Q_2(0) + (Q_1(3) * Q_2(1) - Q_1(1) * Q_2(3));
	Q_out(3) = Q_1(0) * Q_2(3) + Q_1(3) * Q_2(0) + (Q_1(1) * Q_2(2) - Q_1(2) * Q_2(1));
	return Q_out;
}


Eigen::Matrix3d quat2rot(Eigen::Vector4d Q)
{

	double x = Q(1), y = Q(2), z = Q(3), w = Q(0);
	double x2, y2, z2, w2;
	x2 = x * x;  y2 = y * y; z2 = z * z;  w2 = w * w;

	Eigen::Matrix3d R;
	R << w2 + x2 - y2 - z2, 2 * x*y - 2 * w*z, 2 * x*z + 2 * w*y,
	2 * x*y + 2 * w*z, w2 - x2 + y2 - z2, 2 * y*z - 2 * w*x,
	2 * x*z - 2 * w*y, 2 * y*z + 2 * w*x, w2 - x2 - y2 + z2;

	return R;
}

Eigen::Vector4d MadgwickFilter(int P, int N, Eigen::Vector4d qL)
{
	//come N vede P;  N è il mio d (wordl), P è il mio s (sensor)
	// ad esempio N=imu1, P=imu0, quindi come la IMU 1 vede la IMU 0

	// float recipNorm;
	// float qDot1, qDot2, qDot3, qDot4;
	float q1, q2 , q3 , q4;

	float dx, dy, dz;
	float sx, sy, sz;

	Eigen::Vector3d aP, aN, gPpartial, gNpartial;
	Eigen::Vector4d gP, gN, g;

	Eigen::Vector3d fa;
	Eigen::MatrixXd Ja(3, 4);
	Eigen::Vector4d  qdot;

	Eigen::Vector4d Napla;


	aP(0)  = acc[P * 3];
	aP(1)  = acc[P * 3 + 1];
	aP(2)  = acc[P * 3 + 2];

	aN(0)  = acc[N * 3];
	aN(1)  = acc[N * 3 + 1];
	aN(2)  = acc[N * 3 + 2];

	if ((aP.norm() == 0) || (aN.norm() == 0.0))
		return qL;

	// std::cout << "P " << P << "  ax " << aP(0) << "  ay " << aP(1) << "  az " << aP(2) << std::endl;
	// std::cout << "N " << N << "  ax " << aN(0) << "  ay " << aN(1) << "  az " << aN(2) << std::endl;

	aP = aP / aP.norm();
	aN = aN / aN.norm();


	gP(0)  = 0;
	gP(1)  = gyro[P * 3];
	gP(2)  = gyro[P * 3 + 1];
	gP(3)  = gyro[P * 3 + 2];

	gN(0)  = 0;
	gN(1)  = gyro[N * 3];
	gN(2)  = gyro[N * 3 + 1];
	gN(3)  = gyro[N * 3 + 2];

	gP = gP * (M_PI / 180);
	gN = gN * (M_PI / 180);


	// rotate the angular velocity
	g = QxQ(QxQ(qL, gP), ConjQ(qL)) - gN;


	q1 = qL(0);
	q2 = qL(1);
	q3 = qL(2);
	q4 = qL(3);

	//accelerometer
	dx = aN(0);
	dy = aN(1);
	dz = aN(2);

	sx = aP(0);
	sy = aP(1);
	sz = aP(2);

	fa(0) =  2 * dx * (0.5 - q3 * q3 - q4 * q4) + 2 * dy * (q1 * q4 + q2 * q3) + 2 * dz * (q2 * q4 - q1 * q3) - sx;
	fa(1) =  2 * dx * (q2 * q3 - q1 * q4) + 2 * dy * (0.5 - q2 * q2 - q4 * q4) + 2 * dz * (q1 * q2 + q3 * q4) - sy;
	fa(2) =  2 * dx * (q1 * q3 - q2 * q4) + 2 * dy * (q3 * q4 - q1 * q2) + 2 * dz * (0.5 - q2 * q2 - q3 * q3) - sz;

	// Compute the Jacobian
	Ja << 2 * dy*q4 - 2 * dz*q3,    2 * dy*q3 + 2 * dz*q4 ,        -4 * dx*q3 + 2 * dy*q2 - 2 * dz*q1,  -4 * dx*q4 + 2 * dy*q1 + 2 * dz*q2,
	-2 * dx*q4 + 2 * dz*q2,   2 * dx*q3 - 4 * dy*q2 + 2 * dz*q1, 2 * dx*q2 + 2 * dz*q4,           -2 * dx*q1 - 4 * dy*q4 + 2 * dz*q3,
	2 * dx*q3 - 2 * dy*q2,    2 * dx*q4 - 2 * dy*q1 - 4 * dz*q2, 2 * dx*q1 + 2 * dy*q4 - 4 * dz*q3,   2 * dx*q2 + 2 * dy*q3;


	// Compute the Napla
	Napla = Ja.transpose() * fa;

	qdot = 0.5 * QxQ( qL, g ) - ( beta_ * Napla );

	qL = qL + qdot / sampleFreq_;

	qL = qL / qL.norm();

	return qL;
}


void cb_acc(const qb_interface::inertialSensorArray::ConstPtr& msg )
{
	int j = 0;
	for (int i = 0; i < num_imu; ++i)
	{
		imu_ids[i] = msg->m[i].id;
		acc[j] = msg->m[i].x;
		acc[j + 1] = msg->m[i].y;
		acc[j + 2] = msg->m[i].z;
		j += 3;
	}
}

void cb_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg )
{
	int j = 0;
	for (int i = 0; i < num_imu; ++i)
	{
		gyro[j] = msg->m[i].x;
		gyro[j + 1] = msg->m[i].y;
		gyro[j + 2] = msg->m[i].z;
		j += 3;
	}
	j = 0;
	for (int i = 0; i < num_imu; i++)
	{
		if (std::abs(gyro[j]) < 15)  gyro[j] = 0;
		if (std::abs(gyro[j + 1]) < 15)  gyro[j+1] = 0;
		if (std::abs(gyro[j + 2] ) < 15)  gyro[j+2] = 0;
		j += 3;	
	}


}

void publish(ros::Time t)
{
	int j = 0;
	reactive_grasping::GloveIMU imu, imu_local;
	reactive_grasping::GloveIMUArray imus;
	reactive_grasping::GloveIMUArray imus_local;



	for (int i = 0; i < num_imu - 1; i++)
	{


		Eigen::Vector3d acc_relative, acc_local;
		acc_local <<  acc[j],  acc[j + 1], acc[j + 2];
		acc_relative = quat2rot(QL_vector[i]) * acc_local;

		imu.id = imu_ids[i];
		imu.linear_acceleration.x = (acc_relative(0) - acc[num_imu * 3 - 3]) * 4096;
		imu.linear_acceleration.y = (acc_relative(1) - acc[num_imu * 3 - 2]) * 4096;
		imu.linear_acceleration.z = (acc_relative(2) - acc[num_imu * 3 - 1]) * 4096;

		Eigen::Vector3d gyro_relative, gyro_local;
		gyro_local <<  gyro[j],  gyro[j + 1], gyro[j + 2];
		gyro_relative = quat2rot(QL_vector[i]) * gyro_local;

		imu.angular_velocity.x = gyro_relative(0) - gyro[num_imu * 3 - 3];
		imu.angular_velocity.y = gyro_relative(1) - gyro[num_imu * 3 - 2];
		imu.angular_velocity.z = gyro_relative(2) - gyro[num_imu * 3 - 1];

		imus.data.push_back(imu);


		imu_local.id = imu_ids[i];
		imu_local.linear_acceleration.x = acc_local(0);
		imu_local.linear_acceleration.y = acc_local(1);
		imu_local.linear_acceleration.z = acc_local(2);
		imu_local.angular_velocity.x = gyro_local(0);
		imu_local.angular_velocity.y = gyro_local(1);
		imu_local.angular_velocity.z = gyro_local(2);

		imus_local.data.push_back(imu_local);

		j += 3;


	}

	imus.header.stamp = t;
	imus.header.frame_id = std::string("frame_ee_glove");
	imus.acquisition_time = ros::Duration();
	pub_glove.publish(imus);
	pub_glove_local.publish(imus_local);
}

Eigen::Vector3d quat2Angles(Eigen::Vector4d Q_in)
{
	Eigen::Vector3d Angles_out;
	// pitch singularity
	// Angles_out(0) = atan2(2*Q_in(1)*Q_in(2) - 2*Q_in(0)*Q_in(3), 2*Q_in(0)*Q_in(0) + 2*Q_in(1)*Q_in(1)-1); //YAW
	// Angles_out(1) = -asin(2*Q_in(1)*Q_in(3) + 2*Q_in(0)*Q_in(2)); //PITCH
	// Angles_out(2) = atan2(2*Q_in(2)*Q_in(3) - 2*Q_in(0)*Q_in(1), 2*Q_in(0)*Q_in(0) + 2*Q_in(3)*Q_in(3)-1); // ROLL

	// yaw singularity
	float w = Q_in(0);
	float x = Q_in(1);
	float y = Q_in(2);
	float z = Q_in(3);
	Angles_out(0) =  asin(2 * x * y + 2 * z * w); //YAW
	Angles_out(2) = - atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z); //PITCH
	Angles_out(1) = - atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z); // ROLL
	return Angles_out;
}

void print_angles(std::vector<Eigen::Vector4d> QL_vector)
{
	std::cout << "Imu 1: " <<  quat2Angles(QL_vector[0]).transpose() * 180 / M_PI << std::endl;
	std::cout << "Imu 2: " <<  quat2Angles(QL_vector[1]).transpose() * 180 / M_PI << std::endl;
	std::cout << "Imu 3: " <<  quat2Angles(QL_vector[2]).transpose() * 180 / M_PI << std::endl;
	std::cout << "Imu 4: " <<  quat2Angles(QL_vector[3]).transpose() * 180 / M_PI << std::endl;
	std::cout << "Imu 5: " <<  quat2Angles(QL_vector[4]).transpose() * 180 / M_PI << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "glove_relative");
	ROS_INFO("glove_relative started");
	ros::NodeHandle n;
	float f;
	std::string acc_topic_name, gyro_topic_name, glove_topic_name;
	n.param<int>("n_imo", num_imu, 6);
	n.param<float>("speed_rate", f, 80);
	n.param<std::string>("acc_topic", acc_topic_name, "/qb_class_imu/acc");
	n.param<std::string>("gyro_topic", gyro_topic_name, "/qb_class_imu/gyro");
	n.param<std::string>("glove_topic", glove_topic_name, "/glove_data");

	pub_glove = n.advertise<reactive_grasping::GloveIMUArray>(glove_topic_name, 1);
	pub_glove_local = n.advertise<reactive_grasping::GloveIMUArray>(glove_topic_name + std::string("_local"), 1);
	sub_acc = n.subscribe(acc_topic_name, 1, cb_acc);
	sub_gyro = n.subscribe(gyro_topic_name, 1, cb_gyro);

	acc = std::vector<double>(num_imu * 3, 0.0);
	gyro = std::vector<double>(num_imu * 3, 0.0);
	imu_ids = std::vector<int>(num_imu, 0);
	ros::Rate r(f);
	QL_vector = std::vector<Eigen::Vector4d>(5, Eigen::Vector4d(1.0, 0., 0., 0.));

	for (double i = 0.; i < 30.; i = i + 1.0 / f)
	{
		ros::spinOnce();
		// Eigen::MatrixXd All_Q = Eigen::MatrixXd::Zero(4, num_imu-1);
		for (int j = 0; j < num_imu - 1; ++j)
		{
			// All_Q.block<4,1>(0,j) = QL_vector[j];
			QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
			// std::cout << "Q( " << j << "): " << QL_vector[j] << " " << std::endl;
		}
		// std::cout << i << std::endl <<  All_Q << std::endl << std::endl << std::endl;
		publish( ros::Time::now() );
		r.sleep();
	}

	print_angles( QL_vector );
	getchar();



	while (ros::ok())
	{
		for (int j = 0; j < num_imu - 1; ++j)
		{
			QL_vector[j] = MadgwickFilter(j, num_imu - 1, QL_vector[j]);
		}
		ros::spinOnce();
		publish( ros::Time::now() );
		r.sleep();
	}

	return 0;
}