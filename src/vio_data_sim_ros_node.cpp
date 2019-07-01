#include <ros/ros.h> 
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>

#include <fstream>

#include "../include/vio_data_sim_ros/imu.h"
#include "../include/vio_data_sim_ros/utilities.h"

std::vector < std::pair< Eigen::Vector4d, Eigen::Vector4d > >
CreatePointsLines(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& points)
{

    std::vector < std::pair< Eigen::Vector4d, Eigen::Vector4d > > lines;

    std::ifstream f;
    f.open("/home/liudong/catkin_ws/src/vio_data_sim_ros/house.txt");
    if(!f)
    {
        std::cout<< "fail to open the file" << std::endl;
    }

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double x,y,z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0( x, y, z, 1 );
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1( x, y, z, 1 );

            bool isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt0)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt0);

            isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt1)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt1);

            // pt0 = Twl * pt0;
            // pt1 = Twl * pt1;
            lines.push_back( std::make_pair(pt0,pt1) );   // lines
        }
    }

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j) {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5,0.5,-0.5,0);
        points.push_back(p);
    }

    // save points
    // std::stringstream filename;
    // filename<<"all_points.txt";
    // save_points(filename.str(),points);
    return lines;
}


int main(int argc, char** argv)
{
    // 生成3d points
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points;
    std::vector < std::pair< Eigen::Vector4d, Eigen::Vector4d > > lines;
    lines = CreatePointsLines(points);

    ros::init(argc, argv, "vio_data_sim_ros");
    ros::NodeHandle n;

    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu0", 20);
    ros::Publisher img_pub = n.advertise<sensor_msgs::PointCloud>("feature_tracker/feature", 1000);
    ros::Rate loop_rate(200);
    double start_t = ros::Time::now().toSec();
    int imu_num = 0;
    while(ros::ok())
    {
        // IMU model
        Param params;
        IMU imuGen(params);
        double t = ros::Time::now().toSec() - start_t;
        // create imu data
        // imu pose gyro acc
        ROS_INFO_STREAM("T : " << t);
        MotionData data = imuGen.MotionModel(t);
        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        // to qua
        Eigen::Quaterniond q(data_noise.Rwb);


        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "world";
        //四元数位姿
        imu_data.orientation.x = q.x();
        imu_data.orientation.y = q.y();
        imu_data.orientation.z = q.z();
        imu_data.orientation.w = q.w();
        //线加速度
        imu_data.linear_acceleration.x = data_noise.imu_acc(0); 
        imu_data.linear_acceleration.y = data_noise.imu_acc(1);
        imu_data.linear_acceleration.z = data_noise.imu_acc(2);
        //角速度
        imu_data.angular_velocity.x = data_noise.imu_gyro(0); 
        imu_data.angular_velocity.y = data_noise.imu_gyro(1); 
        imu_data.angular_velocity.z = data_noise.imu_gyro(2);

        IMU_pub.publish(imu_data);


        imu_num++;
        if (imu_num == 10)
        {
            imu_num = 0;

            // cam pose 
            MotionData cam;
            cam.Rwb = data.Rwb * params.R_bc;    // cam frame in world frame
            cam.twb = data.twb + data.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            Twc.block(0, 0, 3, 3) = cam.Rwb;
            Twc.block(0, 3, 3, 1) = cam.twb;

            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            feature_points->header = imu_data.header;
            feature_points->header.frame_id = "world";
            
            // 遍历所有的特征点，看哪些特征点在视野里
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pw = points[i];          // 最后一位存着feature id
                pw[3] = 1;                               //改成齐次坐标最后一位
                Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame

                if(pc1(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

                geometry_msgs::Point32 p;
                p.x = pc1(0)/pc1(2);
                p.y = pc1(1)/pc1(2);
                p.z = 1;
                feature_points->points.push_back(p);
                id_of_point.values.push_back(i);
                u_of_point.values.push_back(p.x);
                v_of_point.values.push_back(p.y);
                velocity_x_of_point.values.push_back(0);
                velocity_y_of_point.values.push_back(0);
            }
            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);
            img_pub.publish(feature_points);
        }

        ros::spinOnce();  
        loop_rate.sleep();  
    }

    return 0;
}


