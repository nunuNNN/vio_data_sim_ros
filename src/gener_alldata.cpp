//
// Created by hyj on 17-6-22.
//

#include <fstream>
#include <sys/stat.h>
#include "imu.h"
#include "utilities.h"

using namespace Eigen;

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

void CreatePointsLines(Points& points, Lines& lines)
{
    std::ifstream f;
    f.open("house_model/house.txt");

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
            lines.emplace_back(pt0, pt1);   // lines
        }
    }

    // create more 3d points, you can comment this code
    int n = points.size();
    for (int j = 0; j < n; ++j) {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5,0.5,-0.5,0);
        points.push_back(p);
    }

    // save points
    // save_points("all_points.txt", points);
}

Vector2d cam2pixel(const Vector2d &p, const Vector4d &cam_int)
{
    Vector2d res(p(0)*cam_int(0)+cam_int(1), p(1)*cam_int(2)+cam_int(3));
    return res;
}

typedef struct
{
    uint32_t id;
    float u0;
    float v0;
    float u1;
    float v1;
} Stereo_feature_t;

typedef struct
{
    uint64_t stamp;
    std::vector<Stereo_feature_t> features;
} Feature_measure_t;

Feature_measure_t prev_features;
Feature_measure_t curr_features;


// void drawFeaturesStereo() {
//     // Colors for different features.
//     Scalar tracked(0, 255, 0);
//     Scalar new_feature(0, 255, 255);

//     static int grid_height =
//         cam0_curr_img_ptr->image.rows / processor_config.grid_row;
//     static int grid_width =
//         cam0_curr_img_ptr->image.cols / processor_config.grid_col;

//     // Create an output image.
//     int img_height = cam0_curr_img_ptr->image.rows;
//     int img_width = cam0_curr_img_ptr->image.cols;
//     Mat out_img(img_height, img_width*2, CV_8UC3);
//     cvtColor(cam0_curr_img_ptr->image, out_img.colRange(0, img_width), CV_GRAY2RGB);
//     cvtColor(cam1_curr_img_ptr->image, out_img.colRange(img_width, img_width*2), CV_GRAY2RGB);

//     // Collect features ids in the previous frame.
//     vector<FeatureIDType> prev_ids(0);
//     for (const auto& grid_features : *prev_features_ptr)
//         for (const auto& feature : grid_features.second)
//         prev_ids.push_back(feature.id);

//     // Collect feature points in the previous frame.
//     map<FeatureIDType, Point2f> prev_cam0_points;
//     map<FeatureIDType, Point2f> prev_cam1_points;
//     for (const auto& grid_features : *prev_features_ptr)
//         for (const auto& feature : grid_features.second) {
//         prev_cam0_points[feature.id] = feature.cam0_point;
//         prev_cam1_points[feature.id] = feature.cam1_point;
//         }

//     // Collect feature points in the current frame.
//     map<FeatureIDType, Point2f> curr_cam0_points;
//     map<FeatureIDType, Point2f> curr_cam1_points;
//     for (const auto& grid_features : *curr_features_ptr)
//         for (const auto& feature : grid_features.second) {
//         curr_cam0_points[feature.id] = feature.cam0_point;
//         curr_cam1_points[feature.id] = feature.cam1_point;
//         }

//     // Draw tracked features.
//     for (const auto& id : prev_ids) {
//         if (prev_cam0_points.find(id) != prev_cam0_points.end() && curr_cam0_points.find(id) != curr_cam0_points.end()) {
//             cv::Point2f prev_pt0 = prev_cam0_points[id];
//             cv::Point2f prev_pt1 = prev_cam1_points[id] + Point2f(img_width, 0.0);
//             cv::Point2f curr_pt0 = curr_cam0_points[id];
//             cv::Point2f curr_pt1 = curr_cam1_points[id] + Point2f(img_width, 0.0);

//             circle(out_img, curr_pt0, 3, tracked, -1);
//             circle(out_img, curr_pt1, 3, tracked, -1);
//             line(out_img, prev_pt0, curr_pt0, tracked, 1);
//             line(out_img, prev_pt1, curr_pt1, tracked, 1);

//             prev_cam0_points.erase(id);
//             prev_cam1_points.erase(id);
//             curr_cam0_points.erase(id);
//             curr_cam1_points.erase(id);
//         }
//     }

//     // Draw new features.
//     for (const auto& new_cam0_point : curr_cam0_points) {
//         cv::Point2f pt0 = new_cam0_point.second;
//         cv::Point2f pt1 = curr_cam1_points[new_cam0_point.first] +
//         Point2f(img_width, 0.0);

//         circle(out_img, pt0, 3, new_feature, -1);
//         circle(out_img, pt1, 3, new_feature, -1);
//     }

//     imshow("Feature", out_img);
//     waitKey(5);

//   return;
// }

int main(){
    // 建立keyframe文件夹
    mkdir("keyframe", 0777);

    // 生成3d points
    Points points;
    Lines lines;
    CreatePointsLines(points, lines);

    // IMU model
    Param params;
    IMU imuGen(params);
    bool first_frame = true;

    // create imu data
    // imu pose gyro acc
    std::vector< MotionData > imudata;
    std::vector< MotionData > imudata_noise;
    for (float t = params.t_start; t<params.t_end;) 
    {
        MotionData data = imuGen.MotionModel(t);
        imudata.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        t += 1.0/params.imu_frequency;
    }
    imuGen.init_velocity_ = imudata[0].imu_velocity;
    imuGen.init_twb_ = imudata.at(0).twb;
    imuGen.init_Rwb_ = imudata.at(0).Rwb;
    // save_Pose("imu_pose.txt", imudata);
    // save_Pose("imu_pose_noise.txt", imudata_noise);

    // imuGen.testImu("imu_pose.txt", "imu_int_pose.txt");     // test the imu data, integrate the imu data to generate the imu trajecotry
    // imuGen.testImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");

    // cam pose
    std::vector< MotionData > camdata;
    std::vector< MotionData > camdataRight;
    for (float t = params.t_start; t<params.t_end;) 
    {
        MotionData imu = imuGen.MotionModel(t);   // imu body frame to world frame motion
        MotionData cam;
        MotionData camRight;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;    // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb
        camdata.push_back(cam);

        camRight.timestamp = imu.timestamp;
        camRight.Rwb = imu.Rwb * params.R_bc_right;    // cam frame in world frame
        camRight.twb = imu.twb + imu.Rwb * params.t_bc_right; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb
        camdataRight.push_back(camRight);

        t += 1.0/params.cam_frequency;
    }
    // save_Pose("cam_pose.txt",camdata);
    // save_Pose_asTUM("cam_pose_tum.txt",camdata);

    // points obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;
        data = camdataRight[n];
        Eigen::Matrix4d TwcRight = Eigen::Matrix4d::Identity();
        TwcRight.block(0, 0, 3, 3) = data.Rwb;
        TwcRight.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // 对应的２维图像坐标
        std::vector<Vector3d> points_cur_camRight;    // ３维点在当前帧cam视野里
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_camRight;  // 对应的２维图像坐标
        for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector4d pw = points[i];          // 最后一位存着feature id
            pw[3] = 1;                               //改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector4d pc1Right = TwcRight.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0 || pc1Right(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0)/pc1(2), pc1(1)/pc1(2)) ;
            Eigen::Vector2d obs_piexl = cam2pixel(obs, params.cam_intrinsics);
            points_cam.push_back(points[i]);
            features_cam.push_back(obs_piexl);

            Eigen::Vector2d obsRight(pc1Right(0)/pc1Right(2), pc1Right(1)/pc1Right(2)) ;
            Eigen::Vector2d obsRight_piexl = cam2pixel(obsRight, params.cam_intrinsics);
            features_camRight.push_back(obsRight_piexl); 

            points_cur_camRight.push_back(Vector3d(pc1Right(0), pc1Right(1), pc1Right(2)));
        }
        if(first_frame)
        { 
            first_frame = false;
            prev_features = curr_features;
        }
        // 调用优化函数，进行仿真优化
        
        // .......

        // 将当前特征点保存成为上一帧数据
        prev_features = curr_features;

        // save points
        // std::stringstream filename1;
        // filename1<<"keyframe/all_points_"<<n<<".txt";
        // save_features(filename1.str(),points_cam,features_cam);
    }

    return 0;
}
