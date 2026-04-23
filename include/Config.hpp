#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include "common_lib.h"

struct FastLioConfig
{
  struct Common
  {
    std::string lid_topic = "/livox/lidar";
    std::string imu_topic = "/livox/imu";
    bool time_sync_en = false;
    double time_offset_lidar_to_imu = 0.0;
  } common;

  struct Preprocess
  {
    int lidar_type = 1;
    int scan_line = 6;
    int timestamp_unit = 3; // 0-second, 1-ms, 2-us, 3-ns
    double blind = 4;
  } preprocess;

  // check extrnsics: from what frame to what frame
  struct Mapping
  {
    double acc_cov = 0.1;
    double gyr_cov = 0.1;
    double b_acc_cov = 0.0001;
    double b_gyr_cov = 0.0001;
    double fov_degree = 90;
    float det_range = 450;
    bool extrinsic_est_en = false;
    std::vector<double> extrinsic_T = {0., 0., 0.};
    std::vector<double> extrinsic_R = {1., 0., 0., 0., 1., 0, 0., 0., 1.};
    bool feature_extract_enable = false; // launch
    int point_filter_num = 3;            // launch
    int max_iteration = 3;               // launch
    double filter_size_surf = 0.5;       // launch
    double filter_size_map = 0.5;        // launch
    double cube_side_length = 1000;      // launch

  } mapping;

  struct Publish
  {
    bool path_en = true;
    bool scan_publish_en = true;
    bool dense_publish_en = true;
    bool scan_bodyframe_pub_en = true;
    bool runtime_pos_log_enable = true; // launch
  } publish;

  struct PcdSave
  {
    bool pcd_save_en = true;
    int interval = -1;
  } pcd_save;

  struct Bag
  {
    std::string bag_path = "";
    std::string serial_vs_subscribe = "serial";
    double bag_start = 0.0;
    double bag_duration = -1.0;
  } bag;

  std::string config_path = "--";
  void load(ros::NodeHandle &nh)
  {
    // mapping / EKF
    nh.param<int>("mapping/max_iteration", mapping.max_iteration, 4);
    nh.param<int>("mapping/point_filter_num", mapping.point_filter_num, 2);

    // common
    nh.param<std::string>("common/lid_topic", common.lid_topic, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", common.imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", common.time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", common.time_offset_lidar_to_imu, 0.0);

    // mapping
    nh.param<double>("mapping/filter_size_surf", mapping.filter_size_surf, 0.5);
    nh.param<double>("mapping/filter_size_map", mapping.filter_size_map, 0.5);
    nh.param<double>("mapping/cube_side_length", mapping.cube_side_length, 200.0);
    nh.param<float>("mapping/det_range", mapping.det_range, 300.f);
    nh.param<double>("mapping/gyr_cov", mapping.gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", mapping.acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", mapping.b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", mapping.b_acc_cov, 0.0001);
    nh.param<double>("mapping/fov_degree", mapping.fov_degree, 180.0);
    nh.param<bool>("mapping/extrinsic_est_en", mapping.extrinsic_est_en, false);
    nh.getParam("mapping/extrinsic_T", mapping.extrinsic_T);
    nh.getParam("mapping/extrinsic_R", mapping.extrinsic_R);
    nh.param<bool>("mapping/feature_extract_enable", mapping.feature_extract_enable, false);

    // preprocess
    nh.param<double>("preprocess/blind", preprocess.blind, 1.0);
    nh.param<int>("preprocess/lidar_type", preprocess.lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", preprocess.scan_line, 16);
    nh.param<int>("preprocess/timestamp_unit", preprocess.timestamp_unit, 3);

    // publish
    nh.param<bool>("publish/path_en", publish.path_en, true);
    nh.param<bool>("publish/scan_publish_en", publish.scan_publish_en, true);
    nh.param<bool>("publish/dense_publish_en", publish.dense_publish_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", publish.scan_bodyframe_pub_en, true);

    // misc
    nh.param<bool>("publish/runtime_pos_log_enable", publish.runtime_pos_log_enable, false);

    // pcd_save
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save.pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save.interval, -1);

    // bag / playback
    nh.param<std::string>("bag_path", bag.bag_path, "");
    nh.param<std::string>("serial_vs_subscribe", bag.serial_vs_subscribe, "serial");
    nh.param<std::string>("config_path", config_path, "Unset.");
    nh.param<double>("bag_start", bag.bag_start, 0.0);
    nh.param<double>("bag_duration", bag.bag_duration, -1.0);
  }
};
