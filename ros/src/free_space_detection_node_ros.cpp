// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ipa_visual_free_space_detection/free_space_detection_nodeConfig.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

// other includes
#include <free_space_detection_node_common.cpp>


class free_space_detection_node_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<free_space_detection::free_space_detection_nodeConfig> server;
    dynamic_reconfigure::Server<free_space_detection::free_space_detection_nodeConfig>::CallbackType f;

    ros::Publisher segment_freespace_mask_;
    ros::Publisher colorHistogram_freespace_mask_;
    ros::Publisher disparity_freespace_mask_;
    ros::Publisher final_freespace_mask_;
    ros::Publisher freeSpacePoints_;
    ros::Subscriber camera_rgb_camera_info_;
    ros::Subscriber camera_rgb_image_color_;
    ros::Subscriber camera_depth_disparity_;
    ros::Subscriber camera_depth_points_;
    ros::Subscriber camera_depth_camera_info_;

    free_space_detection_node_data component_data_;
    free_space_detection_node_config component_config_;
    free_space_detection_node_impl component_implementation_;

    free_space_detection_node_ros() : np_("~")
    {
        f = boost::bind(&free_space_detection_node_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        segment_freespace_mask_ = n_.advertise<sensor_msgs::Image>("segment_freespace_mask", 1);
        colorHistogram_freespace_mask_ = n_.advertise<sensor_msgs::Image>("colorHistogram_freespace_mask", 1);
        disparity_freespace_mask_ = n_.advertise<sensor_msgs::Image>("disparity_freespace_mask", 1);
        final_freespace_mask_ = n_.advertise<sensor_msgs::Image>("final_freespace_mask", 1);
        freeSpacePoints_ = n_.advertise<sensor_msgs::PointCloud2>("freeSpacePoints", 1);
        camera_rgb_camera_info_ = n_.subscribe("camera_rgb_camera_info", 1, &free_space_detection_node_ros::topicCallback_camera_rgb_camera_info, this);
        camera_rgb_image_color_ = n_.subscribe("camera_rgb_image_color", 1, &free_space_detection_node_ros::topicCallback_camera_rgb_image_color, this);
        camera_depth_disparity_ = n_.subscribe("camera_depth_disparity", 1, &free_space_detection_node_ros::topicCallback_camera_depth_disparity, this);
        camera_depth_points_ = n_.subscribe("camera_depth_points", 1, &free_space_detection_node_ros::topicCallback_camera_depth_points, this);
        camera_depth_camera_info_ = n_.subscribe("camera_depth_camera_info", 1, &free_space_detection_node_ros::topicCallback_camera_depth_camera_info, this);

        np_.param("m_debug_b", component_config_.m_debug_b, (bool)false);
        np_.param("m_extractROI_b", component_config_.m_extractROI_b, (bool)true);
        np_.param("m_printTimings_b", component_config_.m_printTimings_b, (bool)false);
        np_.param("m_writeResults_b", component_config_.m_writeResults_b, (bool)false);
        np_.param("m_sigma_gaussian_filter_d", component_config_.m_sigma_gaussian_filter_d, (double)0.7);
        np_.param("m_minH_thresh_d", component_config_.m_minH_thresh_d, (double)0.3);
        np_.param("m_minI_thresh_d", component_config_.m_minI_thresh_d, (double)0.3);
        np_.param("m_felz_sigma_d", component_config_.m_felz_sigma_d, (double)0.9);
        np_.param("m_felz_k_i", component_config_.m_felz_k_i, (int)200);
        np_.param("m_felz_minSize_i", component_config_.m_felz_minSize_i, (int)250);
        np_.param("m_segmentClassification_thresh_d", component_config_.m_segmentClassification_thresh_d, (double)0.6);
        np_.param("m_morphSize_i", component_config_.m_morphSize_i, (int)2);
        np_.param("m_Vdisp_thresh_d", component_config_.m_Vdisp_thresh_d, (double)10.0);
        np_.param("m_minValidDisp_thresh_d", component_config_.m_minValidDisp_thresh_d, (double)0.7);
        np_.param("m_resolution_H_d", component_config_.m_resolution_H_d, (double)0.05);
        np_.param("m_resolution_I_d", component_config_.m_resolution_I_d, (double)0.03);
        np_.param("m_Udisp_thresh_d", component_config_.m_Udisp_thresh_d, (double)5.0);
        np_.param("m_maxDelay_d", component_config_.m_maxDelay_d, (double)450.0);
        np_.param("m_alpha_d", component_config_.m_alpha_d, (double)0.7);
        np_.param("m_maxH_thresh_d", component_config_.m_maxH_thresh_d, (double)0.8);
        np_.param("m_maxI_thresh_d", component_config_.m_maxI_thresh_d, (double)0.8);
        np_.param("m_priorityAccuracy_b", component_config_.m_priorityAccuracy_b, (bool)true);
        np_.param("m_minIntensity_i", component_config_.m_minIntensity_i, (int)80);
    }
    void topicCallback_camera_rgb_camera_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        component_data_.in_camera_rgb_camera_info = *msg;
    }
    void topicCallback_camera_rgb_image_color(const sensor_msgs::Image::ConstPtr& msg)
    {
        component_data_.in_camera_rgb_image_color = *msg;
    }
    void topicCallback_camera_depth_disparity(const stereo_msgs::DisparityImage::ConstPtr& msg)
    {
        component_data_.in_camera_depth_disparity = *msg;
    }
    void topicCallback_camera_depth_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        component_data_.in_camera_depth_points = *msg;
    }
    void topicCallback_camera_depth_camera_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        component_data_.in_camera_depth_camera_info = *msg;
    }

    void configure_callback(free_space_detection::free_space_detection_nodeConfig &config, uint32_t level)
    {
        component_config_.m_debug_b = config.m_debug_b;
        component_config_.m_extractROI_b = config.m_extractROI_b;
        component_config_.m_printTimings_b = config.m_printTimings_b;
        component_config_.m_writeResults_b = config.m_writeResults_b;
        component_config_.m_sigma_gaussian_filter_d = config.m_sigma_gaussian_filter_d;
        component_config_.m_minH_thresh_d = config.m_minH_thresh_d;
        component_config_.m_minI_thresh_d = config.m_minI_thresh_d;
        component_config_.m_felz_sigma_d = config.m_felz_sigma_d;
        component_config_.m_felz_k_i = config.m_felz_k_i;
        component_config_.m_felz_minSize_i = config.m_felz_minSize_i;
        component_config_.m_segmentClassification_thresh_d = config.m_segmentClassification_thresh_d;
        component_config_.m_morphSize_i = config.m_morphSize_i;
        component_config_.m_Vdisp_thresh_d = config.m_Vdisp_thresh_d;
        component_config_.m_minValidDisp_thresh_d = config.m_minValidDisp_thresh_d;
        component_config_.m_resolution_H_d = config.m_resolution_H_d;
        component_config_.m_resolution_I_d = config.m_resolution_I_d;
        component_config_.m_Udisp_thresh_d = config.m_Udisp_thresh_d;
        component_config_.m_maxDelay_d = config.m_maxDelay_d;
        component_config_.m_alpha_d = config.m_alpha_d;
        component_config_.m_maxH_thresh_d = config.m_maxH_thresh_d;
        component_config_.m_maxI_thresh_d = config.m_maxI_thresh_d;
        component_config_.m_priorityAccuracy_b = config.m_priorityAccuracy_b;
        component_config_.m_minIntensity_i = config.m_minIntensity_i;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_segment_freespace_mask_active = true;
        component_data_.out_colorHistogram_freespace_mask_active = true;
        component_data_.out_disparity_freespace_mask_active = true;
        component_data_.out_final_freespace_mask_active = true;
        component_data_.out_freeSpacePoints_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_segment_freespace_mask_active)
            segment_freespace_mask_.publish(component_data_.out_segment_freespace_mask);
        if (component_data_.out_colorHistogram_freespace_mask_active)
            colorHistogram_freespace_mask_.publish(component_data_.out_colorHistogram_freespace_mask);
        if (component_data_.out_disparity_freespace_mask_active)
            disparity_freespace_mask_.publish(component_data_.out_disparity_freespace_mask);
        if (component_data_.out_final_freespace_mask_active)
            final_freespace_mask_.publish(component_data_.out_final_freespace_mask);
        if (component_data_.out_freeSpacePoints_active)
            freeSpacePoints_.publish(component_data_.out_freeSpacePoints);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "free_space_detection_node");

    free_space_detection_node_ros node;
    node.configure();

    ros::Rate loop_rate(30.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
