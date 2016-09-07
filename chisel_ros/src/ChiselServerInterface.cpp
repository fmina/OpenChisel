/*
 Copyright (c) 2016, Mina Kamel, ASL, ETH Zurich, Switzerland
 You can contact the author at <mina.kamel@mavt.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "chisel_ros/ChiselServerInterface.h"

namespace chisel_ros {

extern "C" void ChiselServerInterface_querry_sdf_and_gradient(void* chisel_server_ptr, double position[3], double attitude_rpy[3], double *sdf, double *gradient){
	Eigen::Vector3d UAV_position(position[0], position[1], position[2]);

	if (chisel_server_ptr == NULL){
		*sdf = 0;
		gradient[0] = 0;
		gradient[1] = 0;
		gradient[2] = 0;
		return;
	}


	Eigen::Quaterniond q_W_B;
	double L_2 = 1.0/2.0;
	double h_2 = 0.5/2.0;

	tf::Quaternion tf_q_W_B(attitude_rpy[2], attitude_rpy[1], attitude_rpy[0]);
	tf::quaternionTFToEigen(tf_q_W_B, q_W_B);

	std::vector<Eigen::Vector3d> points_B;
	std::vector<double> sdfs_value;
	std::vector<Eigen::Vector3f> gradients;

	points_B.push_back(Eigen::Vector3d(L_2, L_2, h_2));
	points_B.push_back(Eigen::Vector3d(L_2, -L_2, h_2));
	points_B.push_back(Eigen::Vector3d(-L_2, L_2, h_2));
	points_B.push_back(Eigen::Vector3d(-L_2, -L_2, h_2));

	points_B.push_back(Eigen::Vector3d(L_2, L_2, -h_2));
	points_B.push_back(Eigen::Vector3d(L_2, -L_2, -h_2));
	points_B.push_back(Eigen::Vector3d(-L_2, L_2, -h_2));
	points_B.push_back(Eigen::Vector3d(-L_2, -L_2, -h_2));

	for (size_t i = 0; i < 8; i++) {
		Eigen::Vector3f eigen_gradient(0,0,0);
		Eigen::Vector3f point_W = (UAV_position
				+ q_W_B * points_B.at(i)).cast<float>();
		double temp_sdf = 999999.0;

		static_cast<ChiselServerInterface*>(chisel_server_ptr)->GetSDFAndGradient(
				point_W, &temp_sdf, &eigen_gradient);
//		ROS_INFO_STREAM("gradient @ "<< point_W.transpose() << "\tin chisel: " << eigen_gradient.transpose() << "\tsdf: " << temp_sdf);


		sdfs_value.push_back(temp_sdf);
		gradients.push_back(eigen_gradient);
	}



 	std::vector<double>::iterator minimum_sdf = std::min_element(sdfs_value.begin(), sdfs_value.end());

 	*sdf = *minimum_sdf;
 	int idx = minimum_sdf - sdfs_value.begin();

// 	ROS_INFO_STREAM_THROTTLE(0.1,"gradient @ "<< UAV_position.transpose() << "\tin chisel: " << gradients.at(idx).transpose() << "\tsdf: " << *sdf);

	gradient[0] = gradients.at(idx).x();
	gradient[1] = gradients.at(idx).y();
	gradient[2] = gradients.at(idx).z();

}

ChiselServerInterface::ChiselServerInterface(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):
	nh_(nh),
	private_nh_(private_nh, "chisel"){

//	nh_ = ros::NodeHandle();
//	private_nh_ = ros::NodeHandle("~/chisel");
//	nh_.setCallbackQueue(&callback_queue_);
//	private_nh_.setCallbackQueue(&callback_queue_);

	private_nh_.param("chunk_size_x", chunkSizeX_, 32);
	private_nh_.param("chunk_size_y", chunkSizeY_, 32);
	private_nh_.param("chunk_size_z", chunkSizeZ_, 32);
	private_nh_.param("truncation_constant", truncationDistConst_, 0.001504);
	private_nh_.param("truncation_linear", truncationDistLinear_, 0.00152);
	private_nh_.param("truncation_quadratic", truncationDistQuad_, 0.0019);
	private_nh_.param("truncation_scale", truncationDistScale_, 8.0);
	private_nh_.param("use_constant_truncator", useConstantTruncator_, false);
	private_nh_.param("constant_truncation_value", constantTruncationValue_, 0.5);
	private_nh_.param("integration_weight", weight_, 1);
	private_nh_.param("use_voxel_carving", useCarving_, true);
	private_nh_.param("use_color", useColor_, true);
	private_nh_.param("carving_dist_m", carvingDist_, 0.05);
	private_nh_.param("voxel_resolution_m", voxelResolution_, 0.03);
	private_nh_.param("near_plane_dist", nearPlaneDist_, 0.05);
	private_nh_.param("far_plane_dist", farPlaneDist_, 5.0);
	private_nh_.param("depth_image_topic", depthImageTopic_, std::string("/depth_image"));
	private_nh_.param("point_cloud_topic", pointCloudTopic_,
			std::string("/camera/depth_registered/points"));
	private_nh_.param("depth_image_info_topic", depthImageInfoTopic_,
			std::string("/depth_camera_info"));
	private_nh_.param("depth_image_transform", depthImageTransform_,
			std::string("/camera_depth_optical_frame"));
	private_nh_.param("color_image_topic", colorImageInfoTopic_, std::string("/color_image"));
	private_nh_.param("color_image_info_topic", colorImageInfoTopic_,
			std::string("/color_camera_info"));
	private_nh_.param("color_image_transform", colorImageTransform_,
			std::string("/camera_rgb_optical_frame"));
	private_nh_.param("base_transform", baseTransform_, std::string("/camera_link"));
	private_nh_.param("mesh_topic", meshTopic_, std::string("full_mesh"));
	private_nh_.param("chunk_box_topic", chunkBoxTopic_, std::string("chunk_boxes"));
	private_nh_.param("fusion_mode", modeString_, std::string("DepthImage"));

	if(modeString_ == "DepthImage")
	    {
	        ROS_INFO("Mode depth image");
	        mode_ = chisel_ros::ChiselServer::FusionMode::DepthImage;
	    }
	    else if(modeString_ == "PointCloud")
	    {
	        ROS_INFO("Mode point cloud");
	        mode_ = chisel_ros::ChiselServer::FusionMode::PointCloud;
	    }
	    else
	    {
	        ROS_ERROR("Unrecognized fusion mode %s. Recognized modes: \"DepthImage\", \"PointCloud\"\n", modeString_.c_str());
	        abort();
	    }

	    ROS_INFO("Subscribing.");
	    chisel::Vec4 truncation(truncationDistQuad_, truncationDistLinear_, truncationDistConst_, truncationDistScale_);

	    server_.reset(new chisel_ros::ChiselServer(nh, chunkSizeX_, chunkSizeY_, chunkSizeZ_, voxelResolution_, useColor_, mode_));
	    if(useConstantTruncator_){
	    	server_->SetupProjectionIntegrator(constantTruncationValue_, static_cast<uint16_t>(weight_), useCarving_, carvingDist_);
	    }else{
	    	server_->SetupProjectionIntegrator(truncation, static_cast<uint16_t>(weight_), useCarving_, carvingDist_);
	    }
	    if (mode_ == chisel_ros::ChiselServer::FusionMode::DepthImage)
	    {
	        server_->SubscribeDepthImage(depthImageTopic_, depthImageInfoTopic_, depthImageTransform_);
	    }
	    else
	    {
	        server_->SubscribePointCloud(pointCloudTopic_);
	    }

	    server_->SetupDepthPosePublisher("last_depth_pose");
	    server_->SetupDepthFrustumPublisher("last_depth_frustum");

	    server_->SetNearPlaneDist(nearPlaneDist_);
	    server_->SetFarPlaneDist(farPlaneDist_);
	    server_->AdvertiseServices();

	    if (useColor_ && mode_ == chisel_ros::ChiselServer::FusionMode::DepthImage)
	    {
	        server_->SubscribeColorImage(colorImageTopic_, colorImageInfoTopic_, colorImageTransform_);
	        server_->SetupColorPosePublisher("last_color_pose");
	        server_->SetupColorFrustumPublisher("last_color_frustum");
	    }

	    server_->SetBaseTransform(baseTransform_);
	    server_->SetupMeshPublisher(meshTopic_);
	    server_->SetupChunkBoxPublisher(chunkBoxTopic_);
	    ROS_INFO("Beginning to loop.");

	    main_loop_timer_ = private_nh_.createTimer(ros::Duration(0.01), &ChiselServerInterface::main_loop_callback, this);

}

ChiselServerInterface::~ChiselServerInterface() {
	// TODO Auto-generated destructor stub
}

void ChiselServerInterface::main_loop_callback(const ros::TimerEvent&){
//	ROS_INFO("chisel timer is running");
    if(!server_->IsPaused() && server_->HasNewData())
    {
//        ROS_INFO("Got data.");
        switch (server_->GetMode())
        {
            case chisel_ros::ChiselServer::FusionMode::DepthImage:
                server_->IntegrateLastDepthImage();
                break;
            case chisel_ros::ChiselServer::FusionMode::PointCloud:
                server_->IntegrateLastPointCloud();
                break;
        }

        server_->PublishMeshes();
        server_->PublishChunkBoxes();

        if(mode_ == chisel_ros::ChiselServer::FusionMode::DepthImage)
        {
            server_->PublishDepthPose();
            server_->PublishDepthFrustum();

            if(useColor_)
            {
                server_->PublishColorPose();
                server_->PublishColorFrustum();
            }
        }
    }
}

} /* namespace mav_obstacle_detection */
