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
#ifndef SRC_CHISELSERVERINTERFACE_H_
#define SRC_CHISELSERVERINTERFACE_H_

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <chisel_ros/ChiselServer.h>
#include <tf_conversions/tf_eigen.h>

namespace chisel_ros {

class ChiselServerInterface {
public:
	ChiselServerInterface(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
	~ChiselServerInterface();

	bool GetSDFAndGradient(Eigen::Vector3f position, double *distance, Eigen::Vector3f *gradient){
	    	return server_->GetSDFAndGradient(position, distance, gradient);
	    }


private:
    int chunkSizeX_, chunkSizeY_, chunkSizeZ_;
    double voxelResolution_;
    double truncationDistQuad_;
    double truncationDistLinear_;
    double truncationDistConst_;
    double truncationDistScale_;
    int weight_;
    bool useCarving_;
    bool useColor_;
    bool useConstantTruncator_;
    double constantTruncationValue_;
    double carvingDist_;
    std::string depthImageTopic_;
    std::string depthImageInfoTopic_;
    std::string depthImageTransform_;
    std::string colorImageTopic_;
    std::string colorImageInfoTopic_;
    std::string colorImageTransform_;
    std::string baseTransform_;
    std::string meshTopic_;
    std::string chunkBoxTopic_;
    double nearPlaneDist_;
    double farPlaneDist_;
    chisel_ros::ChiselServer::FusionMode mode_;
    std::string modeString_;
    std::string pointCloudTopic_;

    ros::NodeHandle nh_, private_nh_;
    ros::CallbackQueue callback_queue_;

    chisel_ros::ChiselServerPtr server_;

    ros::Timer main_loop_timer_;

    void main_loop_callback(const ros::TimerEvent&);



};

} /* namespace mav_obstacle_detection */

#endif /* SRC_CHISELSERVERINTERFACE_H_ */
