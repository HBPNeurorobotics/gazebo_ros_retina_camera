/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "gazebo_ros_retina_camera/gazebo_ros_retina_camera.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/fill_image.h>

#include <CImg.h>

using namespace cimg_library;
using namespace image_transport;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(RetinaCameraPlugin)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  RetinaCameraPlugin::RetinaCameraPlugin():
  _fov(6),
  _range(10)
  {

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  RetinaCameraPlugin::~RetinaCameraPlugin()
  {
    if(this->_input_retina_image != nullptr)
        delete this->_input_retina_image;

    for(auto it: _itList) {
        delete it;
    }

    ROS_DEBUG_STREAM_NAMED("retina_camera","Unloaded");
  }

  void RetinaCameraPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);

    //----retina init----
    _retina.setColumns(this->width);
    _retina.setRows(this->height);

    //read script path from sdf element
    std::string retinaPath;

    if (_sdf->HasElement("retinaScriptPath"))
      retinaPath = _sdf->GetElement("retinaScriptPath")->Get<std::string>();
    else
      gzwarn << "[gazebo_ros_retina_camera] Please specify a retina script path." << endl;

    //load the retina configuration script
    try {
        _retina.loadCircuit(retinaPath);

    } catch(std::runtime_error& e) {

        gzerr << "[gazebo_ros_retina_camera] " << e.what() << endl;
        ROS_FATAL("gazebo_ros_retina_camera plugin error: %s \n",  e.what());

        return;
    }

    const string MODULE_TOPICS_PREFIX = "retina";

    for(Module* m :_retina.modules) {

        std::string currModuleID = m->getID();

        if (!currModuleID.empty()) {
            //std::cout << "MODULE: "<< m->getID() << std::endl;

            _retinaModuleIDList.push_back(currModuleID);
            //create publishers for each module (defined in the retina script)
            auto currIt = new image_transport::ImageTransport(*this->rosnode_);

            _itList.push_back(currIt);

            auto data_pub = (this->rosnode_)->advertise<std_msgs::Float64MultiArray>(MODULE_TOPICS_PREFIX + "/" + currModuleID + "/data", 1);
            auto image_pub = currIt->advertise(MODULE_TOPICS_PREFIX + "/" + currModuleID, 1);

            _ID2PubsMap[currModuleID] = make_pair(data_pub, image_pub);
        }
    }

    // allocate an uninitialized image
    this->_input_retina_image = new CImg<double>(this->width, this->height, 1, 3);

    //activate the sensor: Retina needs image update even when there are no subscribers to the camera
    this->parentSensor->SetActive(true);
  }
  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void RetinaCameraPlugin::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    assert(_width == this->width && _height == this->height);
    assert(_format == "R8G8B8");

    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->sensor_update_time_ >= this->update_period_)
    {
        //----------publish input image on the camera topic----------
        if ((*this->image_connect_count_) > 0) {
            this->PutCameraData(_image);
            this->PublishCameraInfo();
            this->last_update_time_ = cur_time;
        }

        //---------Convert input image to CImage<double>---------
        convertRGBtoCImg(_image, this->_input_retina_image);

        //----------update retina with the new image----------
        _retina.update(this->_input_retina_image);

        //----------Publish retina layers IF the topics are subscribed----------
        for(auto const &entry : _ID2PubsMap) { //(ID, (dataPub, imagePub))

        auto const &currModule_ID = entry.first;
        auto const &currPublishersPair = entry.second; //(dataPub, imagePub)

        auto const &currDataPublisher = currPublishersPair.first;
        auto const &currImagePublisher = currPublishersPair.second;

        bool const isDataTopicSubscribed = (currDataPublisher.getNumSubscribers() > 0) ;
        bool const isImageTopicSubscribed = (currImagePublisher.getNumSubscribers() > 0) ;

        if(isDataTopicSubscribed) { //publish data first, since in this case the retina_output is not modified
           //read retina-processed image for the current layer---------
           CImg<double>* pRetina_output = _retina.getOutput(currModule_ID);

           publishCImgAsData(*pRetina_output, currDataPublisher);
        }

        if(isImageTopicSubscribed) {
            //copy retina-processed image for the current layer because we have to modify it
            CImg<double> retina_output = *(_retina.getOutput(currModule_ID)); //copy image

            //normalize and apply heatmap
            retina_output.normalize(0,255).map(CImg<double>::jet_LUT256());

            // publish to ROS
            publishCImgAsRGB(retina_output, currImagePublisher);
        }
      }
    }
  }

  void RetinaCameraPlugin::convertRGBtoCImg(const unsigned char *_image_buffer, CImg<double>* converted_image) {

    const unsigned int NUM_CHANNELS = 3;

    cimg_forXYC(*converted_image, x, y, c) {
        auto pixel_idx = NUM_CHANNELS * (x + y* width); // idx of the n-th pixel in the buffer (R8G8B8)
        (*converted_image)(x, y, c) = _image_buffer[pixel_idx+c];
    }
  }

  void RetinaCameraPlugin::publishCImgAsRGB(const CImg<double>& image, const image_transport::Publisher& publisher) {

    //allocate output buffer
    const auto OUTPUT_NUM_CHANNELS = 3;
    const unsigned int OUTPUT_BUFF_LEN = OUTPUT_NUM_CHANNELS * (width * height);

    unsigned char output_image[OUTPUT_BUFF_LEN];

    //convert image
    convertCImgToRGB(image, output_image);

    //create message
    sensor_msgs::Image image_msg;

    image_msg.header.frame_id = this->frame_name_;
    image_msg.header.stamp.sec = this->sensor_update_time_.sec;
    image_msg.header.stamp.nsec = this->sensor_update_time_.nsec;
    // copy from output_image to image_msg
    fillImage(image_msg, this->type_, this->height_, this->width_,
        this->skip_*this->width_, reinterpret_cast<const void*>(output_image));

    // publish to ROS
    publisher.publish(image_msg);
  }

  void RetinaCameraPlugin::convertCImgToRGB(const CImg<double>& input_image, unsigned char* output_buffer) {

    const auto OUTPUT_NUM_CHANNELS = 3;

    if(input_image.spectrum() == 1) { // grayscale image

        cimg_forXY(input_image, x, y) {
            auto pixel_idx = OUTPUT_NUM_CHANNELS * (x + y*width); // idx of the n-th pixel in the buffer

            output_buffer[pixel_idx] =
                output_buffer[pixel_idx + 1] =
                    output_buffer[pixel_idx + 2] = (unsigned char)(input_image)(x, y, 0);
        }
    }
    else { // == 3 -> color
        cimg_forXYC(input_image, x, y, c) {
            auto pixel_idx = OUTPUT_NUM_CHANNELS * (x + y*width);
            output_buffer[pixel_idx + c] = (unsigned char)(input_image)(x, y, c);
        }
    }

  }

  void RetinaCameraPlugin::publishCImgAsData(const CImg<double>& image, const ros::Publisher& publisher) {

    //----Create the message----
    std_msgs::Float64MultiArray msg;

    fillDataMessage(msg, image);

    //----publish the message----
    publisher.publish(msg);

  }

  void RetinaCameraPlugin::fillDataMessage(std_msgs::Float64MultiArray& msg, const CImg<double>& image) {

    const auto NUM_CHANNELS = 1; //image.spectrum();

    /*
    [std_msgs/Float64MultiArray]:
    std_msgs/MultiArrayLayout layout
      std_msgs/MultiArrayDimension[] dim
        string label
        uint32 size
        uint32 stride
      uint32 data_offset
    float64[] data
    */

    //MultiArrayLayout
    msg.layout.dim.resize(3);
    msg.layout.dim[0].label = "height";
    msg.layout.dim[0].size = height;
    msg.layout.dim[0].stride = NUM_CHANNELS*height*width;

    msg.layout.dim[1].label = "width";
    msg.layout.dim[1].size = width;
    msg.layout.dim[1].stride = NUM_CHANNELS*width;

    msg.layout.dim[2].label = "channel";
    msg.layout.dim[2].size = NUM_CHANNELS;
    msg.layout.dim[2].stride = NUM_CHANNELS;

    msg.layout.data_offset = 0;

    msg.data.resize(image.size());

    cimg_forXY(image, x_column, y_row) {
        auto pixel_idx = NUM_CHANNELS * (x_column + y_row*width); // idx of the n-th pixel in the buffer

        msg.data[pixel_idx] = (image)(x_column, y_row, 0);
    }

  }

}
