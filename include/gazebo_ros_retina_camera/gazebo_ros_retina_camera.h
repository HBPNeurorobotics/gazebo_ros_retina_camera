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

#ifndef GAZEBO_ROS_RETINA_CAMERA_HH
#define GAZEBO_ROS_RETINA_CAMERA_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <corem/retina.h>

#include <CImg.h>
using namespace cimg_library;

namespace gazebo
{
  class RetinaCameraPlugin : public CameraPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: RetinaCameraPlugin();

    /// \brief Destructor
    public: ~RetinaCameraPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format);

    vector<image_transport::ImageTransport*> _itList;

    std::map<string, std::pair<ros::Publisher, image_transport::Publisher>> _ID2PubsMap;
    vector<string> _retinaModuleIDList;

    double _fov;
    double _range;

    Retina _retina;

    CImg<double>* _input_retina_image = nullptr;

    // Set _current_retina_image with the data of image. image is in R8G8B8 format
    /// \param image An image stored in R8G8B8 format
    private:
        void convertRGBtoCImg(const unsigned char *image, CImg<double>* converted_image);

        void convertCImgToRGB(const CImg<double>& input_image, unsigned char* output_buffer);

        void publishCImgAsRGB(const CImg<double>& image, const image_transport::Publisher& publisher);

        void publishCImgAsData(const CImg<double>& image, const ros::Publisher& publisher);
        void fillDataMessage(std_msgs::Float64MultiArray& msg, const CImg<double>& image);
  };
}
#endif