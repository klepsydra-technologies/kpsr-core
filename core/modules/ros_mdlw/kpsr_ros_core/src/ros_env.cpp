/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#include "ros_env.h"
#include <string.h>

kpsr::ros_mdlw::RosEnv::RosEnv(ros::NodeHandle * nodeHandle) {
	this->nodeHandle = nodeHandle;
}

void kpsr::ros_mdlw::RosEnv::getPropertyString(const std::string key, std::string & value) {
    nodeHandle->getParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyInt(const std::string key, int & value) {
        nodeHandle->getParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyFloat(const std::string key, float & value) {
        nodeHandle->getParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::getPropertyBool(const std::string key, bool & value) {
        nodeHandle->getParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyString(const std::string key, const std::string value) {
	nodeHandle->setParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyInt(const std::string key, const int & value) {
	nodeHandle->setParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyFloat(const std::string key, const float & value) {
	nodeHandle->setParam(key, value);
}

void kpsr::ros_mdlw::RosEnv::setPropertyBool(const std::string key, const bool & value) {
	nodeHandle->setParam(key, value);
}

