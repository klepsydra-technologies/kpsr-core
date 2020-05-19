/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

#include <klepsydra/core/prop_file_environment.h>

kpsr::PropertyFileEnvironment::PropertyFileEnvironment(const std::string & propertyFileName)
     : _configFile(propertyFileName)
{}

kpsr::PropertyFileEnvironment::PropertyFileEnvironment(std::istream & propertyFileContent)
     : _configFile(propertyFileContent)
{}

void kpsr::PropertyFileEnvironment::getPropertyString(const std::string& key, std::string & value, std::string const& rootNode) {
	value = _configFile.getValueOfKey<std::string>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyInt(const std::string& key, int & value, std::string const& rootNode) {
	value = _configFile.getValueOfKey<int>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyFloat(const std::string& key, float & value, std::string const& rootNode) {
	value = _configFile.getValueOfKey<float>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyBool(const std::string& key, bool & value, std::string const& rootNode) {
	value = _configFile.getValueOfKey<bool>(key);
}

void kpsr::PropertyFileEnvironment::setPropertyString(const std::string& key, const std::string & value, std::string const& rootNode) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyInt(const std::string& key, const int & value, std::string const& rootNode) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyFloat(const std::string& key, const float & value, std::string const& rootNode) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyBool(const std::string& key, const bool & value, std::string const& rootNode) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::persist() {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::loadFile(const std::string& fileName, const std::string& nodeName) {
    // Unsupported
}
