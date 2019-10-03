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

#include <klepsydra/core/prop_file_environment.h>

kpsr::PropertyFileEnvironment::PropertyFileEnvironment(const std::string propertyFileName)
     : _configFile(propertyFileName)
{}

kpsr::PropertyFileEnvironment::PropertyFileEnvironment(std::istream & propertyFileContent)
     : _configFile(propertyFileContent)
{}

void kpsr::PropertyFileEnvironment::getPropertyString(const std::string key, std::string & value) {
	value = _configFile.getValueOfKey<std::string>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyInt(const std::string key, int & value) {
	value = _configFile.getValueOfKey<int>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyFloat(const std::string key, float & value) {
	value = _configFile.getValueOfKey<float>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyBool(const std::string key, bool & value) {
	value = _configFile.getValueOfKey<bool>(key);
}

void kpsr::PropertyFileEnvironment::setPropertyString(const std::string key, const std::string value) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyInt(const std::string key, const int & value) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyFloat(const std::string key, const float & value) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyBool(const std::string key, const bool & value) {
	// Unsupported
}

void kpsr::PropertyFileEnvironment::persist() {
	// Unsupported
}
