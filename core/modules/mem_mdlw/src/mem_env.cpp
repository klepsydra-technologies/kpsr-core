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

#include <klepsydra/mem_core/mem_env.h>

void kpsr::mem::MemEnv::getPropertyString(const std::string key, std::string & value) {
   pthread_mutex_lock(&lock);
   value = stringDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::getPropertyInt(const std::string key, int & value) {
   pthread_mutex_lock(&lock);
   value = intDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::getPropertyFloat(const std::string key, float & value) {
   pthread_mutex_lock(&lock);
   value = floatDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::getPropertyBool(const std::string key, bool & value) {
   pthread_mutex_lock(&lock);
   value = boolDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyString(const std::string key, const std::string value) {
   pthread_mutex_lock(&lock);
   stringDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyInt(const std::string key, const int & value) {
   pthread_mutex_lock(&lock);
   intDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyFloat(const std::string key, const float & value) {
   pthread_mutex_lock(&lock);
   floatDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyBool(const std::string key, const bool & value) {
   pthread_mutex_lock(&lock);
   boolDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}
