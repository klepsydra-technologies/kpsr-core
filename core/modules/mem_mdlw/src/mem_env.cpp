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

#include <klepsydra/mem_core/mem_env.h>

void kpsr::mem::MemEnv::getPropertyString(const std::string key, std::string & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   value = stringDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::getPropertyInt(const std::string key, int & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   value = intDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::getPropertyFloat(const std::string key, float & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   value = floatDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::getPropertyBool(const std::string key, bool & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   value = boolDataMap[key];
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyString(const std::string key, const std::string value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   stringDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyInt(const std::string key, const int & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   intDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyFloat(const std::string key, const float & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   floatDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}

void kpsr::mem::MemEnv::setPropertyBool(const std::string key, const bool & value, std::string const& rootNode) {
   pthread_mutex_lock(&lock);
   boolDataMap[key] = value;
   pthread_mutex_unlock(&lock);
}
