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

void kpsr::mem::MemEnv::getPropertyString(const std::string& key, std::string & value, std::string const& rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    value = stringDataMap[key];
}

void kpsr::mem::MemEnv::getPropertyInt(const std::string& key, int & value, std::string const& rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    value = intDataMap[key];
}

void kpsr::mem::MemEnv::getPropertyFloat(const std::string& key, float & value, std::string const& rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    value = floatDataMap[key];
}

void kpsr::mem::MemEnv::getPropertyBool(const std::string& key, bool & value, std::string const& rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    value = boolDataMap[key];
}

void kpsr::mem::MemEnv::setPropertyString(const std::string& key, const std::string & value, std::string const& rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    stringDataMap[key] = value;
}

void kpsr::mem::MemEnv::setPropertyInt(const std::string & key, const int & value, const std::string & rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    intDataMap[key] = value;
}

void kpsr::mem::MemEnv::setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    floatDataMap[key] = value;
}

void kpsr::mem::MemEnv::setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode) {
    std::lock_guard<std::mutex> lock(mutex);
    boolDataMap[key] = value;
}
