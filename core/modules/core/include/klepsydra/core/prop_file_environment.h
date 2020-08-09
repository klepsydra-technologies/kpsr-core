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

#ifndef PROP_FILE_ENVIRONMENT_H
#define PROP_FILE_ENVIRONMENT_H

#include <klepsydra/core/environment.h>
#include <klepsydra/core/config_file.h>

namespace kpsr {
/**
 * @brief The PropertyFileEnvironment class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details A PROP_FILE based implementation of the environment. Properties are read and written to the member PROP_FILE file.
 */
class PropertyFileEnvironment : public Environment {
public:
    /**
     * @brief PropertyFileEnvironment
     * @param propertyFileName
     */
    explicit PropertyFileEnvironment(const std::string & propertyFileName);

    /**
     * @brief PropertyFileEnvironment
     */
    explicit PropertyFileEnvironment(std::istream & propertyFileContent);

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string& key, std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string& key, int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string& key, float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string& key, bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string& key, const std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string& key, const int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string& key, const float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string& key, const bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief persist
     */
    void persist();

    /**
     * \brief reload
     * \param propertyFileContent
     */
    void reload(std::string propertyFileContent, std::string const& rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief exportEnvironment
     * @return
     */
    std::string exportEnvironment();

    virtual void loadFile(const std::string& fileName, const std::string& nodeName);
protected:
	ConfigFile _configFile;
};
}

#endif // PROP_FILE_ENVIRONMENT_H
