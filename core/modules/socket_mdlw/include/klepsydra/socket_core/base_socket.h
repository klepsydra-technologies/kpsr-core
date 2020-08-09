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

#ifndef BASE_SOCKET_H
#define BASE_SOCKET_H

namespace kpsr
{
namespace socket_mdlw
{
/**
 * @brief The BaseSocket class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details A RAII base class for handling sockets. Socket is movable but not copyable.
 *
 */
class BaseSocket
{
    /**
     * @brief socketId
     */
    int     socketId;
protected:
    /**
     * @brief invalidSocketId
     */
    static constexpr int invalidSocketId      = -1;

    /**
     * @brief Constructor
     *
     * Designed to be a base class not used used directly.
     */
    explicit BaseSocket(int socketId);

    /**
     * @brief getSocketId
     */
    int getSocketId() const {return socketId;}
public:
    /**
     * @brief ~BaseSocket
     */
    virtual ~BaseSocket();

    /**
     * @brief BaseSocket
     * @param move
     * @details Moveable but not Copyable
     */
    BaseSocket(BaseSocket&& move)               noexcept;

    /**
     * @brief operator =
     * @param move
     * @return
     */
    BaseSocket& operator=(BaseSocket&& move)    noexcept;

    /**
     * @brief swap
     * @param other
     */
    void swap(BaseSocket& other)                noexcept;

    /**
     * @brief BaseSocket
     */
    BaseSocket(BaseSocket const&)               = delete;

    /**
     * @brief operator =
     * @return
     */
    BaseSocket& operator=(BaseSocket const&)    = delete;

    /**
     * @brief close
     * @details User can manually call close
     */
    void close();
};

}
}

#endif

