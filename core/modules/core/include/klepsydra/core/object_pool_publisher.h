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

#ifndef OBJECT_POOL_PUBLISHER_H
#define OBJECT_POOL_PUBLISHER_H

#include <spdlog/spdlog.h>

#include <klepsydra/core/publisher.h>
#include <klepsydra/core/smart_object_pool.h>

namespace kpsr {
template<class T>
/*!
 * \brief The ObjectPoolPublisher class
 *
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details Abstract publisher based for most of the concrete implementations (ROS, DDS, ZMQ and event loop.).
 * It has an object pool that pre allocates objects before copying for publication.
 */
class ObjectPoolPublisher : public Publisher<T>
{
public:
    /*!
     * \brief ObjectPoolPublisher
     * \param container Container to attach in case of monitoring.
     * \param name name of the publisher
     * \param type valid values are EVENT_EMITTER, ROS, etc.
     * \param poolSize size of the pool to create.
     * \param initializerFunction init function to execute after creation.
     * \param eventCloner optional function to use instead of the copy.
     */
    ObjectPoolPublisher(Container *container,
                        const std::string &name,
                        const std::string type,
                        int poolSize,
                        std::function<void(T &)> initializerFunction,
                        std::function<void(const T &, T &)> eventCloner)
        : Publisher<T>(container, name, type)
    {
        _objectPool = poolSize == 0 ? nullptr
                                    : new SmartObjectPool<T>(name, poolSize, initializerFunction);
        this->_publicationStats._totalEventAllocations = poolSize;
        if (initializerFunction) {
            _initializerFunction = [this, initializerFunction](T &t) { initializerFunction(t); };
        } else {
            _initializerFunction = [](T &t) {};
        }
        if (eventCloner) {
            _eventCloner = [this, eventCloner](const T &original, T &cloned) {
                eventCloner(original, cloned);
            };
        } else {
            _eventCloner = [](const T &original, T &cloned) { cloned = original; };
        }

        if (_objectPool) {
            _eventCreatorWrapper = [this](const T &eventData) {
                try {
                    std::shared_ptr<T> newEvent = std::move(this->_objectPool->acquire());
                    this->_eventCloner(eventData, *newEvent);
                    return newEvent;
                } catch (const std::out_of_range &ex) {
                    spdlog::info("ObjectPoolPublisher::internalPublish. Object Pool failure. {}",
                                 this->_publicationStats.name);
                    std::shared_ptr<T> newEvent = std::make_shared<T>();
                    this->_initializerFunction(*newEvent);
                    this->_eventCloner(eventData, *newEvent);
                    return newEvent;
                }
            };
            _processEventCreatorWrapper = [this]() {
                std::shared_ptr<T> newEvent;
                try {
                    newEvent = std::move(this->_objectPool->acquire());
                } catch (const std::out_of_range &ex) {
                    spdlog::info("ObjectPoolPublisher::processAndPublish. Object Pool failure. {}",
                                 this->_publicationStats.name);
                    this->_publicationStats._totalEventAllocations++;
                    newEvent = std::make_shared<T>();
                }
                this->_initializerFunction(*newEvent);
                return newEvent;
            };

        } else {
            if (eventCloner == nullptr) {
                // default cloner, so initialization function is not necessary.
                _eventCreatorWrapper = [](const T &eventData) {
                    return std::make_shared<T>(eventData);
                };
            } else {
                _eventCreatorWrapper = [this](const T &eventData) -> std::shared_ptr<T> {
                    std::shared_ptr<T> newEvent = std::make_shared<T>();
                    this->_initializerFunction(*newEvent);
                    this->_eventCloner(eventData, *newEvent);
                    return newEvent;
                };
            }
            _processEventCreatorWrapper = [this]() {
                this->_publicationStats._totalEventAllocations++;
                std::shared_ptr<T> newEvent = std::make_shared<T>();
                this->_initializerFunction(*newEvent);
                return newEvent;
            };
        }
    }

    virtual ~ObjectPoolPublisher()
    {
        if (_objectPool) {
            delete _objectPool;
        }
    }
    /*!
     * @brief internalPublish
     * @param eventData
     */
    void internalPublish(const T &eventData) override
    {
        auto newEvent = _eventCreatorWrapper(eventData);
        internalPublish(newEvent);
    }

    /*!
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(T &)> process)
    {
        auto newEvent = _processEventCreatorWrapper();
        process(*newEvent);
        this->publish(newEvent);
    }

    /*!
     * @brief internalPublish abstract method that is implemented by the different concrete classes.
     * @param process
     */
    virtual void internalPublish(std::shared_ptr<const T> eventData) = 0;

private:
    std::function<void(T &)> _initializerFunction;
    SmartObjectPool<T> *_objectPool;
    std::function<void(const T &, T &)> _eventCloner;

    std::function<std::shared_ptr<T>(const T &eventData)> _eventCreatorWrapper = nullptr;
    std::function<std::shared_ptr<T>(void)> _processEventCreatorWrapper = nullptr;
};
} // namespace kpsr
#endif
