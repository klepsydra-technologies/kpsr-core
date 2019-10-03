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

#ifndef OBJECT_POOL_PUBLISHER_H
#define OBJECT_POOL_PUBLISHER_H

#include <iostream>

#include <klepsydra/core/publisher.h>
#include <klepsydra/core/smart_object_pool.h>

namespace kpsr
{
template <class T>
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
    ObjectPoolPublisher(Container * container,
                        const std::string name,
                        const std::string type,
                        int poolSize,
                        std::function<void(T &)> initializerFunction,
                        std::function<void(const T &, T &)> eventCloner)
        : Publisher<T>(container, name, type)
        , _initializerFunction(initializerFunction)
        , _eventCloner(eventCloner)
    {
        _objectPool = poolSize == 0 ? nullptr : new SmartObjectPool<T>(poolSize, initializerFunction);
        this->_publicationStats._totalEventAllocations = poolSize;
    }

    /*!
     * @brief internalPublish
     * @param eventData
     */
    void internalPublish(const T & eventData) override {
        if (_objectPool != nullptr) {
            try {
                std::shared_ptr<T> newEvent = std::move(_objectPool->acquire());
                if (_eventCloner == nullptr) {
                    (*newEvent.get()) = eventData;
                } else {
                    _eventCloner(eventData, (*newEvent.get()));
                }
                internalPublish(newEvent);
                return;
            } catch (std::out_of_range ex) {
                //TODO: log
                std::cout << "ObjectPoolPublisher::internalPublish. Object Pool failure." << std::endl;
            }
        }
        this->_publicationStats._totalEventAllocations++;
        if (_eventCloner == nullptr) {
            std::shared_ptr<T> newEvent = std::shared_ptr<T>(new T(eventData));
            internalPublish(newEvent);
        } else {
            T * t = new T();
            if (_initializerFunction != nullptr) {
                _initializerFunction(* t);
            }
            std::shared_ptr<T> newEvent = std::shared_ptr<T>(t);
            _eventCloner(eventData, (*newEvent.get()));
            internalPublish(newEvent);
        }
    }

    /*!
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(T &)> process) {
        if (_objectPool != nullptr) {
            try {
                std::shared_ptr<T> newEvent = std::move(_objectPool->acquire());
                if (_initializerFunction != nullptr) {
                    _initializerFunction(*newEvent);
                }
                process(*newEvent);
                internalPublish(newEvent);
                return;
            } catch (std::out_of_range ex) {
                //TODO: log
                std::cout << "ObjectPoolPublisher::processAndPublish. Object Pool failure." << std::endl;
            }
        }
        this->_publicationStats._totalEventAllocations++;
        T * t = new T();
        if (_initializerFunction != nullptr) {
            _initializerFunction(*t);
        }
        std::shared_ptr<T> newEvent = std::shared_ptr<T>(t);
        process(*newEvent);
        internalPublish(newEvent);
    }

    /*!
     * @brief internalPublish abstract method that is implemented by the different concrete classes.
     * @param process
     */
    virtual void internalPublish(std::shared_ptr<const T> eventData) = 0;

private:
    std::function<void(T &)> _initializerFunction;
    SmartObjectPool<T> * _objectPool;
    std::function<void(const T &, T &)> _eventCloner;
};
}
#endif
