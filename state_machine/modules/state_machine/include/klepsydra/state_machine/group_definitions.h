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
*****************************************************************************/

#ifndef GROUP_DEFINITIONS_H
#define GROUP_DEFINITIONS_H
/**
 * @defgroup kpsr-application Application API
 *
 * This group of classes contains the API for the application development. That means that the client codes that
 * involves the actual logic of the application, should use this and only this API.
 *
 */

/**
 * @defgroup kpsr-test Test API
 *
 * This group of classes are meant to be used in the unit testing of the application. They provide facilities
 * to test middleware publishing and subscription along with some other syntatic sugar for testing.
 *
 */

/**
 * @defgroup kpsr-composition Composition API
 *
 * This group of classes relates exclusively to the assemblying of the application. In Spring terms, the 'wiring'
 * of the application is done using this API.
 *
 */

/**
 * @defgroup kpsr-monitoring Monitoring API
 *
 * This group of classes is intended to monitor the performance of the application along with allowing some administration
 * tasks like restart services and read and write configurration.
 *
 */

#endif // GROUP_DEFINITIONS_H
