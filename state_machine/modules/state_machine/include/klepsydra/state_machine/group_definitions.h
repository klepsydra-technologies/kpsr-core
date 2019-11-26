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
