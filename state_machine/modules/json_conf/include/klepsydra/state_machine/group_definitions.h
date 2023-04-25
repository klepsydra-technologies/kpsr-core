/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
