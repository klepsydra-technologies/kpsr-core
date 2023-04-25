// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include <klepsydra/state_machine/json_config_loader.h>
#include <klepsydra/state_machine/kpsr_state_machine.h>

TEST(StateMachine, test1)
{
    kpsr::fsm::JsonConfigLoader cnfLoader;
    kpsr::fsm::ConfigStateMachine cnfSm = cnfLoader.loadConfig(TEST_DATA "/sm1.json");
    ASSERT_EQ(cnfSm.states.size(), 3);
    ASSERT_EQ(cnfSm.states[0].id, "st1");
    ASSERT_EQ(cnfSm.states[1].id, "st2");
    ASSERT_EQ(cnfSm.states[2].id, "st3");

    ASSERT_EQ(cnfSm.states[0].transitions.size(), 2);
    ASSERT_EQ(cnfSm.states[0].transitions[0].destinationId, "st2");
    ASSERT_EQ(cnfSm.states[0].transitions[0].event, "event1");
    ASSERT_EQ(cnfSm.states[0].transitions[1].destinationId, "st3");
    ASSERT_EQ(cnfSm.states[0].transitions[1].event, "event2");

    ASSERT_EQ(cnfSm.states[1].transitions.size(), 1);

    ASSERT_EQ(cnfSm.states[2].transitions.size(), 1);
    ASSERT_EQ(cnfSm.states[2].transitions[0].destinationId, "st2");
    ASSERT_EQ(cnfSm.states[2].transitions[0].event, "event1");
}

TEST(StateMachine, test2)
{
    struct Observer
    {
        std::string currentState;
        void updateCurrentState(const std::string &currentState, bool stateChanged)
        {
            this->currentState = currentState;
        }
    };
    auto smObserver = std::make_shared<Observer>();
    auto sm = kpsr::fsm::FromJson::createStateMachine(TEST_DATA "/sm1.json");
    sm->registerObserver(std::bind(&Observer::updateCurrentState,
                                   smObserver,
                                   std::placeholders::_1,
                                   std::placeholders::_2));
    sm->start();

    sm->enqueueAndUpdate("event1");
    ASSERT_EQ(smObserver->currentState, "sm1:st2");
    sm->enqueueEvent("event2");
    sm->update();
    ASSERT_EQ(smObserver->currentState, "sm1:st2");

    sm->reset();
    sm->start();
    sm->enqueueAndUpdate("event2");
    ASSERT_EQ(smObserver->currentState, "sm1:st3");
    sm->enqueueAndUpdate("event3");
    ASSERT_EQ(smObserver->currentState, "sm1:st3");
    sm->enqueueAndUpdate("event1");
    ASSERT_EQ(smObserver->currentState, "sm1:st2");
}

TEST(StateMachine, test3)
{
    struct Observer
    {
        std::string currentState;
        void updateCurrentState(const std::string &currentState, bool stateChanged = true)
        {
            this->currentState = currentState;
        }
    };
    auto smObserver = std::make_shared<Observer>();
    auto sm = kpsr::fsm::FromJson::createStateMachine(TEST_DATA "/sm1.json");
    sm->registerObserver(std::bind(&Observer::updateCurrentState,
                                   smObserver,
                                   std::placeholders::_1,
                                   std::placeholders::_2));
    sm->start();

    sm->enqueueEvent("event1");
    sm->enqueueEvent("event2");
    sm->update();
    ASSERT_EQ(smObserver->currentState, "sm1:st2");
    sm->update();
    ASSERT_EQ(smObserver->currentState, "sm1:st2");

    sm->reset();
    sm->start();
    sm->enqueueEvent("event2");
    sm->enqueueEvent("event3");
    sm->enqueueEvent("event1");
    sm->update();
    ASSERT_EQ(smObserver->currentState, "sm1:st3");
    sm->update();
    ASSERT_EQ(smObserver->currentState, "sm1:st2");
}

TEST(StateMachine, test4)
{
    struct ServiceMock
    {
        std::string currentState;
        std::string act1Msg;
        std::string act2Msg;
        std::shared_ptr<kpsr::fsm::StateMachine> sm;
        std::shared_ptr<kpsr::fsm::StateMachineListener> smListener;
        ServiceMock(const std::string &specPath)
        {
            smListener = std::make_shared<kpsr::fsm::StateMachineListener>("sm1");
            smListener->addAction("st1", std::bind(&ServiceMock::act1, this, std::placeholders::_1));
            smListener->addAction("st2", std::bind(&ServiceMock::act2, this, std::placeholders::_1));

            sm = kpsr::fsm::FromJson::createStateMachine(specPath);
            sm->registerObserver(smListener->getObserverFunc());
            sm->registerObserver(std::bind(&ServiceMock::updateCurrentState,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));

            sm->start();
        }
        void updateCurrentState(const std::string &currentState, bool stateChanged = true)
        {
            this->currentState = currentState;
        }
        void update() { sm->update(); }
        void pushEvent(const std::string &event) { sm->enqueueEvent(event); }
        void act1(const std::string &stateId)
        {
            act2Msg = "act2-not-executed";
            act1Msg = "act1-executed";
        }
        void act2(const std::string &stateId)
        {
            act1Msg = "act1-not-executed";
            act2Msg = "act2-executed";
        }
    };

    auto svc = std::make_shared<ServiceMock>(TEST_DATA "/sm1.json");

    svc->pushEvent("event2");
    svc->pushEvent("event1");

    ASSERT_EQ(svc->currentState, "sm1:st1");

    ASSERT_EQ(svc->act2Msg, "act2-not-executed");
    ASSERT_EQ(svc->act1Msg, "act1-executed");

    svc->update();
    ASSERT_EQ(svc->currentState, "sm1:st3");

    ASSERT_EQ(svc->act2Msg, "act2-not-executed");
    ASSERT_EQ(svc->act1Msg, "act1-executed");

    svc->update();
    ASSERT_EQ(svc->currentState, "sm1:st2");

    ASSERT_EQ(svc->act1Msg, "act1-not-executed");
    ASSERT_EQ(svc->act2Msg, "act2-executed");
}

TEST(StateMachine, test5)
{
    struct ServiceMock
    {
        std::string currentState;
        std::string act1Msg;
        std::string act2Msg;
        std::shared_ptr<kpsr::fsm::StateMachine> sm;
        std::shared_ptr<kpsr::fsm::StateMachineListener> smListener;
        ServiceMock(const std::string &specPath)
        {
            smListener = std::make_shared<kpsr::fsm::StateMachineListener>("sm1");
            smListener->addOneOffAction("st1",
                                        std::bind(&ServiceMock::act1, this, std::placeholders::_1));
            smListener->addAction("st2", std::bind(&ServiceMock::act2, this, std::placeholders::_1));

            sm = kpsr::fsm::FromJson::createStateMachine(specPath);
            sm->registerObserver(smListener->getObserverFunc());
            sm->registerObserver(std::bind(&ServiceMock::updateCurrentState,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));

            sm->start();
        }
        void updateCurrentState(const std::string &currentState, bool stateChanged = true)
        {
            this->currentState = currentState;
        }
        void update() { sm->update(); }
        void pushEvent(const std::string &event) { sm->enqueueEvent(event); }
        void act1(const std::string &stateId)
        {
            act2Msg = "act2-not-executed";
            act1Msg = "act1-executed";
        }
        void act2(const std::string &stateId)
        {
            act1Msg = "act1-not-executed";
            act2Msg = "act2-executed";
        }
    };

    auto svc = std::make_shared<ServiceMock>(TEST_DATA "/sm1.json");

    svc->pushEvent("event2");
    svc->pushEvent("event1");
    svc->pushEvent("event3");

    ASSERT_EQ(svc->currentState, "sm1:st1");

    ASSERT_EQ(svc->act2Msg, "act2-not-executed");
    ASSERT_EQ(svc->act1Msg, "act1-executed");

    svc->update();
    ASSERT_EQ(svc->currentState, "sm1:st3");

    ASSERT_EQ(svc->act2Msg, "act2-not-executed");
    ASSERT_EQ(svc->act1Msg, "act1-executed");

    svc->update();
    ASSERT_EQ(svc->currentState, "sm1:st2");

    ASSERT_EQ(svc->act1Msg, "act1-not-executed");
    ASSERT_EQ(svc->act2Msg, "act2-executed");

    svc->update();
    ASSERT_EQ(svc->currentState, "sm1:st1");

    ASSERT_EQ(svc->act1Msg, "act1-not-executed");
    ASSERT_EQ(svc->act2Msg, "act2-executed");
}

TEST(StateMachine, test6)
{
    struct ServiceMock
    {
        std::string currentState;
        std::string act1Msg;
        std::shared_ptr<kpsr::fsm::StateMachine> sm;
        std::shared_ptr<kpsr::fsm::StateMachineListener> smListener;
        int act1Counter = 0;
        int act2Counter = 0;
        ServiceMock(const std::string &specPath)
        {
            smListener = std::make_shared<kpsr::fsm::StateMachineListener>("sm1");
            smListener->addPeriodicAction("st1",
                                          std::bind(&ServiceMock::act1,
                                                    this,
                                                    std::placeholders::_1));

            sm = kpsr::fsm::FromJson::createStateMachine(specPath);
            sm->registerObserver(smListener->getObserverFunc());
            sm->registerObserver(std::bind(&ServiceMock::updateCurrentState,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));

            sm->start();
        }
        void updateCurrentState(const std::string &currentState, bool stateChanged = true)
        {
            this->currentState = currentState;
        }
        void update() { sm->update(); }
        void pushEvent(const std::string &event) { sm->enqueueEvent(event); }
        void act1(const std::string &stateId)
        {
            act1Msg = "act1-executed-" + std::to_string(act1Counter);
            act1Counter++;
        }
    };

    auto svc = std::make_shared<ServiceMock>(TEST_DATA "/sm1.json");

    ASSERT_EQ(svc->currentState, "sm1:st1");

    svc->update();
    ASSERT_EQ(svc->act1Msg, "act1-executed-1");
    svc->update();
    ASSERT_EQ(svc->act1Msg, "act1-executed-2");
    svc->update();
    ASSERT_EQ(svc->act1Msg, "act1-executed-3");
    svc->update();
    ASSERT_EQ(svc->act1Msg, "act1-executed-4");
}
