#include <string>

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>
#include <klepsydra/serialization/identity_mapper.h>

#include "to_ros_middleware_provider.h"
#include "from_ros_middleware_provider.h"


template <typename T>
/*
  \brief Class to convert 1 template params into 2 params

  Google test needs only one template parameter, so this class allows
  us to take a template struct made of up two template parameters and
  gives access to both.
 */
class KpsrRosIdentityTest : public ::testing::Test {
protected:
    KpsrRosIdentityTest()
        : testClass() {
    }

    virtual ~KpsrRosIdentityTest () {
    }

    T const testClass;
};

// The list of types we want to test. Each pair refers to the native c
// object & the Ros equivalent msg type.
using Implementations =  ::testing::Types <std_msgs::Bool,
                                           std_msgs::Int32,
                                           std_msgs::Int64,
                                           std_msgs::Float32>;

TYPED_TEST_CASE(KpsrRosIdentityTest, Implementations);


template <typename T>
void callback(const typename T::ConstPtr& msg, T& item) {
    bool val = msg->data == item.data;
    ASSERT_TRUE(val);
    ros::shutdown();
}

TYPED_TEST(KpsrRosIdentityTest, nominalCasePublish) {
    // Set up ros node.
	int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_identity_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    std::string topicName("kpsr_ros_core_identity_topic");

    TypeParam itemToSend; // Random item
    auto subscriber = nodeHandle.subscribe<TypeParam>(topicName, 100,boost::bind(callback<TypeParam>, _1, itemToSend));

	// Create a publisher.
    ros::Publisher publisher = nodeHandle.advertise<TypeParam>(topicName, 1, true);

    // Create a handler to the ros provider using klepsydra.
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    // Create klepsydra publisher.
    auto kpsrPublisher = toRosProvider.getToMiddlewareChannel<TypeParam>(topicName, 1, nullptr, publisher);

    while (ros::ok()) {
        kpsrPublisher->publish(itemToSend);
        ros::spinOnce();
    }
}

TEST(KpsrIdentityTest, nominalCaseStringPublisher) {
    // Set up ros node.
	int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_identity_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    std::string topicName("kpsr_ros_core_identity_topic");

    std_msgs::String itemToSend; // Random item
    itemToSend.data = "Test data: Hello world!";
    auto subscriber = nodeHandle.subscribe<std_msgs::String>(topicName, 100,boost::bind(callback<std_msgs::String>, _1, itemToSend));

	// Create a publisher.
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>(topicName, 1, true);

    // Create a handler to the ros provider using klepsydra.
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    // Create klepsydra publisher.
    auto kpsrPublisher = toRosProvider.getToMiddlewareChannel<std_msgs::String>(topicName, 1, nullptr, publisher);

    while (ros::ok()) {
        kpsrPublisher->publish(itemToSend);
        ros::spinOnce();
    }
}
    
    
TYPED_TEST(KpsrRosIdentityTest, nominalCaseSubscribe) {
    // Set up ros node.
	int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_identity_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    std::string topicName("kpsr_ros_core_identity_topic");

    TypeParam itemToSend; // Random item

    kpsr::mem::BasicMiddlewareProvider <TypeParam > safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
    safeQueueProvider.start();

    // Create a handler to the ros provider using klepsydra.
    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);

    fromRosProvider.registerToTopic<TypeParam>(topicName.c_str(), 1, safeQueueProvider.getPublisher());

    auto kpsrCallback = [&itemToSend](const TypeParam& msg) {
                            bool val = msg.data == itemToSend.data;
                            ASSERT_TRUE(val);
                            ros::shutdown();
                        };
    safeQueueProvider.getSubscriber()->registerListener("messageChecker", kpsrCallback);

	// Create a publisher.
    ros::Publisher publisher = nodeHandle.advertise<TypeParam>(topicName, 1);

    while (ros::ok()) {
        publisher.publish(itemToSend);
        ros::spinOnce();
    }
    safeQueueProvider.stop();
}

TEST(KpsrIdentityTest, nominalCaseSubscribeString) {
    // Set up ros node.
	int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_identity_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    std::string topicName("kpsr_ros_core_identity_topic");

    std_msgs::String itemToSend; // Random item

    kpsr::mem::BasicMiddlewareProvider <std_msgs::String > safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);

    // Create a handler to the ros provider using klepsydra.
    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);

    fromRosProvider.registerToTopic<std_msgs::String>(topicName.c_str(), 1, safeQueueProvider.getPublisher());
    safeQueueProvider.start();

    auto kpsrCallback = [&itemToSend](const std_msgs::String& msg) {
                            bool val = msg.data == itemToSend.data;
                            ASSERT_TRUE(val);
                            ros::shutdown();
                        };
    safeQueueProvider.getSubscriber()->registerListener("messageChecker", kpsrCallback);

	// Create a publisher.
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>(topicName, 1);

    while (ros::ok()) {
        publisher.publish(itemToSend);
        ros::spinOnce();
    }
    safeQueueProvider.stop();
}
