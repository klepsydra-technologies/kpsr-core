#include <string>
#include <algorithm>

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include "to_ros_middleware_provider.h"
#include "from_ros_middleware_provider.h"
#include "ros_env.h"
#include <typeinfo>  // Needed to check type.

// Setting up templates and params for google typed parameter tests.

// Structure to make google test to believe there is only one template parameter.
template <typename A, typename B>
struct TypeDefinitions
{
  typedef  A MyA;
  typedef  B MyB;
};


// Actual ros class to be tested for pair of templates.
// typename A refers to native C object
// typename B refers to the Ros primitive type.
template  <typename A, typename B>
class KpsrRosType
{
public:
	KpsrRosType() {}

	A sampleA;
	B sampleB;
};


//Constructors for above class.
template <typename A, typename B>
KpsrRosType<A, B>* CreateKpsrClass()
{
	return new KpsrRosType < A, B >;
}


template <typename T>
/*
  \brief Class to convert 1 template params into 2 params

  Google test needs only one template parameter, so this class allows
  us to take a template struct made of up two template parameters and
  gives access to both.
 */
class KpsrRosWrapperTest : public ::testing::Test {
protected:
	KpsrRosWrapperTest() : testClass(CreateKpsrClass<typename T::MyA, typename T::MyB>()){}
	virtual ~KpsrRosWrapperTest() { delete testClass;}
	KpsrRosType<typename T::MyA, typename T::MyB>* const testClass;
};


// The list of types we want to test. Each pair refers to the native c
// object & the Ros equivalent msg type.
typedef ::testing::Types <TypeDefinitions<bool,std_msgs::Bool>,
                          TypeDefinitions<int,std_msgs::Int32>,
                          TypeDefinitions<long, std_msgs::Int64>,
                          TypeDefinitions<float, std_msgs::Float32> > Implementations;

TYPED_TEST_CASE(KpsrRosWrapperTest, Implementations);

// Check if above classes work as expected.
TYPED_TEST(KpsrRosWrapperTest, DefaultConstructor)
{
	typename TypeParam::MyA itemA;
	std::string typeA = typeid(itemA).name();
	std::string typeSampleA = typeid(this->testClass->sampleA).name();

	typename TypeParam::MyB itemB;
	std::string typeB = typeid(itemB).name();
	std::string typeSampleB = typeid(this->testClass->sampleB).name();

	ASSERT_EQ(typeA, typeSampleA);
	ASSERT_EQ(typeB, typeSampleB);
}

// Check if publisher/subscribers work and data is coherent.
TYPED_TEST(KpsrRosWrapperTest, nomicalCaseNoPool) {
	// Set up ros node.
	int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    std::string topicName("kpsr_ros_core_test_topic");

    // Set up subscribers.
    kpsr::mem::BasicMiddlewareProvider<typename TypeParam::MyA> safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
    safeQueueProvider.start();

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<typename TypeParam::MyA, typename TypeParam::MyB>(
	    topicName.c_str(), 1, safeQueueProvider.getPublisher());

    kpsr::mem::CacheListener<typename TypeParam::MyA> cacheListener;
    safeQueueProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

	// Create a publisher.
    ros::Publisher publisher = nodeHandle.advertise<typename TypeParam::MyB>(topicName, 1, true);

    // Create a handler to the ros provider using klepsydra.
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    // Create klepsydra publisher.
    kpsr::Publisher<typename TypeParam::MyA> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<
	    typename TypeParam::MyA, typename TypeParam::MyB>(topicName, 1, nullptr, publisher);

    ASSERT_EQ(cacheListener.counter, 0);

    typename TypeParam::MyA item;  // Initialize to default value.
    // Check if item is nan or -nan
    if (item != item)
    {
	    item = 0.0;
    }
    else
    {
	    ++item;
	    // This raises warning for boolean, but sets the boolean to False.
    }
		    
    kpsrPublisher->publish(item);
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter != 1) {
        ros::spinOnce();
        rate.sleep();
    }

    safeQueueProvider.stop();

    ASSERT_EQ(cacheListener.counter, 1);
    ASSERT_EQ(static_cast<typename TypeParam::MyA>(*cacheListener.getLastReceivedEvent().get()), item);
}


// String is not included in the templated classes. this case is for that.
TEST(KpsrRosCoreTest, nominalCaseNoPoolString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    std::string topicName = "kpsr_ros_core_test_topic";
    kpsr::mem::BasicMiddlewareProvider<std::string> safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<std::string, std_msgs::String>(topicName.c_str(), 1, safeQueueProvider.getPublisher());

    safeQueueProvider.start();

    kpsr::mem::CacheListener<std::string> cacheListener;
    safeQueueProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 1, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<std::string> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>(topicName, 1, nullptr, stringPublisher);

    kpsrPublisher->publish("hola.1");
    ros::spinOnce();
    rate.sleep();
    kpsrPublisher->publish("hola.2");
    ros::spinOnce();
    rate.sleep();
    kpsrPublisher->publish("hola.3");
    ros::spinOnce();
    rate.sleep();
    kpsrPublisher->publish("hola.4");
    ros::spinOnce();
    rate.sleep();
    kpsrPublisher->publish("hola.5");
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter < 5 && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    safeQueueProvider.stop();

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(*cacheListener.getLastReceivedEvent().get(), "hola.5");
}

////// Int test ///////////////
/// Set params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootSetInt) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const int sampleValue(10);

    environment.setPropertyInt(testKey, sampleValue);

    int checkValue;
    ASSERT_TRUE(nodeHandle.hasParam(testKey));
    nodeHandle.getParam(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootSetInt) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const int sampleValue(10);

    environment.setPropertyInt(testKey, sampleValue, testRootNode);

    int checkValue;
    ASSERT_FALSE(nodeHandle.hasParam(testKey));
    std::string actualKey ("/" + testRootNode + "/" + testKey);
    ASSERT_TRUE(nodeHandle.hasParam(actualKey));
    nodeHandle.getParam(actualKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

/// Get params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootGetInt) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const int sampleValue(10);

    nodeHandle.setParam(testKey, sampleValue);

    int checkValue;
    environment.getPropertyInt(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootGetInt) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const int sampleValue(10);
    std::string actualKey ("/" + testRootNode + "/" + testKey);

    nodeHandle.setParam(actualKey, sampleValue);

    int checkValue;
    environment.getPropertyInt(testKey, checkValue, testRootNode);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

//////// Float tests ///////////////////
/// Set params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootSetFloat) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const float sampleValue(10.0F);

    environment.setPropertyFloat(testKey, sampleValue);

    float checkValue;
    ASSERT_TRUE(nodeHandle.hasParam(testKey));
    nodeHandle.getParam(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootSetFloat) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const float sampleValue(10.0F);

    environment.setPropertyFloat(testKey, sampleValue, testRootNode);

    float checkValue;
    ASSERT_FALSE(nodeHandle.hasParam(testKey));
    std::string actualKey ("/" + testRootNode + "/" + testKey);
    ASSERT_TRUE(nodeHandle.hasParam(actualKey));
    nodeHandle.getParam(actualKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

/// Get params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootGetFloat) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const float sampleValue(10.0F);

    nodeHandle.setParam(testKey, sampleValue);

    float checkValue;
    environment.getPropertyFloat(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootGetFloat) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const float sampleValue(10.0F);
    std::string actualKey ("/" + testRootNode + "/" + testKey);

    nodeHandle.setParam(actualKey, sampleValue);

    float checkValue;
    environment.getPropertyFloat(testKey, checkValue, testRootNode);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

/////////// Bool test ////////////
/// Set params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootSetBool) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const bool sampleValue(true);

    environment.setPropertyBool(testKey, sampleValue);

    bool checkValue;
    ASSERT_TRUE(nodeHandle.hasParam(testKey));
    nodeHandle.getParam(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootSetBool) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const bool sampleValue(false);

    environment.setPropertyBool(testKey, sampleValue, testRootNode);

    bool checkValue;
    ASSERT_FALSE(nodeHandle.hasParam(testKey));
    std::string actualKey ("/" + testRootNode + "/" + testKey);
    ASSERT_TRUE(nodeHandle.hasParam(actualKey));
    nodeHandle.getParam(actualKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

/// Get params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootGetBool) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const bool sampleValue(true);

    nodeHandle.setParam(testKey, sampleValue);

    bool checkValue;
    environment.getPropertyBool(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootGetBool) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const bool sampleValue(false);
    std::string actualKey ("/" + testRootNode + "/" + testKey);

    nodeHandle.setParam(actualKey, sampleValue);

    bool checkValue;
    environment.getPropertyBool(testKey, checkValue, testRootNode);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

////// String test ///////////////
/// Set params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootSetString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");

    environment.setPropertyString(testKey, sampleValue);

    std::string checkValue;
    ASSERT_TRUE(nodeHandle.hasParam(testKey));
    nodeHandle.getParam(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootSetString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");

    environment.setPropertyString(testKey, sampleValue, testRootNode);

    std::string checkValue;
    ASSERT_FALSE(nodeHandle.hasParam(testKey));
    std::string actualKey ("/" + testRootNode + "/" + testKey);
    ASSERT_TRUE(nodeHandle.hasParam(actualKey));
    nodeHandle.getParam(actualKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

/// Get params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootGetString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");

    nodeHandle.setParam(testKey, sampleValue);

    std::string checkValue;
    environment.getPropertyString(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, EnvironmentTestRootGetString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");
    std::string actualKey ("/" + testRootNode + "/" + testKey);

    nodeHandle.setParam(actualKey, sampleValue);

    std::string checkValue;
    environment.getPropertyString(testKey, checkValue, testRootNode);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

////// Char String test ///////////////
/// Set params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootSetCharString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    // const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");

    environment.setPropertyString("testKey", sampleValue);

    std::string checkValue;
    ASSERT_TRUE(nodeHandle.hasParam("testKey"));
    nodeHandle.getParam("testKey", checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam("testKey");
}

TEST(KpsrRosCoreTest, EnvironmentTestRootSetCharString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");

    environment.setPropertyString("testkey", sampleValue, "testroot");

    std::string checkValue;
    ASSERT_FALSE(nodeHandle.hasParam(testKey));
    std::string actualKey ("/" + testRootNode + "/" + testKey);
    ASSERT_TRUE(nodeHandle.hasParam(actualKey));
    nodeHandle.getParam(actualKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}

// /// Get params ///////////////
TEST(KpsrRosCoreTest, EnvironmentTestDefaultRootGetCharString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    
    const std::string sampleValue("sampleValue");

    nodeHandle.setParam("testKey", sampleValue);

    std::string checkValue;
    environment.getPropertyString("testKey", checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam("testKey");
}

TEST(KpsrRosCoreTest, EnvironmentTestRootGetCharString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::ros_mdlw::RosEnv environment(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");
    std::string actualKey ("/" + testRootNode + "/" + testKey);

    nodeHandle.setParam(actualKey, sampleValue);

    std::string checkValue;
    environment.getPropertyString("testkey", checkValue, "testroot");
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}


//////////// Pointer tests

// /// Get params ///////////////
TEST(KpsrRosCoreTest, PointerEnvironmentTestDefaultRootGetString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::Environment * environment = new kpsr::ros_mdlw::RosEnv(&nodeHandle);

    
    const std::string sampleValue("sampleValue");
    const std::string testKey("testkey");
    nodeHandle.setParam(testKey, sampleValue);

    std::string checkValue;
    environment->getPropertyString(testKey, checkValue);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(testKey);
}

TEST(KpsrRosCoreTest, PointerEnvironmentTestRootGetString) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_core_env_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::Environment * environment = new kpsr::ros_mdlw::RosEnv(&nodeHandle);

    const std::string testRootNode("testroot");
    const std::string testKey("testkey");
    const std::string sampleValue("sampleValue");
    std::string actualKey ("/" + testRootNode + "/" + testKey);

    nodeHandle.setParam(actualKey, sampleValue);

    std::string checkValue;
    environment->getPropertyString(testKey, checkValue, testRootNode);
    ASSERT_EQ(sampleValue, checkValue);
    nodeHandle.deleteParam(actualKey);
}
