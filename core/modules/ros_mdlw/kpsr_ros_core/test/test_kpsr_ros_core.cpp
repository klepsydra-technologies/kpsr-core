#include <string>

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include "to_ros_middleware_provider.h"
#include "from_ros_middleware_provider.h"
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

	// Create a publisher.
    ros::Publisher publisher = nodeHandle.advertise<typename TypeParam::MyB>(topicName, 1);

    // Create a handler to the ros provider using klepsydra.
    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    // Create klepsydra publisher.
    kpsr::Publisher<typename TypeParam::MyA> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<
	    typename TypeParam::MyA, typename TypeParam::MyB>(topicName, 1, nullptr, publisher);

    // Set up subscribers.
    kpsr::mem::BasicMiddlewareProvider<typename TypeParam::MyA> safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
    safeQueueProvider.start();

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<typename TypeParam::MyA, typename TypeParam::MyB>(
	    topicName.c_str(), 1, safeQueueProvider.getPublisher());

    kpsr::mem::CacheListener<typename TypeParam::MyA> cacheListener;
    safeQueueProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

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
    ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>(topicName, 1);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<std::string> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>(topicName, 1, nullptr, stringPublisher);

    kpsr::mem::BasicMiddlewareProvider<std::string> safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
    safeQueueProvider.start();

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<std::string, std_msgs::String>(topicName.c_str(), 1, safeQueueProvider.getPublisher());

    kpsr::mem::CacheListener<std::string> cacheListener;
    safeQueueProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

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

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    safeQueueProvider.stop();

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(*cacheListener.getLastReceivedEvent().get(), "hola.5");
}

