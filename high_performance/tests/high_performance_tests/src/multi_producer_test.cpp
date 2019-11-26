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

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <math.h>

#include <sstream>
#include <fstream>

#include "spdlog/spdlog.h"
#include <spdlog/sinks/basic_file_sink.h>

#include <klepsydra/core/cache_listener.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

#include "gtest/gtest.h"

struct Event {
    long value = -1;
    bool ready = false;
};

class int_handler : public disruptor4cpp::event_handler<Event>
{
public:
    int_handler() = default;
    virtual ~int_handler() = default;
    virtual void on_start() { }
    virtual void on_shutdown() { }
    virtual void on_event(Event & event, int64_t sequence, bool end_of_batch)
    {
        spdlog::info("Received event: {}. event ready: {}. sequence: {}. end_of_batch: {}", event.value, (event.ready ? "true" : "false"), sequence, (end_of_batch ? "true" : "false"));
    }
    virtual void on_timeout(int64_t sequence) { }
    virtual void on_event_exception(const std::exception& ex, int64_t sequence, Event * event) { }
    virtual void on_start_exception(const std::exception& ex) { }
    virtual void on_shutdown_exception(const std::exception& ex) { }
};

TEST(MultiproducerTest, NominalCase) {
    // Create the ring buffer.
    disruptor4cpp::ring_buffer<Event, 8, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::multi> ring_buffer;

    // Create and run the consumer on another thread.
    auto barrier = ring_buffer.new_barrier();
    int_handler handler;
    disruptor4cpp::batch_event_processor<decltype(ring_buffer)> processor(ring_buffer, std::move(barrier), handler);

    std::vector<disruptor4cpp::sequence * > sequences_to_add;
    sequences_to_add.resize(1);
    sequences_to_add[0] = &(processor.get_sequence());
    ring_buffer.add_gating_sequences(sequences_to_add);

    std::thread processor_thread([&processor] { processor.run(); });

    for (int i = 0; i < 9; i++)
    {
        int64_t seq = ring_buffer.try_next();
        ring_buffer[seq].ready = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ring_buffer[seq].value = i + 1000;
        ring_buffer[seq].ready = true;
        ring_buffer.publish(seq);
    }


    // Stop the consumer.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    processor.halt();
    processor_thread.join();
}
