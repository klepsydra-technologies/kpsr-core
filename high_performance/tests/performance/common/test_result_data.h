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

#ifndef TEST_RESULT_DATA_H
#define TEST_RESULT_DATA_H

#include <math.h>
#include <numeric>
#include <vector>

class TestResults
{
public:
    float sum;
    float average;
    float stddev;

    TestResults(std::vector<long> totalTime)
    {
        sum = std::accumulate(totalTime.begin(), totalTime.end(), 0.0);
        average = sum / totalTime.size();

        float sq_sum = std::inner_product(totalTime.begin(),
                                          totalTime.end(),
                                          totalTime.begin(),
                                          0.0);
        stddev = std::sqrt((sq_sum / totalTime.size()) - (average * average));
    }
};

#endif // TEST_RESULT_DATA_H
