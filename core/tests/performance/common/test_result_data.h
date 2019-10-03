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

#ifndef TEST_RESULT_DATA_H
#define TEST_RESULT_DATA_H

#include <numeric>

class TestResults {
public:
    long average;
    long stddev;

    TestResults(std::vector<long> totalTime) {
        long sum = std::accumulate(totalTime.begin(), totalTime.end(), 0.0);
        average = sum / totalTime.size();

        long sq_sum = std::inner_product(totalTime.begin(), totalTime.end(), totalTime.begin(), 0.0);
        stddev = std::sqrt((sq_sum / totalTime.size()) - (average * average));
    }
};

#endif // TEST_RESULT_DATA_H
