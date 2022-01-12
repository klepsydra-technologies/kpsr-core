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
