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


// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef CLASS_WITH_RELATED_CLASSES
#define CLASS_WITH_RELATED_CLASSES

#include <string>
#include <vector>

#include "basic_class.h"

class ClassWithRelatedClasses
{
public:
    ClassWithRelatedClasses() {}

    ClassWithRelatedClasses(
        string i, BasicClass ii, int32 iii, std::vector<BasicClass> iv, std::vector<int32> v)
        : i(i)
        , ii(ii)
        , iii(iii)
        , iv(iv)
        , v(v)
    {}

    ClassWithRelatedClasses(const ClassWithRelatedClasses &that)
        : i(that.i)
        , ii(that.ii)
        , iii(that.iii)
        , iv(that.iv)
        , v(that.v)
    {}

    void clone(const ClassWithRelatedClasses &that)
    {
        this->i = that.i;
        this->ii = that.ii;
        this->iii = that.iii;
        this->iv = that.iv;
        this->v = that.v;
    }

    string i;
    BasicClass ii;
    int32 iii;
    std::vector<BasicClass> iv;
};

#endif // CLASS_WITH_RELATED_CLASSES
