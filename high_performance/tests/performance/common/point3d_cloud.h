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

#ifndef POINT3D_CLOUD_H
#define POINT3D_CLOUD_H

class Point3dCloud
{
public:
    static int constructorInvokations;
    static int copyConstructorInvokations;
    static int emptyConstructorInvokations;

    struct Point3d
    {
        double x;
        double y;
        double z;
    };

    Point3dCloud(const std::string &label, std::vector<Point3d> values)
        : _label(label)
        , _values(values)
    {
        Point3dCloud::constructorInvokations++;
    }

    Point3dCloud() { Point3dCloud::emptyConstructorInvokations++; }

    Point3dCloud(const Point3dCloud &that)
        : _label(that._label)
        , _values(that._values)
    {
        Point3dCloud::copyConstructorInvokations++;
    }

    std::string _label;
    std::vector<Point3d> _values;
};

int Point3dCloud::constructorInvokations = 0;
int Point3dCloud::emptyConstructorInvokations = 0;
int Point3dCloud::copyConstructorInvokations = 0;

#endif // POINT3D_CLOUD_H
