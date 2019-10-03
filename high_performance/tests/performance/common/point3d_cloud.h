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

#ifndef POINT3D_CLOUD_H
#define POINT3D_CLOUD_H

class Point3dCloud {
public:

    static int constructorInvokations;
    static int copyConstructorInvokations;
    static int emptyConstructorInvokations;

    struct Point3d {
        double x;
        double y;
        double z;
    };

    Point3dCloud(std::string label, std::vector<Point3d> values)
        : _label(label)
        , _values(values) {
        Point3dCloud::constructorInvokations++;
    }

    Point3dCloud() {
        Point3dCloud::emptyConstructorInvokations++;
    }

    Point3dCloud(const Point3dCloud & that)
        : _label(that._label)
        , _values(that._values) {
        Point3dCloud::copyConstructorInvokations++;
    }

    std::string _label;
    std::vector<Point3d> _values;
};

int Point3dCloud::constructorInvokations = 0;
int Point3dCloud::emptyConstructorInvokations = 0;
int Point3dCloud::copyConstructorInvokations = 0;

#endif // POINT3D_CLOUD_H
