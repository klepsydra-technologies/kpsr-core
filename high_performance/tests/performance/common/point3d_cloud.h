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

    Point3dCloud(const std::string & label, std::vector<Point3d> values)
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
