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

#ifndef NON_COPYING_STREAM_BUFFER_H
#define NON_COPYING_STREAM_BUFFER_H

#include <streambuf>
#include <string>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The NonCopyingStringBuffer class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class NonCopyingStringBuffer : public std::basic_streambuf<char> {
public:
    /**
     * @brief NonCopyingStringBuffer
     * @param data
     * @param size
     */
    NonCopyingStringBuffer(char * data, long size) {
        setg(data, data, data + size);
    }
};
}
}
#endif // NON_COPYING_STREAM_BUFFER_H
