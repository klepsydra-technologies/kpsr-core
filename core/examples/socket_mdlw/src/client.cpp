// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <klepsydra/socket_core/connect_socket.h>
#include <klepsydra/socket_core/protocol_simple.h>

// Usage:
//     client <host> <message>
// or
//     client <host_ip> <port> <message>
//

int main(int argc, char *argv[])
{
    std::string messageToSend(argv[argc - 1]);
    std::string message;
    std::shared_ptr<kpsr::socket_mdlw::ConnectSocket> connect;

    if (argc == 3) {
        std::shared_ptr<kpsr::socket_mdlw::ConnectSocket> connectT(
            new kpsr::socket_mdlw::ConnectSocket(argv[1]));
        connect = connectT;
    } else if (argc == 4) {
        int port(std::atoi(argv[2]));
        std::shared_ptr<kpsr::socket_mdlw::ConnectSocket> connectF(
            new kpsr::socket_mdlw::ConnectSocket(argv[1], port));
        connect = connectF;
    } else {
        spdlog::error("Usage: client <host> <Message>\n"
                      "Or\n: client <hostIP> <port> <Message>\n");
        std::exit(1);
    }
    kpsr::socket_mdlw::ProtocolSimple simpleConnect(connect);
    simpleConnect.sendMessage(messageToSend);

    simpleConnect.recvMessage(message);
    spdlog::info(message);
}
