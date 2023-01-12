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

#include <thread>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <klepsydra/socket_core/protocol_simple.h>
#include <klepsydra/socket_core/server_socket.h>

// Usage:
//     server path <path>
// or
//     server port <port>
//

int main(int argc, char *argv[])
{
    std::shared_ptr<kpsr::socket_mdlw::ServerSocket> serverPtr;
    if (1 == argc) {
        std::shared_ptr<kpsr::socket_mdlw::ServerSocket> server(
            new kpsr::socket_mdlw::ServerSocket("/tmp/CO_command_socket"));
        serverPtr = server;
    } else if (3 == argc) {
        std::string option(argv[1]);
        if ("path" == option) {
            std::string path(argv[2]);
            std::shared_ptr<kpsr::socket_mdlw::ServerSocket> server(
                new kpsr::socket_mdlw::ServerSocket(path));
            serverPtr = server;
        } else if ("port" == option) {
            int port = std::atoi(argv[2]);
            std::shared_ptr<kpsr::socket_mdlw::ServerSocket> server(
                new kpsr::socket_mdlw::ServerSocket(port));
            serverPtr = server;
        } else {
            spdlog::error("Usage: server path <hostpath>\n"
                          "Or\n:   server port <port>\n");
            std::exit(1);
        }
    } else {
        spdlog::error("Usage: server path <hostpath>\n"
                      "Or\n:   server port <port>\n");
        std::exit(1);
    }
    int finished = 0;
    while (!finished) {
        std::shared_ptr<kpsr::socket_mdlw::DataSocket> accept = serverPtr->accept();

        std::thread socketThread([accept]() {
            kpsr::socket_mdlw::ProtocolSimple acceptSimple(accept);
            std::string message;
            acceptSimple.recvMessage(message);
            spdlog::info(message);

            acceptSimple.sendMessage("OK");
        });
        socketThread.detach();
    }
}
