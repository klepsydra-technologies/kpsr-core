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

#include <thread>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include <klepsydra/socket_core/server_socket.h>
#include <klepsydra/socket_core/protocol_simple.h>

// Usage:
//     server path <path>
// or
//     server port <port>
//


int main(int argc, char* argv[])
{
	std::shared_ptr<kpsr::socket_mdlw::ServerSocket> serverPtr;
	if (1 == argc) {
		std::shared_ptr<kpsr::socket_mdlw::ServerSocket> server(
			new kpsr::socket_mdlw::ServerSocket("/tmp/CO_command_socket"));
		serverPtr = server;
	}
	else if (3 == argc) {
		std::string option(argv[1]);
		if ("path" == option) {
			std::string path(argv[2]);
			std::shared_ptr<kpsr::socket_mdlw::ServerSocket> server(
				new kpsr::socket_mdlw::ServerSocket(path));
			serverPtr = server;
		}
		else if ("port" == option) {
			int port = std::atoi(argv[2]);
			std::shared_ptr<kpsr::socket_mdlw::ServerSocket> server(
				new kpsr::socket_mdlw::ServerSocket(port));
			serverPtr = server;
		}
		else {
            spdlog::error("Usage: server path <hostpath>\n"
			"Or\n:   server port <port>\n");
			std::exit(1);
		}
	}
	else {
        spdlog::error("Usage: server path <hostpath>\n"
		"Or\n:   server port <port>\n");
		std::exit(1);
	}
    int finished = 0;
    while(!finished)
    {
        std::shared_ptr<kpsr::socket_mdlw::DataSocket> accept  = serverPtr->accept();

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
