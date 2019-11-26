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

#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

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
