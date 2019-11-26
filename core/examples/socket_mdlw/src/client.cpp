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

#include <cstdlib>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <klepsydra/socket_core/connect_socket.h>
#include <klepsydra/socket_core/protocol_simple.h>

// Usage:
//     client <host> <message>
// or
//     client <host_ip> <port> <message>
//


int main(int argc, char* argv[])
{
	std::string messageToSend(argv[argc-1]);
	std::string message;
	std::shared_ptr<kpsr::socket_mdlw::ConnectSocket> connect;

	if (argc == 3) {
		std::shared_ptr<kpsr::socket_mdlw::ConnectSocket> connectT(
			new kpsr::socket_mdlw::ConnectSocket(argv[1]));
		connect = connectT;
	}
	else if (argc == 4) {
		int port(std::atoi(argv[2]));
		std::shared_ptr<kpsr::socket_mdlw::ConnectSocket> connectF(
			new kpsr::socket_mdlw::ConnectSocket(argv[1], port));
		connect = connectF;
	}
	else {
        spdlog::error("Usage: client <host> <Message>\n"
        "Or\n: client <hostIP> <port> <Message>\n");
        std::exit(1);
    }
	kpsr::socket_mdlw::ProtocolSimple   simpleConnect(connect);
	simpleConnect.sendMessage(messageToSend);

	simpleConnect.recvMessage(message);
    spdlog::info(message);
}


