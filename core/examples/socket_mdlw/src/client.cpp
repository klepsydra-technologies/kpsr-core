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

#include <cstdlib>
#include <iostream>

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
        std::cerr << "Usage: client <host> <Message>\n";
        std::cerr<< "Or\n: client <hostIP> <port> <Message>\n";
        std::exit(1);
    }
	kpsr::socket_mdlw::ProtocolSimple   simpleConnect(connect);
	simpleConnect.sendMessage(messageToSend);
	
	simpleConnect.recvMessage(message);
	std::cout << message << "\n";
}


