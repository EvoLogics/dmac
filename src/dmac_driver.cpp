/* Copyright (c) 2016, Oleksiy Kebkal <lesha@evologics.de>
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met: 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 * 3. The name of the author may not be used to endorse or promote products 
 *    derived from this software without specific prior written permission. 
 * 
 * Alternatively, this software may be distributed under the terms of the 
 * GNU General Public License ("GPL") version 2 as published by the Free 
 * Software Foundation. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <string>
#include <sstream>

#include "include/comm_middlemen.h"
#include "include/tcp_client.h"
#include "include/config.h"

namespace std {
    template<typename T>
    std::string to_string(const T &n) {
        std::ostringstream s;
        s << n;
        return s.str();
    }
}

int main(int argc, char* argv[])
{
  // Initialize ROS.
  ros::init(argc, argv, "dmac_node");

  dmac::config config;
  
  std::string IP, type;
  int port;

  ros::param::param<std::string>(ros::this_node::getName() + "/modem_config/connection_type", type, "TCP/IP");
  boost::asio::io_service io_service;
  dmac::tcp_client *s = NULL;
  
  if (type == "TCP/IP") {

      ros::param::param<std::string>(ros::this_node::getName() + "/modem_config/tcp_config/ip", IP, "192.168.6.2");
      ros::param::param<int>(ros::this_node::getName() + "/modem_config/tcp_config/port", port, 9200);
      
      ROS_INFO_STREAM("Connecting to IP: " << IP << ", port: " << port);

      ros::param::param<bool>(ros::this_node::getName() + "/modem_config/hasAHRS", config.hasAHRS, false);
      ROS_INFO_STREAM("hasAHRS: " << config.hasAHRS);

      config.node_name = ros::this_node::getName();
      
      // Listen for TCP connections in background thread.
      tcp::resolver resolver(io_service);
      tcp::resolver::query query(IP, std::to_string(port));
      tcp::resolver::iterator iterator = resolver.resolve(query);

      dmac::tcp_client *s = new dmac::tcp_client(io_service, iterator, config);
  } else {
      ROS_ERROR_STREAM("Unsupported connection type: " << type);
      /* TODO: add serial support */
      return -1;
  }
  
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
  ROS_INFO_STREAM("Spinning... ");
  ros::spin();
  
  return 0;
}
