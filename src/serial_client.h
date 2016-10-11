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
#ifndef DMAC_SERIAL_CLIENT_H
#define DMAC_SERIAL_CLIENT_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "parser.h"
#include "config.h"
#include "comm_middlemen.h"

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace dmac
{

class serial_client : public dmac::comm_middlemen
{
  public:
  serial_client(boost::asio::io_service& io_service, dmac::config &config)
      : io_service_(io_service),
        reconnect_timer_(io_service),
        capacity_(4096),
        parser_(io_service, config, this)
  {
      mem_.resize(capacity_);
      config.load();
      parser_.ctrl(dmac::EOL, "\r");
      connect();
  }      

  ~serial_client()
  {
      if (port_) {
          port_->cancel();
          port_->close();
          port_.reset();
      }
      io_service_.stop();
      io_service_.reset();
  }

  void do_close()
  {
      ROS_WARN_STREAM("connection closed...");
      parser_.disconnected();
      if (port_) {
          port_->cancel();
          port_->close();
          port_.reset();
      }
      reconnect_timer_.cancel();
      reconnect_timer_.expires_from_now(boost::posix_time::milliseconds(1000));
      reconnect_timer_.async_wait(boost::bind(&serial_client::reconnect_timeout, this,
                                              boost::asio::placeholders::error));
  }
    
  void reconnect_timeout(const boost::system::error_code& error)
  {
      if (error == boost::asio::error::operation_aborted) {
          return;
      }
      ROS_INFO_STREAM("reconnecting");
      connect();
  }

  void connect()
  {
      if (port_) {
          ROS_WARN_STREAM("error : port is already opened...");
          do_close();
          return;
      }

      std::string portname;
      ros::param::param<std::string>(ros::this_node::getName() + "/modem_config/serial_config/port", portname, "/dev/ttyUSB0");
      
      port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
      boost::system::error_code ec;
      port_->open(portname, ec);
      if (ec) {
          ROS_WARN_STREAM("error : port_->open() failed... portname=" << portname << ", e=" << ec.message().c_str());
          do_close();
          return;
      }
      
      int baudrate;
      ros::param::param<int>(ros::this_node::getName() + "/modem_config/serial_config/baudrate", baudrate, 19200);
      port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
      port_->set_option(boost::asio::serial_port_base::character_size(8));
      port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

      parser_.connected();
      port_->async_read_some( 
          boost::asio::buffer(&mem_[0], capacity_),
          boost::bind(
              &serial_client::on_receive_,
              this, boost::asio::placeholders::error, 
              boost::asio::placeholders::bytes_transferred));
  }

  void on_receive_(const boost::system::error_code& ec, size_t bytes_transferred)
  {
      if (!ec)
      {
          parser_.to_term(mem_, bytes_transferred);
          port_->async_read_some( 
              boost::asio::buffer(&mem_[0], capacity_),
              boost::bind(
                  &serial_client::on_receive_,
                  this, boost::asio::placeholders::error, 
                  boost::asio::placeholders::bytes_transferred));
      }
      else
      {
          do_close();
      }
  }

  void handle_write(const boost::system::error_code& error, std::size_t bytes_transferred)
  {
      if (error)
      {
          do_close();
      }
  }

  void send(std::string &msg)
  {
      port_->async_write_some(
          boost::asio::buffer(msg, msg.length()),
          boost::bind(&serial_client::handle_write, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred));
  }

  private:
    boost::asio::io_service& io_service_;
    boost::asio::deadline_timer reconnect_timer_;
    serial_port_ptr port_;
    size_t capacity_;
    std::vector<uint8_t> mem_;
    dmac::parser parser_;
};

} // namespace

#endif  // DMAC_SERIAL_CLIENT_H
