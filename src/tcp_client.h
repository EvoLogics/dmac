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
#ifndef DMAC_TCP_CLIENT_H
#define DMAC_TCP_CLIENT_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "parser.h"
#include "config.h"
#include "comm_middlemen.h"

using boost::asio::ip::tcp;

namespace dmac
{

typedef enum { DISCONNECTED = 0, WAITING = 1, CONNECTED = 2 } connection_state;

class tcp_client : public dmac::comm_middlemen
{
public:
  tcp_client(boost::asio::io_service& io_service, tcp::resolver::iterator endpoint_iterator, dmac::config &config)
      : io_service_(io_service),
        capacity_(4096),
        socket_(io_service),
        reconnect_timer_(io_service),
        endpoint_iterator_(endpoint_iterator),
        state_(DISCONNECTED),
        parser_(io_service, config, this)
  {
    mem_.resize(capacity_);
    config.load();
    connect();
  }      

  ~tcp_client()
  {
      io_service_.stop();
  }
  
  void connect()
  {
      state_ = WAITING;
      boost::asio::async_connect(socket_, endpoint_iterator_,
                                 boost::bind(&tcp_client::handle_connect, this,
                                             boost::asio::placeholders::error));
  }
  
  void do_close()
  {
      /* FIXME: what to do if data arrives when socket is closed? */
      if (state_ == DISCONNECTED) {
          ROS_WARN_STREAM("already closed...");
      } else {
          ROS_WARN_STREAM("connection closed...");
          socket_.close();
          parser_.disconnected();
          reconnect_timer_.cancel();
          reconnect_timer_.expires_from_now(boost::posix_time::milliseconds(1000));
          reconnect_timer_.async_wait(boost::bind(&tcp_client::reconnect_timeout, this,
                                                  boost::asio::placeholders::error));
      }
      state_ = DISCONNECTED;
  }

  void send(std::string &msg) 
  {
      if (state_ == CONNECTED) {
          ROS_INFO_STREAM("sending " << msg);
          boost::asio::async_write(socket_,
                                   boost::asio::buffer(msg, msg.length()),
                                   boost::bind(&tcp_client::handle_write, this,
                                               boost::asio::placeholders::error,
                                               boost::asio::placeholders::bytes_transferred));
      } else {
          ROS_WARN_STREAM("cannot send, not connected");
      }
  }
  
  void reconnect_timeout(const boost::system::error_code& error) {
      if (error == boost::asio::error::operation_aborted) {
          return;
      }
      ROS_INFO_STREAM("reconnecting");
      connect();
  }
  
  void handle_connect(const boost::system::error_code& error)
  {
    if (!error)
    {
        state_ = CONNECTED;
        parser_.connected();
        boost::asio::async_read(socket_,
          boost::asio::buffer(&mem_[0], capacity_),
          boost::asio::transfer_at_least(1),
          boost::bind(&tcp_client::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        ROS_WARN_STREAM("do_close by " << __func__);
        do_close();
    }
  }

  void handle_write(const boost::system::error_code& error, std::size_t bytes_transferred)
  {
    if (error)
    {
        ROS_WARN_STREAM("do_close by " << __func__);
        do_close();
    }
  }

  void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred)
  {
    if (!error)
    {
      parser_.to_term(mem_, bytes_transferred);
      
      boost::asio::async_read(socket_,
        boost::asio::buffer(&mem_[0], capacity_),
        boost::asio::transfer_at_least(1),
        boost::bind(&tcp_client::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        ROS_WARN_STREAM("do_close by " << __func__);
        do_close();
    }
  }
  
private:
  connection_state state_;
  boost::asio::io_service& io_service_;
  boost::asio::deadline_timer reconnect_timer_;
  tcp::resolver::iterator endpoint_iterator_;
  tcp::socket socket_;
  size_t capacity_;
  std::vector<uint8_t> mem_;
  dmac::parser parser_;
  
};

}  // namespace

#endif  // DMAC_TCP_CLIENT_H
