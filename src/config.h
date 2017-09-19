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
#ifndef DMAC_CONFIG_H
#define DMAC_CONFIG_H

#include <string>
#include <boost/lexical_cast.hpp>
#include "dmac/DMACSync.h"

using dmac::DMACSync;

namespace dmac
{

class config
{
  public:
    
  config(std::string node_name) :
      hasAHRS_(false),
      node_name_(node_name)
    {
        commands_.clear();
    };

    std::string nodeName()
    {
        return node_name_;
    }

    bool hasAHRS()
    {
        return hasAHRS_;
    }

    std::vector<DMACSyncPtr> commands()
    {
        return commands_;
    }

    std::vector<DMACSyncPtr>::iterator initializer_begin()
    {
        return commands_.begin();
    }

    std::vector<DMACSyncPtr>::iterator initializer_end()
    {
        return commands_.end();
    }
    
    void load(void)
    {
      ros::param::param<bool>(node_name_ + "/modem_config/hasAHRS", hasAHRS_, false);
      ROS_INFO_STREAM("hasAHRS: " << hasAHRS_);
      
      /* default: @ZX1, @ZU1, !C1? */
      pushSync("@CTRL");

      int source_level, local_address;
      ros::param::param<int>(node_name_ + "/modem_config/source_level", source_level, 3);
      pushSync("!L",boost::lexical_cast<std::string>(source_level));

      if (ros::param::has(node_name_ + "/modem_config/local_address")) {
          ros::param::param<int>(node_name_ + "/modem_config/local_address", local_address, 0/**/);
          
          pushSync("!AL",boost::lexical_cast<std::string>(local_address));
      }
      
      std::map<std::string,std::string> ini;
      if (ros::param::has(node_name_ + "/modem_config/initialiser")) {
          ros::param::get(node_name_ + "/modem_config/initialiser", ini);
      } else {
          ros::param::get(node_name_ + "/modem_config/initializer", ini);
      }
      std::map<std::string,std::string>::iterator mit;
      
      /* add ctrl, local address */
      for (mit = ini.begin(); mit != ini.end(); ++mit) {
          pushSync(mit->first,mit->second);
      }
    }

  private:
    bool hasAHRS_;
    std::string node_name_;
    std::vector<DMACSyncPtr> commands_;

    void pushSync(std::string command, std::string parameters = "")
    {
        DMACSyncPtr sync(new DMACSync);
        sync->command = command;
        sync->parameters = parameters;
        commands_.push_back(sync);
    }
};

}  // namespace

#endif  // DMAC_CONFIG_H
