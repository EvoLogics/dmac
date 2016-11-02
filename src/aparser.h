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
#ifndef DMAC_APARSER_H
#define DMAC_APARSER_H

#include "dmac/DMACAsync.h"

using dmac::DMACSync;

namespace dmac
{

typedef enum { DMAC_UNDEF = 0, DMAC_READY = 1 } dmac_filter_state;
typedef enum { DMAC_UNKNOWN = 0, DMAC_AT = 1, DMAC_NET = 2 } dmac_filter_type;
typedef enum { DMAC_DATA_MODE = 0, DMAC_COMMAND_MODE = 1 } dmac_filter_mode;
typedef enum { WAITSYNC_NO = 0,
               WAITSYNC_SINGLELINE = 1,
               WAITSYNC_MULTILINE = 2,
               WAITSYNC_BINARY = 3} dmac_waitsync_status;

typedef enum { WAITSYNC = 0,
               MODE = 1,
               FILTER = 2,
               EOL = 3} parser_state_ctrl;

class abstract_parser
{
  public:
    virtual void ctrl(parser_state_ctrl ctrl, int value) = 0;
    virtual void syncCallback(const dmac::DMACSync::ConstPtr& msg, bool privilege) = 0;
    virtual void syncCallback(const dmac::DMACSync::ConstPtr& msg) = 0;
};

} // namespace

#endif /* DMAC_APARSER_H */
