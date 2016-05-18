#   sends message to the acoustic modem for transmission

uint8 DMAC_BURST = 0
# Valid azimuth
uint8 DMAC_IM = 1
# Valid position (usually assumes valid range and azimuth)
uint8 DMAC_IMS = 2
# Relative Position (usually assumes valid range and azimuth)
uint8 DMAC_PBM = 3

Header header                           # message header

uint32 msg_id                           # unique message id (for acknowledgement management)
uint8 source_address                    # source address 
uint8 destination_address               # destination address 
string source_name
string destination_name

uint8 type # media access type: BURST/IM/IMS/PBM

bool ack                                # if the message is to be acknowledged
bool force                              # if the message is forced to be sent in BACKOFF (requires ack false)
uint32 bitrate
float32 rssi                            # received signal strength indicator (dB re 1V)
float32 integrity                       # values above 100 are considered good
uint32 propagation_time                 # microseconds
uint32 duration                         # microseconds
uint32 timestamp                        # timestamp of the message that has been received or 
                                        # needs to be sent (in microseconds)
bool timestamp_undefined                # if true, timestamp field undefined
float32 relative_velocity               # relative velocity between communicating devices in m/s

string payload                          # the message payload

diagnostic_msgs/KeyValue[] info         # optional field for additional data
