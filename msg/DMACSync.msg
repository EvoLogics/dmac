# AcousticModemSync.msg
#   sends AT command to the acoustic modem

Header header      # message header

string command     # AT command
string parameters  # AT command parameters
string report      # synchronous AT reply