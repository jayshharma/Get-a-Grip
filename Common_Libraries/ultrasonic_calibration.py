# Ultrasonic sensors will come with different calibrations (offsets) depending on when and where
# they are purchased; so, a calibration curve needs to be created for each sensor for accuracy.
# For simplicity, we will use an offset for now.
# Note: The sensors used on Q-Bot RP-105 to RP-115 are were purchased from the some supplier on the
# same day. Therefore, their offsets are the same.

from subprocess import check_output


x = check_output(['hostname','-I']) # returns the IP address as byte encoded as utf-8

ip_str = x.decode('utf-8').split()[0] # return string in the form: '172.17.160.1xx \n'.

if ip_str == '172.17.160.113': # RP-105
    us_sensor_offset = 36.8

elif ip_str == '172.17.160.114': # RP-106
    us_sensor_offset = 35.9

elif ip_str == '172.17.160.115': # RP-107
    us_sensor_offset = 36.2

elif ip_str == '172.17.160.116': # RP-108
    us_sensor_offset = 37.1

elif ip_str == '172.17.160.117': # RP-109
    us_sensor_offset = 36.3

elif ip_str == '172.17.160.118': # RP-110
    us_sensor_offset = 36.7

elif ip_str == '172.17.160.119': # RP-111
    us_sensor_offset = 36.2

elif ip_str == '172.17.160.120': # RP-112
    us_sensor_offset = 37.1

elif ip_str == '172.17.160.121': # RP-113
    us_sensor_offset = 36.7

elif ip_str == '172.17.160.122': # RP-114
    us_sensor_offset = 37.1

else:
    us_sensor_offset = 37.1

