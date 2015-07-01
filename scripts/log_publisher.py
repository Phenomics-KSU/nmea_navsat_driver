#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import os.path
import serial

import rospy

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser

from nmea_navsat_driver.msg import GGK

if __name__ == '__main__':
    rospy.init_node('log_publisher')

    directory = rospy.get_param('~directory', 'none')
    
    ggk_pub = rospy.Publisher('ggk', GGK, queue_size=1000)
    
    if not os.path.isdir(directory):
        rospy.logwarn('{} is not a valid directory. Exiting.'.format(directory))
        sys.exit(1)
    
    messages = []
    for fname in os.listdir(directory):
        extension = os.path.splitext(fname)[1][1:]
        if extension != 'cap':
            print 'Skipping {} due to incorrect extension.'.format(fname)
            continue
        with open(os.path.join(directory, fname)) as log_file:
            messages.extend(log_file.readlines())
    
    positions = []
    for nmea_string in messages:
    
        nmea_string = nmea_string.strip()
    
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                              "Sentence was: %s" % nmea_string)
            continue
    
        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logwarn("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            continue
    
        if 'GGK' in parsed_sentence:
            
            data = parsed_sentence['GGK']
            
            ggk = GGK()
            #ggk.header.stamp = rospy.get_rostime()
            ggk.header.frame_id = 'gps'

            ggk.utc_time = data['utc_time']
            
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            ggk.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            ggk.longitude = longitude
            
            # Strip off EHT prefix and convert to float
            try:
                altitude = data['height_above_ellipsoid']
                altitude = float(altitude[3:])
            except ValueError:
                altitude = 0
            
            ggk.altitude = altitude

            ggk.gps_quality = data['gps_quality']
            ggk.dop = data['dop']
            ggk.sats_used = data['sats_used']

            positions.append(ggk)
    
    rospy.loginfo('Extracted {} position messages.'.format(len(positions)))
    
    rospy.loginfo('Sorting by utc time.')
    positions = sorted(positions, key=lambda item: item.utc_time)
    
    rospy.loginfo('Publishing position messages.')
    rate = rospy.Rate(100) # hz
    try:
        for position in positions:
            ggk_pub.publish(position)
            rate.sleep()
    except rospy.ROSInterruptException:
        print 'interrupted'

    rospy.loginfo('Finished publishing log messages.')