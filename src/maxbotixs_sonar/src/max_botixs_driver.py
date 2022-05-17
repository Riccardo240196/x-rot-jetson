#!/usr/bin/python
# $Id: maxbotix.py 1563 2016-10-25 15:11:37Z mwall $
# Copyright 2015 Matthew Wall

# From the maxbotix datasheet:
#
# The MB736X sensors have an RS232 data format (with 0V to Vcc levels) and the
# MB738X sensors have a TTL outputs. The output is an ASCII capital R,
# followed by four ASCII character digits representing the range in
# millimeters, followed by a carriage return (ASCII 13). The maximum range
# reported is 4999 mm (5-meter models) or 9998 mm (10-meter models). A range
# value of 5000 or 9999 corresponds to no target being detected in the field
# of view.
#
# The serial data format is 9600 baud, 8 data bits, no parity, with one stop
# bit (9600-8-N-1).
#
# How to test:
#
# First ensure that the device is attached.  Use screen or other serial
# utility to talk directly with the sensor.  For example, if the sensor is
# connected with a USB-serial converter:
#
#   sudo screen /dev/ttyUSB0
#
# You should see the sensor output, such as R2257.
#
# Next, verify that the weewx driver can talk to the sensor.  Run the driver
# directly, for example:
#
#   cd /home/weewx
#   sudo PYTHONPATH=bin python bin/user/maxbotix.py --port /dev/ttyUSB0
#
# Finally, verify that the driver works in a full weewx configuration.

import serial
import syslog
import time

import weewx.drivers
import weewx.engine
import weewx.units

DRIVER_NAME = "Maxbotix"
DRIVER_VERSION = "0.5"
DEFAULT_MODEL = 'MB7363'

def logmsg(dst, msg):
    syslog.syslog(dst, 'maxbotix: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

def loader(config_dict, engine):
    return MaxbotixDriver(**config_dict['Maxbotix'])


schema = [('dateTime',  'INTEGER NOT NULL UNIQUE PRIMARY KEY'),
          ('usUnits',   'INTEGER NOT NULL'),
          ('interval',  'INTEGER NOT NULL'),
          ('range',     'REAL')]

weewx.units.obs_group_dict['range'] = 'group_range'
weewx.units.obs_group_dict['range2'] = 'group_range'
weewx.units.obs_group_dict['range3'] = 'group_range'
weewx.units.USUnits['group_range'] = 'inch'
weewx.units.MetricUnits['group_range'] = 'cm'
weewx.units.MetricWXUnits['group_range'] = 'cm'



class MaxbotixDriver(weewx.drivers.AbstractDevice):

    def __init__(self, **stn_dict):
        loginf("driver version is %s" % DRIVER_VERSION)
        self.max_tries = int(stn_dict.get('max_tries', 5))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.poll_interval = float(stn_dict.get('poll_interval', 1))
        loginf("polling interval is %s" % self.poll_interval)
        self.port = stn_dict.get('port', '/dev/ttyUSB0')
        loginf("port is %s" % self.port)
        self.model = stn_dict.get('model', DEFAULT_MODEL)
        loginf("model is %s" % self.model)

    @property
    def hardware_name(self):
        return "Maxbotix"

    def genLoopPackets(self):
        ntries = 0
        while ntries < self.max_tries:
            ntries += 1
            try:
                with Sensor(self.model, self.port) as sensor:
                    v = sensor.get_range()
                ntries = 0
                _packet = {'dateTime': int(time.time() + 0.5),
                           'usUnits': weewx.METRIC,
                           'range': v / 10.0 if v is not None else None}
                yield _packet
                if self.poll_interval:
                    time.sleep(self.poll_interval)
            except (serial.serialutil.SerialException, weewx.WeeWxIOError), e:
                logerr("Failed attempt %d of %d to get LOOP data: %s" %
                       (ntries, self.max_tries, e))
                time.sleep(self.retry_wait)
        else:
            msg = "Max retries (%d) exceeded for LOOP data" % self.max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)


class MaxbotixService(weewx.engine.StdService):

    def __init__(self, engine, config_dict):
        loginf("service version is %s" % DRIVER_VERSION)
        self.port = config_dict.get('port', '/dev/ttyUSB0')
        loginf("port is %s" % self.port)
        self.model = config_dict.get('model', DEFAULT_MODEL)

    def handle_new_loop(self, event):
        self.get_data(event.packet)

    def handle_new_archive(self, event):
        self.get_data(event.record)

    def get_data(self, data):
        v = None
        try:
            with Sensor(self.model, self.port) as sensor:
                v = sensor.get_range()
        except (serial.serialutil.SerialException, weewx.WeeWxIOError), e:
            logerr("Failed to get reading: %s" % e)
        if v is not None:
            v /= 10.0 # convert to cm
            if 'usUnits' in data and data['usUnits'] == weewx.US:
                v /= 2.54 # convert to inches
        data['range'] = v


class Sensor():

    # information about each type of sensor.  the key is the model number.  the
    # associated tuple contains the units of the value that is returned, the
    # value the sensor returns when the range is maxxed out, and the number or
    # characters (excluding the R and trailing newline) in the value string.
    MODEL_INFO = {
        'MB1040': ['inch', 254, 3], # 6in min; 254in max; 1in res
        # 5-meter sensors
        'MB7360': ['mm', 5000, 4],
        'MB7369': ['mm', 5000, 4],
        'MB7380': ['mm', 5000, 4],
        'MB7389': ['mm', 5000, 4],
        # 10-meter sensors
        'MB7363': ['mm', 9999, 4],
        'MB7366': ['mm', 9999, 4],
        'MB7383': ['mm', 9999, 4],
        'MB7386': ['mm', 9999, 4]
        }

    def __init__(self, model, port, baudrate=9600, timeout=1):
        self.model = model
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        model_info = Sensor.MODEL_INFO[self.model]
        self.units = model_info[0]
        self.no_target = model_info[1]
        self.data_length = model_info[2]

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

    def get_range(self):
        # return value is always mm
        line = self.serial_port.read(self.data_length + 2)
        if line:
            line = line.strip()
        if line and len(line) == self.data_length + 1 and line[0] == 'R':
            try:
                v = int(line[1:])
                if v == self.no_target:
                    logdbg("no target detected: v=%s" % v)
                    v = None
                if self.units == 'inch':
                    v *= 25.4
                return v
            except ValueError, e:
                raise weewx.WeeWxIOError("bogus value: %s" % e)
        else:
            raise weewx.WeeWxIOError("unexpected line: '%s'" % line)


# To test this driver, do the following:
#   PYTHONPATH=/home/weewx/bin python /home/weewx/bin/user/maxbotix.py
if __name__ == "__main__":
    usage = """%prog [options] [--help]"""

    def main():
        import optparse
        syslog.openlog('wee_maxbotix', syslog.LOG_PID | syslog.LOG_CONS)
        parser = optparse.OptionParser(usage=usage)
        parser.add_option('--port', dest="port", metavar="PORT",
                          default='/dev/ttyUSB0',
                          help="The port to use. Default is '/dev/ttyUSB0'.")
        parser.add_option('--model', dest="model", metavar="MODEL",
                          default=DEFAULT_MODEL,
                          help="The sensor model. Default is %s" % DEFAULT_MODEL)
        parser.add_option('--test-sensor', dest='tc', action='store_true',
                          help='test the sensor')
        parser.add_option('--test-driver', dest='td', action='store_true',
                          help='test the driver')
        parser.add_option('--test-service', dest='ts', action='store_true',
                          help='test the service')
        (options, args) = parser.parse_args()

        if options.tc:
            test_sensor(options.model, options.port)
        elif options.td:
            test_driver()
        elif options.ts:
            test_service(options.model, options.port)

    def test_driver():
        import weeutil.weeutil
        driver = MaxbotixDriver()
        print "range is cm"
        for pkt in driver.genLoopPackets():
            print weeutil.weeutil.timestamp_to_string(pkt['dateTime']), pkt

    def test_service(model, port):
        import sys
        config = {
            'Station': {
                'station_type': 'Simulator',
                'altitude': [0, 'foot'],
                'latitude': 0,
                'longitude': 0},
            'Simulator': {
                'driver': 'weewx.drivers.simulator',
                'mode': 'simulator'},
            'Maxbotix': {
                'model': model,
                'port': port},
            'Engine': {
                'Services': {
                    'archive_services': 'user.maxbotix.MaxbotixService'}}}
        engine = weewx.engine.StdEngine(config)
        svc = MaxbotixService(engine, config)
        print "range is inches"
        while True:
            data = {'usUnits': weewx.US}
            svc.get_data(data)
            sys.stdout.write("\r%s" % data)
            sys.stdout.flush()
            time.sleep(1)

    def test_sensor(model, port):
        import sys
        print "range is mm"
        with Sensor(model, port) as sensor:
            while True:
                sys.stdout.write("\r%s" % sensor.get_range())
                sys.stdout.flush()
                time.sleep(1)

    main()