#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#                            All Rights Reserved.
#
#  This file is subject to the terms and conditions defined in
#  file 'LICENSE.md', which is part of this source code package.
#
#  NOTICE:  All information contained herein is, and remains the property of Klepsydra
#  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
#  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
#  trade secret or copyright law. Dissemination of this information or reproduction of
#  this material is strictly forbidden unless prior written permission is obtained from
#  Klepsydra Technologies GmbH.
#
#****************************************************************************

import getopt
import sys
import os
import argparse

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './generator')))

from generator_engine import Generator

DISABLE_ROS_OPTION = "disable_ros"
DISABLE_ZMQ_OPTION = "disable_zmq"
DISABLE_DDS_OPTION = "disable_dds"
INCLUDE_PATH_OPTION = "include_path"
OUTPUT_DIR_OPTION = "odir"
INPUT_DIR_OPTION = "idir"
CONF_PATH_OPTION = "conf_path"
TEMPLATE_PATH_OPTION = "template_path"

DISABLE_ROS_SHORT_OPT = "r"
DISABLE_ZMQ_SHORT_OPT = "z"
DISABLE_DDS_SHORT_OPT = "d"
INCLUDE_PATH_SHORT_OPT = "p"
OUTPUT_DIR_SHORT_OPT = "o"
INPUT_DIR_SHORT_OPT = "i"
CONF_PATH_SHORT_OPT = "c"
TEMPLATE_PATH_SHORT_OPT = "t"

def main(argv = None):
    """The main routine.

    This function accepts arguments for using the getopt python
    module. Arguments are provided by using the long or short (shown
    in parenthesis below) option name followed by its value. Provide
    the following arguments:

    idir (i) : Input folder name.
    odir (o) : Output folder name.
    include_path (p) : The prefix to the include_path for generated files.
    disable_zmq (z) : Disable ZMQ functionality (True or False)
    disable_dds (d) : Disable DDS functionality (True or False)
    disable_ros (r) : Disable ROS functionality (True or False)
    conf_path (c) : The configuration path. Currently defaults to 'conf' folder
    template_path (t) : The template path. Currently defaults to 'templates' folder

    """
    if argv is None:
        argv = sys.argv[1:]

    input_directory = ''
    output_directory = ''
    include_path = ''
    disable_dds = False
    disable_zmq = False
    disable_ros = False
    conf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'conf'))
    template_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'templates'))

    try:
        opts, args = getopt.getopt(argv, "h%s:%s:%s:%s:%s:%s:%s:%s:" % (INPUT_DIR_SHORT_OPT, OUTPUT_DIR_SHORT_OPT,
                                                                        INCLUDE_PATH_SHORT_OPT, DISABLE_DDS_SHORT_OPT,
                                                                        DISABLE_ZMQ_SHORT_OPT, DISABLE_ROS_SHORT_OPT,
                                                                        CONF_PATH_SHORT_OPT, TEMPLATE_PATH_SHORT_OPT),
                                   [("%s=" % INPUT_DIR_OPTION), ("%s=" % OUTPUT_DIR_OPTION),
                                    ("%s=" % INCLUDE_PATH_OPTION), ("%s=" % DISABLE_DDS_OPTION), ("%s=" % DISABLE_ZMQ_OPTION),
                                    ("%s=" % DISABLE_ROS_OPTION), ("%s=" % CONF_PATH_OPTION), ("%s=" % TEMPLATE_PATH_OPTION)])
    except getopt.GetoptError:
        print_help()
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print_help()
            sys.exit()
        elif opt in (("-%s" % INPUT_DIR_SHORT_OPT), ("--%s" % INPUT_DIR_OPTION)):
            input_directory = add_slash_at_end(arg)
        elif opt in (("-%s" % OUTPUT_DIR_SHORT_OPT), ("--%s" % OUTPUT_DIR_OPTION)):
            output_directory = add_slash_at_end(arg)
        elif opt in (("-%s" % INCLUDE_PATH_SHORT_OPT), ("--%s" % INCLUDE_PATH_OPTION)):
            include_path = add_slash_at_end(arg)
        elif opt in (("-%s" % DISABLE_DDS_SHORT_OPT), ("--%s" % DISABLE_DDS_OPTION)):
            disable_dds = arg.lower() == 'true'
        elif opt in (("-%s" % DISABLE_ZMQ_SHORT_OPT), ("--%s" % DISABLE_ZMQ_OPTION)):
            disable_zmq = arg.lower() == 'true'
        elif opt in (("-%s" % DISABLE_ROS_SHORT_OPT), ("--%s" % DISABLE_ROS_OPTION)):
            disable_ros = arg.lower() == 'true'
        elif opt in (("-%s" % CONF_PATH_SHORT_OPT), ("--%s" % CONF_PATH_OPTION)):
            conf_path = add_slash_at_end(arg)
        elif opt in (("-%s" % TEMPLATE_PATH_SHORT_OPT), ("--%s" % TEMPLATE_PATH_OPTION)):
            template_path = add_slash_at_end(arg)

    print ('Input directory is "', input_directory)
    print ('Output directory is "', output_directory)
    print ('Include path is "', include_path)
    print ('Disable DDS is "', disable_dds)
    print ('Disable ZMQ is "', disable_zmq)
    print ('Disable ROS is "', disable_ros)

    generator = Generator(conf_path, template_path)
    generator.render(input_directory, output_directory, include_path, disable_ros, disable_dds, disable_zmq)


def add_slash_at_end(arg):
    if arg.endswith("/"):
        return arg
    else:
        return arg + "/"


def print_help():
    print ('integration_test.py [options]: ')
    print ("   Input directory: -%s | --%s" % (INPUT_DIR_SHORT_OPT, INPUT_DIR_OPTION))
    print ("   Output directory: -%s | --%s" % (OUTPUT_DIR_SHORT_OPT, OUTPUT_DIR_OPTION))
    print ("   Include path: -%s | --%s" % (INCLUDE_PATH_SHORT_OPT, INCLUDE_PATH_OPTION))
    print ("   Disable DDS: -%s | --%s" % (DISABLE_DDS_SHORT_OPT, DISABLE_DDS_OPTION))
    print ("   Disable ZMQ: -%s | --%s" % (DISABLE_ZMQ_SHORT_OPT, DISABLE_ZMQ_OPTION))
    print ("   Disable ROS: -%s | --%s" % (DISABLE_ROS_SHORT_OPT, DISABLE_ROS_OPTION))
    print ("   Configuration path: -%s | --%s" % (CONF_PATH_SHORT_OPT, CONF_PATH_OPTION))
    print ("   Template path: -%s | --%s" % (TEMPLATE_PATH_SHORT_OPT, TEMPLATE_PATH_OPTION))


if __name__ == "__main__":
    main(sys.argv[1:])
