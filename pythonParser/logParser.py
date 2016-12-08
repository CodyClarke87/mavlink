from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from Tkinter import Tk
from tkFileDialog import askopenfilename
import struct, sys, inspect


# Try import easygui, if not then try automatically installing it using pip.
try:
    import easygui as eg
except ImportError:
    print 'Installing easygui...'
    import pip
    try:
        pip.main(['install', 'easygui'])
    except:
        print 'Could not install easygui using pip. Please install manually.'

#import os

class FormatMessage(object):
    """A c-struct-like class for storing the fields of a format message.
    
    Instance variables:
    type: log message types from the Ardupilot DataFlash module such as 'FMT' or 'Param'. Objects of this class should only have type 'FMT'.
    id: unique id of the message type as defined in Adrupilot DataFlash. Is associated with the unique name of the message.
    size: Size of the message type in bytes. All messages of a type are a fixed size.
    name: Unique name of the message. Human readable identifier of the message if a little cryptic due to space saving.
    format: short form indicating the order and c-type of the .BIN message columns as shown by:
    b   : int8_t
    B   : uint8_t
    h   : int16_t
    H   : uint16_t
    i   : int32_t
    I   : uint32_t
    f   : float
    d   : double
    n   : char[4]
    N   : char[16]
    Z   : char[64]
    c   : int16_t * 100
    C   : uint16_t * 100
    e   : int32_t * 100
    E   : uint32_t * 100
    L   : int32_t latitude/longitude
    M   : uint8_t flight mode
    q   : int64_t
    Q   : uint64_t
    columns: The human readable names that the values of each message column represents
    """
    
    def __init__(self, format_line):
        """Create an instance of the FormatMessage class from the fixed format line.
        
        format_line is an input string beginning with 'FMT' and containing at least 6 comma separated values, representing the object's
        instance variables as defined by the class.
        """        
        format_line_list = format_line.split(', ', 5)
        msg_type, msg_id, msg_size, msg_name, msg_format, msg_columns = format_line_list        
        self.type = msg_type
        self.id = msg_id
        self.size = msg_size
        self.name = msg_name
        self.format = msg_format
        self.columns = msg_columns
        self.num_data_elements = len(self.format)        

    def __repr__(self):
        """Represent the instance variables of the object as a formatted string list."""
        return 'Format message: [type: %s, id: %s, size: %s, name: %s, format: %s, columns: %s]' % (self.type, self.id, self.size, self.name, self.format, self.columns)

    def __str__(self):
        """Convert the class to a comma and space separated list of strings.
        
        This will be identical to the string that was used to create the object.
        """
        return '%s, %s, %s, %s, %s, %s' % (self.type, self.id, self.size, self.name, self.format, self.columns)

class MessageType(object):
    """Contains the name, time and data for any message type.
    
    The name and time are extacted from the first two elements of the line. The rest of the
    values in the line are assumed to be the data that the line is referring to. This data
    is stored as a list. The idea is that an instance of this class can have its time and/or
    data changed after each new message. comes in. Time is updated on every new desired
    message. If the name matches the received desired message then the data list is iterated
    over and changed.
    """
        
    def __init__(self, line):
        """Create an instance of a generic message type with a name, time and list of data.
        
        name is simply the name of the message type as a string literal e.g. 'IMU'. time is
        the time in microseconds as standard in the message formats. This should be the first
        of the comma separated values present in rest_of_line. data_list is the remainder 
        of the rest_of_line comma separated values and varies in size from message to message.
        It is stored simply as a list so that it can be iterated over and changed each time 
        the data needs to be updated.
        """
        name, time, data = line.split(', ', 2)
        data_list = data.split(', ')
        for i, datum in enumerate(data_list):
            data_list[i] = datum
        self.name = name
        self.time = time
        self.data = data_list        
        
    def __repr__(self):
        """Represent the instance variables of the object as a formatted string list."""
        return 'Message type: [name: %s, time: %s, data: %s]' % (self.name, self.time, self.data)
        
    def __str__(self):
        """Represent the MessageType object simply as a CSV format string for sanity checks."""
        return self.name + ', ' + self.time + ', ' + str(self.data)

    def update_message_times(self, current_message_types):
        """Change the time value to be the latest for every message in current_message_types."""
        for message_type in current_message_types:
            current_message_types[message_type].time = self.time
        return current_message_types

    def update_message_values(self, current_message_types):
        """Change the data in the current_message_types corrensponding to this message. If it's not there then add it in."""
        try:
            current_message_types[self.name].data = self.data
        except KeyError:
            current_message_types[self.name] = self
        current_message_types = self.update_message_times(current_message_types)
        return current_message_types

def get_all_message_formats_dataflash(infile):
    """Creates and returns a dictionary of format messages from the first lines of the input file.
    
    The input file should be a .bin.log file from the output of MissionPlanner's log converter.
    The first lines of code in the file will be all of the format messages, each beginning with
    'FMT' and conataining six comma separated fields identifying that message format. The messages
    are stored in a dictionary with the key being the message name e.g. 'RCIN', 'IMU' and the
    value being the FormatMessage object representing the format line.
    """
    message_formats = {}
    
    end_of_format_lines = infile.tell() #Trace to bring file pointer back to start of non-format messages once we've read one
    line = infile.readline()

    while line.strip().startswith('FMT'):
        format_message = FormatMessage(line)
        message_formats[format_message.name] = format_message
        
        end_of_format_lines = infile.tell() #Trace to bring file pointer back to start of non-format messages once we've read one
        line = infile.readline()
        
    infile.seek(end_of_format_lines)
    return message_formats    

def get_all_message_formats_telemetry_log(desired_messages):
    # Hacky code that takes the desired message and gets the corresponding class definition in ardupilotmega.py and extracts that message class's attributes. Cart before the horse type stuff. Sorry.
    message_formats = {}
    for message_name in desired_messages:
        class_string = 'mavlink2.MAVLink_' + message_name.lower() + '_message'
        class_reference = reduce(getattr, class_string.split('.'), sys.modules[__name__]) # gets a reference to the class from a string
        msg_type = 'FMT'
        msg_id = str(class_reference.id)
        msg_size = str(0) # We could figure this out. We can extract the format, eg fff corresponding to 3 floats then add the length in bytes (chars) of NAME. Don't need it for this code though so we'll default to zero
        msg_name = message_name
        msg_format = class_reference.format # This has form '<fff' whereas in dataflash has form 'Qfff'. The first char is removed in the write output function so we leave the '<' here as it will be removed anyway. 
        msg_columns = ','.join(class_reference.ordered_fieldnames)
        format_list = [msg_type, msg_id, msg_size, msg_name, msg_format, msg_columns]
        format_line = ', '.join(format_list)
        format_message = FormatMessage(format_line)
        message_formats[format_message.name] = format_message
    return message_formats

def get_current_data(desired_message, current_message_types, message_formats):
    try:
        data = current_message_types[desired_message].data
    except KeyError:
        num_empty_data_columns = message_formats[desired_message].num_data_elements - 1 #-1 to ignore the time element
        data = ['']*num_empty_data_columns
    return data

def write_output_message_line(desired_messages, current_message_types, message_formats, outfile):
    """Write the desired messages to the output csv file in order with the time at the beginning."""
    time = current_message_types[current_message_types.keys()[0]].time # get the time of any item in the dictionary, they're all the same
    output_string_list = [time]
    for desired_message in desired_messages:
        data = get_current_data(desired_message, current_message_types, message_formats)
        data_string = ', '.join(data)
        output_string_list.append(data_string)
    output_string = ', '.join(output_string_list) + '\n'
    outfile.write(output_string)

def get_message_buffer(infile, mav):
    buff = b''
    time = bytearray(infile.read(8))
    start = bytearray(infile.read(1))
    if time == '': # EOF, return empty buffers
        return time, buff
    buff += start
    length = infile.read(1)
    array_length = bytearray(length)
    length = int(ord(length))
    buff += array_length
    protocol_id = format(buff[0], '02X') # Convert to hex string
    if protocol_id == 'FE':
        buff += bytearray(infile.read(length + 6)) # Mavlink protocol V1.0
    elif protocol_id == 'FD':
        buff += bytearray(infile.read(length + 23)) # Mavlink protocol V2.0
    return time, buff

def get_list_of_desired_messages(list_of_message_format_names):
    question = 'Please select the data to convert.'
    title = 'Convert .bin.log or .tlog to .csv.'
    choice = eg.multchoicebox(question, title, list_of_message_format_names)
    return choice

def get_desired_messages_dataflash(message_formats):
    list_of_message_format_names = sorted(message_formats.keys())
    return get_list_of_desired_messages(list_of_message_format_names)

def get_desired_messages_telemetrylog():
    prefix = 'MAVLINK_MSG_ID_'
    mavlink_globals_list = dir(mavlink2)
    available_messages = []
    for global_object in mavlink_globals_list:
        if global_object.startswith(prefix):
            available_messages.append(global_object[len(prefix):])
    return get_list_of_desired_messages(available_messages)
    

def process_dataflash_file(input_file, output_file):
    with open(input_file) as infile, open(output_file,'w') as outfile:
        message_formats = get_all_message_formats_dataflash(infile)
        #desired_messages = ['VIBE']
        desired_messages = get_desired_messages_dataflash(message_formats)
        if desired_messages == None:
            return False # User cancelled
        current_message_types = {}
        line = infile.readline().strip()
        while line != "":
            message = MessageType(line)
            if message.name in desired_messages:
                current_message_types = message.update_message_values(current_message_types)
                write_output_message_line(desired_messages, current_message_types, message_formats, outfile)                
            line = infile.readline().strip()
        return True

def process_telemetrylog_file(input_file, output_file):
    desired_messages = get_desired_messages_telemetrylog()
    #desired_messages = ['WIND', 'AHRS']
    with open(input_file, 'rb') as infile, open(output_file, 'w') as outfile:
        current_message_types = {}
        message_formats = get_all_message_formats_telemetry_log(desired_messages)
        mav = mavlink2.MAVLink(infile)
        time, buff = get_message_buffer(infile, mav)
        while buff != '':
            time = str(int(struct.unpack('>Q', time)[0]))
            mav_message = mav.decode(buff)
            values = []
            for field_name in mav_message.fieldnames:
                value = getattr(mav_message, field_name)
                values.append(str(value))
            value_string = ','.join(values)
            line = mav_message.name + ', ' + time + ', ' + value_string
            message = MessageType(line)
            if message.name in desired_messages:
                current_message_types = message.update_message_values(current_message_types)
                write_output_message_line(desired_messages, current_message_types, message_formats, outfile)
            time, buff = get_message_buffer(infile, mav)
        return True

def get_input_file():
    Tk().withdraw()
    input_file_name = askopenfilename(title='Select a log file.', filetypes=(('All logs', '*.tlog;*.bin.log'), ('DataFlash logs', '*.bin.log'), ('Telemetry logs', '*.tlog') ))
    return input_file_name

def main():
    input_file = get_input_file()
    if input_file == '':
        return
    output_file = input_file + '.csv'
    if input_file.endswith('.bin.log'):
        success = process_dataflash_file(input_file, output_file)
        if success:
            print 'Output in ' + output_file
    elif input_file.endswith('.tlog'):
        success = process_telemetrylog_file(input_file, output_file)
        if success:
            print 'Output in ' + output_file
    
main()    
