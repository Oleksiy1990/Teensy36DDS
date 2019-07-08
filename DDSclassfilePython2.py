import numpy as np
#import matplotlib.pyplot as plt
#import sys
import serial
import time
import sys

# directory = "C:\\Users\\Oleksiy\\Documents\\HeIntOleksiy\\"
# file = "currentscript"
freq_mult_factor = 100*1000000 # (100 is due to aconfig conversion factor) this is just because the frequency will be given in MHz
power_mult_factor = 100 #due to aconfig that only goes to 5 volt.
# freq_channel = 5
# power_channel = 12
serial_timeout_val = 3



# path = directory + file
#plt.plot(np.linspace(1,20))
#plt.show()



class DDScomm:
    def __init__(self,filepath,comport,baud,freq_output_channel=None,power_output_channel=None,default_freq = 80000000, default_power = 10):

        #print "initiating class DDScomm"
        self.my_path = filepath
        self.my_com = comport
        self.my_baud = baud
        self.my_ao_freq = freq_output_channel
        self.my_ao_power = power_output_channel

        split_lines = self.get_split_lines()
        self.my_split_lines = split_lines
        self.my_serial_timeout = serial_timeout_val

        self._my_sPort = None
        self._handshake_done = False
        self._sequence_freq = None
        self._sequence_power = None
        self._my_freq_toset = 0
        self._my_power_toset = 0
        self._my_setvalue_line = None # this is the line that holds the set values. it's the line that does not have the time stamp in currentscript

    def get_split_lines(self):
        """
        This will read the file and return a list of lists containing the lines
        with each line split into individual words based on spaces.
        Note that the lines with are not time-stamped sequence steps but rather
        contain @ sign or # or something like that at the beginning will be
        ignored.
        The file should be "currentscript"
        """
        file_copy = open(self.my_path, "r")
        lines = file_copy.readlines() # read line by line, get a list of lines
        file_copy.close()
        lines = [line.strip() for line in lines] # get rid of newlines and such
        linesplit = [line.split() for line in lines] # split into individual elements
        for l in linesplit:
            if l[0] == "ao0":
                self._my_setvalue_line = l
                break
        for n in reversed(range(len(linesplit))): # check out if the first entry is a number, which means that it's a valud time step
            # reversed is used because I want to count from the back, because the lines are being deleted. If we counted from the front
            # the indices would be changing as we are deleting the lines, which would be a mess
            try:
                float(linesplit[n][0])
            except:
                del linesplit[n]
        del linesplit[-1] # This is because Saneface writes the "set" line as the last line, but it doesn't carry the trigger, so it's better to delete it and let it sit at the last value
        return linesplit

    def get_setvalue(self,ao_channel):
        self.get_split_lines()
        line_input = self._my_setvalue_line
        column_ao = 2*(ao_channel)+1
        value_str = line_input[column_ao]
        return np.array(value_str,dtype="float")


    def get_simple_steps(self,ao_channel,ramps_numbers):
        """
        start with a list of all lines from the file, where the commented out steps have been removed
        find simple changes in the given ao value and return them together with their row number in the lines_split list
        """
        lines_input = self.my_split_lines
        column_ao = 2*(ao_channel+1)
        vals_str = [lines_input[n][column_ao] for n in range(len(lines_input))]
        # This line above simply takes the column of values that corresponds to
        # the analog output of interest
        steps_positions = [0]
        steps_vals = [vals_str[0]]
        for q in range(1,len(vals_str)):
            if (vals_str[q] != vals_str[q-1]) or ((q-1) in ramps_numbers):
                # we just check if the input changed
                # notice that we compare strings in order to not get the problem of comparing floats
                # and string comparison should work perfectly well because we are starting
                # from a text file
                # also, if there was a ramp before, the next one should for sure be a trigger and a step
                if q in ramps_numbers: # This is to make sure that a ramp is NOT also double-counted as a simple step
                    continue
                steps_positions.append(q)
                steps_vals.append(vals_str[q])
        return (np.array(steps_positions,dtype = "int"), np.array(steps_vals,dtype = "float"))

    def get_ramps(self,ao_channel):
        """
        start with a list of all lines from the file, where the commented out steps have been removed
        make sure that this lines_split argument has an array of lines, where each individual word is a new element
        """
        ramps_step_num = [] # row number in lines_split where a ramp occurs
        ramps_content = [] # contents of that ramp
        for (counter,l) in enumerate(self.my_split_lines): #"l" is one given line of the my_split_lines
            for q in range(len(l)-1):
                if (l[q] == "ramp" and (l[q+1] == "ao%d"%ao_channel)):
                    # The line above shows how ramps are defined in the "currentscript"
                    # file, meaning that it says "ramp ao1" for exanple
                    # so we search them in that way too
                    ramps_step_num.append(counter)
                    ramps_content.append(l[q+2:q+9]) # each ramp has 7 entries
        # get_ramps returns a tuple of 2 arrays: the number of line in which a ramp occurs
        # and the contents (7 numbers) of that ramp
        return (np.array(ramps_step_num,dtype = "int"),np.array(ramps_content,dtype="float"))

    def assemble_sequence(self,ao_channel):
        """
        This assembles the sequence to be sent to the DDS.
        The position in the array determines the time step along the original sequence
        The fact whether anything happens in that step is controlled by
        "simple", "ramp", or "void"
        """
        (ramps_step_num, ramp_data) = self.get_ramps(ao_channel)
        (simple_step_num, step_data) = self.get_simple_steps(ao_channel,ramps_step_num)
        #print(simple_step_num)
        #print(ramps_step_num)
        #print(len(simple_step_num))
        #print(len(ramps_step_num))
        if len(simple_step_num) == 0:
            full_sequence_length = np.nanmax(ramps_step_num)+1
        elif len(ramps_step_num) == 0:
            full_sequence_length = np.nanmax(simple_step_num)+1
        else:
            full_sequence_length = np.nanmax([np.nanmax(simple_step_num),np.nanmax(ramps_step_num)]) + 1
        #full_sequence_length = len(simple_step_num) + len(ramps_step_num)
        #steps_list = [0 for i in range(full_sequence_length)] # This holds the step number in which the sequence occurs
        sequence_list = [0 for i in range(full_sequence_length)] # This holds the actual content of the sequence
        type_list = ["void" for i in range(full_sequence_length)] # this holds array type, "simple", "ramp", or "void"

        for (index,val) in enumerate(simple_step_num):
            sequence_list[val] = step_data[index]
            type_list[val] = "simple"
        for (index,val) in enumerate(ramps_step_num): # Important that the ramps are second! Because essentially every ramp will also be detected as a "change", so a simple step too
            sequence_list[val] = ramp_data[index]
            type_list[val] = "ramp"
        return (type_list,sequence_list)
        # not good for now because it can never be 0

    def _open_serial(self):
        self._my_sPort = serial.Serial(self.my_com,self.my_baud, timeout=self.my_serial_timeout)
        if not self._my_sPort.isOpen():
            print "Cannot open "+self.my_com
            return False
        return True

    def do_handshake(self):
        if self._my_sPort is None:
            isPortOpen = self._open_serial()
        elif self._my_sPort.isOpen():
            isPortOpen = True
        else:
            isPortOpen = self._open_serial()

        if not isPortOpen:
            print "Cannot open COM port"
            return False
        self._my_sPort.write('h'.encode("ascii"))
        incoming_msg = self._my_sPort.read().decode("ascii")
        if incoming_msg == 'y':
            #print "Handshake done"
            self._handshake_done = True
            return True
        elif incoming_msg == 'n':
            print "Got an n from Arduino handshake routine"
            return False
        else:
            print "do_handshake communication failed"
            return False


    def set_frequency(self):
        if not self._handshake_done:
            handshake = self.do_handshake()
            #print str(handshake) + " handshake in set_frequency"
        if not handshake:
            print "Handshake failed in set_frequency"
            self._handshake_done = False
            return False
        self._handshake_done = False
        command = "f" + "{:d}".format(int(self._my_freq_toset))
        #print "frequeny command " +command
        self._my_sPort.write(command.encode("ascii"))
        incoming_msg = self._my_sPort.read().decode("ascii")
        if incoming_msg == 'c':
            print "Frequency set to {:.04f}".format(self._my_freq_toset/1000000.) + " MHz on port {:s}".format(self.my_com)
            return True
        elif incoming_msg == 'n':
            print "Got an n from Arduino in set_frequency"
            return False
        elif incoming_msg == 'f':
            print "Frequency out of range (message from set_frequency function)"
        else:
            print "set_frequency communication failed"
            return False

    def set_power(self):
        if not self._handshake_done:
            handshake = self.do_handshake()

        if not handshake:
            print "Handshake failed in set_power"
            self._handshake_done = False
            return False
        self._handshake_done = False
        command = 'p' + "{:.04f}".format(self._my_power_toset)
        #print "power command "+command
        self._my_sPort.write(command.encode("ascii"))
        incoming_msg = self._my_sPort.read().decode("ascii")
        if incoming_msg == 'c':
            print "Power set to {:.04f}".format(self._my_power_toset) + " percent on {:s}".format(self.my_com)
            return True
        elif incoming_msg == 'n':
            print "Got an n from Arduino in set_power"
            return False
        else:
            print "set_power communication failed"
            return False

    def transmit_setvalues(self):
        self._my_freq_toset = self.get_setvalue(self.my_ao_freq)*freq_mult_factor
        self._my_power_toset = self.get_setvalue(self.my_ao_power)*power_mult_factor
        result_f = self.set_frequency()
        result_p = self.set_power()
        if result_f and result_p:
            self._my_sPort.close()
            return True
        else:
            return False


    def send_sequence(self):
        (freq_type,freq_content) = self.assemble_sequence(self.my_ao_freq)
        (power_type,power_content) = self.assemble_sequence(self.my_ao_power)

        # These 2 for-loops will use the range function in a smart way and
        # check if one array is longer than the other, and if so, simply
        # attach an appropriate number of "void" and zeros to the other
        # to make them equally long
        for idx in range(len(freq_type),len(power_type)):
            freq_type.append("void")
            freq_content.append(0)
        for idx in range(len(power_type),len(freq_type)):
            power_type.append("void")
            power_content.append(0)

        for idx in range(len(freq_type)):
            if (freq_type[idx] == "ramp" and power_type[idx] == "ramp"):
                print "Two ramps cannot be send in the same step"
                return False

        # Let's collect the indices where something happens (so some change)
        # Basically we will delete all cases of "void" "void" (in both frequency and power)
        sequence_step_indices = [n for n in range(len(freq_type)) if (freq_type[n] != "void") or (power_type[n] != "void")]
        num_sequence_steps = len(sequence_step_indices)

        # Do the handshake
        if not self._handshake_done:
            handshake = self.do_handshake()
        if not handshake:
            print "First handshake in send_sequence failed"
            self._handshake_done = False
            return False

        # first we tell how many sequence steps there are
        command = "s" + "{:d}".format(num_sequence_steps)
        print "steps "+ command
        self._my_sPort.write(command.encode("ascii"))
        incoming_msg = self._my_sPort.read().decode("ascii")
        if incoming_msg == 'c':
            #print "Send sequence length done"
            pass;
        elif incoming_msg == 'n':
            print "Got an n from Arduino in send_sequence_length"
            return False
        elif incoming_msg == 'x':
            print "Wrong symbol sent to define sequence type!"
            return False
        else:
            print "send_sequence_length communication failed"
            return False


        for idx in sequence_step_indices:
            instruction_statement = ""
            # First fill the first two positions
            if freq_type[idx] == "simple":
                instruction_statement = instruction_statement+"f"
                data_statement_freq = "{:d}\n".format(int(freq_content[idx]*freq_mult_factor)) #type-cast to integer for simplicity and because we cannot send sub-Hz
            else:
                instruction_statement = instruction_statement + "v"
                data_statement_freq = None
            if power_type[idx] == "simple":
                instruction_statement = instruction_statement + "p"
                data_statement_power = "{:.04f}\n".format(power_content[idx]*power_mult_factor)
            else:
                instruction_statement = instruction_statement + "v"
                data_statement_power = None

            if freq_type[idx] == "ramp":
                instruction_statement = instruction_statement+"w"
                data_statement_ramp_freq = []
                if round(freq_content[idx][3]) == 0:
                    data_statement_ramp_freq.append("0\n")
                elif round(freq_content[idx][3]) == 1:
                    data_statement_ramp_freq.append("1\n")
                else:
                    print "Wrong type sent in ramp type lin or exp"
                    return False
                data_statement_ramp_freq.append("{:d}\n".format(int(freq_content[idx][5]*freq_mult_factor)))
                data_statement_ramp_freq.append("{:d}\n".format(int(freq_content[idx][6]*freq_mult_factor)))
                data_statement_ramp_freq.append("{:02f}\n".format(freq_content[idx][1]*1000)) # multiplication by 1000 because in the file it's in s but in communication it must be in ms
                data_statement_ramp_freq.append("{:02f}\n".format(freq_content[idx][4]*1000)) # multiplication by 1000 because in the file it's in s but in communication it must be in ms
            else:
                instruction_statement = instruction_statement + "v"
                data_statement_ramp_freq = None

            if power_type[idx] == "ramp":
                instruction_statement = instruction_statement+"w"
                data_statement_ramp_power = []
                if round(power_content[idx][3]) == 0:
                    data_statement_ramp_power.append("0\n")
                elif round(power_content[idx][3]) == 1:
                    data_statement_ramp_power.append("1\n")
                else:
                    print "Wrong type sent in ramp type lin or exp"
                    return False
                data_statement_ramp_power.append("{:d}\n".format(power_content[idx][5]*power_mult_factor))
                data_statement_ramp_power.append("{:d}\n".format(power_content[idx][6]*power_mult_factor))
                data_statement_ramp_power.append("{:02f}\n".format(power_content[idx][1]*1000)) # multiplication by 1000 because in the file it's in s but in communication it must be in ms
                data_statement_ramp_power.append("{:02f}\n".format(power_content[idx][4]*1000)) # multiplication by 1000 because in the file it's in s but in communication it must be in ms
            else:
                instruction_statement = instruction_statement + "v"
                data_statement_ramp_power = None

            possible_instruction_statements = ["fpvv", "fvvv", "vpvv", "fvvw", "vpwv", "vvwv", "vvvw"]
            if instruction_statement in possible_instruction_statements:
                print instruction_statement # This is only for debugging, this has to be deleted
            else:
                print "Invalid instruction statement "+instruction_statement
                return False

            if (data_statement_freq is not None) and (data_statement_ramp_freq is not None):
                print "Trying to send frequency and ramp frequency. Undefined. Exiting"
                return False
            if (data_statement_power is not None) and (data_statement_ramp_power is not None):
                print "Trying to send power and ramp power. Undefined. Exiting"
                return False

            # first we will send the command to the Arduino, so what type of data is coming
            self._my_sPort.write(instruction_statement.encode("ascii"))
            incoming_msg = self._my_sPort.read().decode("ascii")
            if incoming_msg == 'r': # 'r' -> "received"
                #print "Instruction statement successful, idx {:d}".format(idx))
                pass
            elif incoming_msg == 'n':
                print "Got an n from Arduino in sending instruction_statement"
                return False
            else:
                print "instruction_statement communication failed"
                return False

            # Now we will send frequency data if it's a single-frequency point (not ramp)
            #print(data_statement_freq)
            if data_statement_freq is not None:
                #print data_statement_freq
                self._my_sPort.write(data_statement_freq.encode("ascii"))
                incoming_msg = self._my_sPort.read().decode("ascii")
                if incoming_msg == 'r':
                    #print "data_statement_freq sent successfully"
                    pass
                elif incoming_msg == 'n':
                    print "Got an n from Arduino in sending data_statement_freq"
                    return False
                else:
                    print "send_sequence communication failed on data_statement_freq"
                    return False

            # Now we will send power data if it is a single-power point (not ramp)
            #print(data_statement_power)
            if data_statement_power is not None:
                #print data_statement_power
                self._my_sPort.write(data_statement_power.encode("ascii"))
                incoming_msg = self._my_sPort.read().decode("ascii")
                if incoming_msg == 'r':
                    #print "data_statement_power sent successfully"
                    pass
                elif incoming_msg == 'n':
                    print "Got an n from Arduino in sending data_statement_power"
                    return False
                else:
                    print "send_sequence communication failed on data_statement_power"
                    return False

            if data_statement_ramp_freq is not None:
                for q in range(len(data_statement_ramp_freq)):
                    self._my_sPort.write(data_statement_ramp_freq[q].encode("ascii"))
                    incoming_msg = self._my_sPort.read().decode("ascii")
                    if incoming_msg == 'r':
                        #print "data_statement_power sent successfully"
                        pass
                    elif incoming_msg == 'n':
                        print "Got an n from Arduino in sending data_statement_ramp_freq, position {:d}".format(idx)
                        return False
                    else:
                        print "send_sequence communication failed on data_statement_ramp_freq, positon {:d}".format(idx)
                        return False

            if data_statement_ramp_power is not None:
                for q in range(len(data_statement_ramp_power)):
                    self._my_sPort.write(data_statement_ramp_power[q].encode("ascii"))
                    incoming_msg = self._my_sPort.read().decode("ascii")
                    if incoming_msg == 'r':
                        #print "data_statement_power sent successfully"
                        pass
                    elif incoming_msg == 'n':
                        print "Got an n from Arduino in sending data_statement_ramp_power, position {:d}".format(idx)
                        return False
                    else:
                        print "send_sequence communication failed on data_statement_ramp_power, positon {:d}".format(idx)
                        return False

        print "Full sequence sent port {:s}".format(self.my_com)
        self._my_sPort.close()
        return True

#a = DDScomm(directory+file,"COM8",57600,freq_output_channel = freq_channel, power_output_channel = power_channel)
#a.send_sequence()
#print(result)
#ramps = a.get_ramps()
#print(ramps)
#a.send_sequence()
