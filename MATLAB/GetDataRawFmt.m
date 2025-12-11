function [data]=GetDataRawFmt(device,cmd, datatype)

device.ByteOrder = 'little-endian';
device.Timeout = 4;    

write(device,cmd,"uint8");
datalength_byte = read(device,1,"uint16");

if (datatype=="uint8") || (datatype=="int8") || (datatype=="byte")
  datatoread =   round(datalength_byte /2 );
end

if (datatype=="uint16") || (datatype=="int16") || (datatype=="word")
  datatoread =   round(datalength_byte /2 );
end

if (datatype=="float") || (datatype=="single") 
  datatoread =   round(datalength_byte /4 );
end


data =   read(device,datatoread ,datatype)' ;


