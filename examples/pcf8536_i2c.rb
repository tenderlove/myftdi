require "myftdi"
require "myftdi/i2c"

def bcd2dec val
  ((val >> 4) * 10) + (val & 0xF)
end

dev = MyFTDI.ft232h_i2c
reader = dev.get_port 0x51

loop do
  bytes = reader.read_from(0, 6).bytes
  p READ: bytes
  next if bytes.length != 6
  _, _, seconds, minutes, hours, days = bytes
  #second = reader.read_from(2, 1).bytes.first
  p TIME: [
    bcd2dec(days & 0x3F),
    bcd2dec(hours & 0x3F),
    bcd2dec(minutes & 0x7F),
    bcd2dec(seconds & 0x7F)]
  #first & 0x7F
  #p byte.to_s(2) => bcd2dec(bytes.)
  sleep 1
end
