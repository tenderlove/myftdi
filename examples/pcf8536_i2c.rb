require "myftdi"
require "myftdi/i2c"

def bcd2dec val
  ((val >> 4) * 10) + (val & 0xF)
end

def dec2bcd val
  ((val / 10) << 4) | (val % 10)
end

def time2bcd time
  [
    time.sec,
    time.min,
    time.hour,
    time.day,
    time.wday,
    time.month,
    time.year % 2000
  ].map { |x| dec2bcd(x) }
end

def bcd2time bytes
  _, _, seconds, minutes, hours, days, weekdays, c_months, years = bytes
  Time.local bcd2dec(years) + 2000,
             bcd2dec(c_months & 0xF), # don't care about century flag
             bcd2dec(days & 0x3F),
             bcd2dec(hours & 0x3F),
             bcd2dec(minutes & 0x7F),
             bcd2dec(seconds & 0x7F)
end

dev = MyFTDI.ft232h_i2c
reader = dev.get_port 0x51

# Set the RTC to the current time
reader.write_to(0x2, time2bcd(Time.now).pack("C7"))

loop do
  bytes = reader.read_from(0, 9).bytes
  p bcd2time(bytes)
  sleep 1
end
