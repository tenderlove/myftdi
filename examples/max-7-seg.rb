# This example works with the MAX 7219 / 7221 7 segment LED driver
#
# With the Adafruit FT232H breakout -> MAX7221:
#   D0 -> CLK
#   D1 -> DIN
#   D3 -> LOAD
require "myftdi"

def clear_display port
  8.times { |i| port.write [i + 1, 0xF].pack("CC") }
end

def set_digits dev, num
  buf = ''.b
  8.times do |x|
    if num == 0
      dev.write([x + 1, 0xF].pack("CC"))
    else
      dig = num % 10
      dev.write([x + 1, (1 << 7) | dig].pack("CC"))
      num = num / 10
    end
  end
end

def set_date port, date
  port.write [1, date.day % 10].pack "CC"
  port.write [2, date.day / 10].pack "CC"
  port.write [3, (1 << 7) | (date.month % 10)].pack "CC"
  port.write [4, date.month / 10].pack "CC"
  port.write [5, (1 << 7) | date.year % 10].pack "CC"
  port.write [6, (date.year / 10) % 10].pack "CC"
  port.write [7, (date.year / 100) % 10].pack "CC"
  port.write [8, (date.year / 1000)].pack "CC"
end

dev = FTDI.ft232h_spi

port = dev.get_port 0, 0

# See the datasheet for commands:
#   https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
port.write "\x0C\x01".b # Turn on
port.write "\x09\xFF".b # Decode B on all digits
port.write "\x0A\x0F".b # Full intensity
port.write "\x0B\x07".b # Scan all digits
port.write "\x0F\x01".b # Display test
sleep(1)
port.write "\x0F\x00".b # Disable test

clear_display port

set_date port, Time.now

loop do
  num = rand(99999999)
  p num
  set_digits(port, num)
  sleep 0.5
end
