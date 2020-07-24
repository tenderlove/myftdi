# This example works with the MAX 7219 / 7221 7 segment LED driver
#
# With the Adafruit FT232H breakout -> MAX7221:
#   D0 -> CLK
#   D1 -> DIN
#   D3 -> LOAD
require "myftdi"

def clear_display port
  2.times { |i| port.write [i + 1, 0xF].pack("CC") }
end

def set_digits dev, num
  buf = ''.b
  2.times do |x|
    #if num == 0
    #  dev.write([x + 1, 0xF].pack("CC"))
    #else
      dig = num % 10
      dev.write([x + 1, (1 << 7) | dig].pack("CC"))
      num = num / 10
    #end
  end
end

dev = MyFTDI.ft232h_spi

port = dev.get_port 0, 0

# See the datasheet for commands:
#   https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
port.write "\x0C\x01".b # Turn on
#port.write "\x09\xFF".b # Decode B on all digits
port.write "\x09\x00".b # Decode B off all digits
port.write "\x0A\x0F".b # Full intensity
port.write "\x0B\x03".b # Scan all digits
port.write "\x0F\x01".b # Display test
port.write "\x0F\x00".b # Display test

def filter bm
  if bm & (1 << 7) == 0 && bm & 1 == 0
    bm
  else
    # if the top bit is off, we're moving clockwise
    if bm & (1 << 7) == 0
      (bm | (1 << 6)) & ~(1 << 0)
    else
      (bm | (1 << 1)) & ~(1 << 7)
    end
  end
end

loop do
  break
  l = 0.1
  port.write([1, 0b01100001].pack("CC"))
  sleep l
  port.write([1, 0b00100101].pack("CC"))
  sleep l
  port.write([1, 0b00001101].pack("CC"))
  sleep l
  port.write([1, 0b00011100].pack("CC"))
  sleep l
  port.write([1, 0b00011001].pack("CC"))
  sleep l
  port.write([1, 0b00010011].pack("CC"))
  sleep l
  port.write([1, 0b01000011].pack("CC"))
  sleep l
  port.write([1, 0b01100010].pack("CC"))
  sleep l
end

bitmask = (0x7 << 1)
bitmask2 = (0x7 << 1)

port.write([1, 0b00001110].pack("CC")) # L
port.write([2, 0b01111110].pack("CC")) # O
port.write([3, 0b00001110].pack("CC")) # L
exit

loop do
  bitmask = filter(bitmask >> 1)
  bitmask2 = filter(bitmask2 << 1)
  port.write([1, bitmask].pack("CC"))
  port.write([2, bitmask2].pack("CC"))
  port.write([3, bitmask].pack("CC"))
  sleep 0.1
end

loop do
  9.times do |r|
    port.write([1, r].pack("CC"))
    port.write([2, r].pack("CC"))
    port.write([3, r].pack("CC"))
    sleep 0.250
    #port.write([1, r | (1 << 7)].pack("CC"))
    #sleep 0.250
  end
end
