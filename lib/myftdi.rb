require "libusb"
require "logger"

class FTDI
  module BitModes
    RESET = 0x00    # switch off altnerative mode (default to UART)
    BITBANG = 0x01  # classical asynchronous bitbang mode
    MPSSE = 0x02    # MPSSE mode, available on 2232x chips
    SYNCBB = 0x04   # synchronous bitbang mode
    MCU = 0x08      # MCU Host Bus Emulation mode,
    OPTO = 0x10     # Fast Opto-Isolated Serial Interface Mode
    CBUS = 0x20     # Bitbang on CBUS pins of R-type chips
    SYNCFF = 0x40   # Single Channel Synchronous FIFO mode
    MASK = 0x7F
  end

  FIFO_SIZES = {
    0x0200 => [128, 128],    # FT232AM: TX: 128, RX: 128
    0x0400 => [128, 384],    # FT232BM: TX: 128, RX: 384
    0x0500 => [128, 384],    # FT2232C: TX: 128, RX: 384
    0x0600 => [256, 128],    # FT232R:  TX: 256, RX: 128
    0x0700 => [4096, 4096],  # FT2232H: TX: 4KiB, RX: 4KiB
    0x0800 => [2048, 2048],  # FT4232H: TX: 2KiB, RX: 2KiB
    0x0900 => [1024, 1024],  # FT232H:  TX: 1KiB, RX: 1KiB
    0x1000 => [512, 512],    # FT-X:    TX: 512, RX: 512
  }

  # Reset arguments
  SIO_RESET_SIO = 0        # Reset device
  SIO_RESET_PURGE_RX = 1   # Drain USB RX buffer (host-to-ftdi)
  SIO_RESET_PURGE_TX = 2   # Drain USB TX buffer (ftdi-to-host)

  # Requests
  SIO_REQ_RESET = 0x0              # Reset the port
  SIO_REQ_SET_MODEM_CTRL = 0x1     # Set the modem control register
  SIO_REQ_SET_FLOW_CTRL = 0x2      # Set flow control register
  SIO_REQ_SET_BAUDRATE = 0x3       # Set baud rate
  SIO_REQ_SET_DATA = 0x4           # Set the data characteristics of the port
  SIO_REQ_POLL_MODEM_STATUS = 0x5  # Get line status
  SIO_REQ_SET_EVENT_CHAR = 0x6     # Change event character
  SIO_REQ_SET_ERROR_CHAR = 0x7     # Change error character
  SIO_REQ_SET_LATENCY_TIMER = 0x9  # Change latency timer
  SIO_REQ_GET_LATENCY_TIMER = 0xa  # Get latency timer
  SIO_REQ_SET_BITMODE = 0xb        # Change bit mode
  SIO_REQ_READ_PINS = 0xc          # Read GPIO pin value (or "get bitmode")

  # Clocks and baudrates
  BUS_CLOCK_BASE = 6.0E6  # 6 MHz
  BUS_CLOCK_HIGH = 30.0E6  # 30 MHz
  BAUDRATE_REF_BASE = 3.0E6.to_i  # 3 MHz
  BAUDRATE_REF_HIGH = 12.0E6.to_i  # 12 MHz
  BITBANG_BAUDRATE_RATIO_BASE = 16
  BITBANG_BAUDRATE_RATIO_HIGH = 5
  BAUDRATE_TOLERANCE = 3.0  # acceptable clock drift for UART, in %

  # MPSSE Commands
  WRITE_BYTES_PVE_MSB = 0x10
  WRITE_BYTES_NVE_MSB = 0x11
  WRITE_BITS_PVE_MSB = 0x12
  WRITE_BITS_NVE_MSB = 0x13
  WRITE_BYTES_PVE_LSB = 0x18
  WRITE_BYTES_NVE_LSB = 0x19
  WRITE_BITS_PVE_LSB = 0x1a
  WRITE_BITS_NVE_LSB = 0x1b
  READ_BYTES_PVE_MSB = 0x20
  READ_BYTES_NVE_MSB = 0x24
  READ_BITS_PVE_MSB = 0x22
  READ_BITS_NVE_MSB = 0x26
  READ_BYTES_PVE_LSB = 0x28
  READ_BYTES_NVE_LSB = 0x2c
  READ_BITS_PVE_LSB = 0x2a
  READ_BITS_NVE_LSB = 0x2e
  RW_BYTES_PVE_NVE_MSB = 0x31
  RW_BYTES_NVE_PVE_MSB = 0x34
  RW_BITS_PVE_PVE_MSB = 0x32
  RW_BITS_PVE_NVE_MSB = 0x33
  RW_BITS_NVE_PVE_MSB = 0x36
  RW_BITS_NVE_NVE_MSB = 0x37
  RW_BYTES_PVE_NVE_LSB = 0x39
  RW_BYTES_NVE_PVE_LSB = 0x3c
  RW_BITS_PVE_PVE_LSB = 0x3a
  RW_BITS_PVE_NVE_LSB = 0x3b
  RW_BITS_NVE_PVE_LSB = 0x3e
  RW_BITS_NVE_NVE_LSB = 0x3f
  WRITE_BITS_TMS_PVE = 0x4a
  WRITE_BITS_TMS_NVE = 0x4b
  RW_BITS_TMS_PVE_PVE = 0x6a
  RW_BITS_TMS_PVE_NVE = 0x6b
  RW_BITS_TMS_NVE_PVE = 0x6e
  RW_BITS_TMS_NVE_NVE = 0x6f
  SEND_IMMEDIATE = 0x87
  WAIT_ON_HIGH = 0x88
  WAIT_ON_LOW = 0x89
  READ_SHORT = 0x90
  READ_EXTENDED = 0x91
  WRITE_SHORT = 0x92
  WRITE_EXTENDED = 0x93
  # -H series only
  DISABLE_CLK_DIV5 = 0x8a
  ENABLE_CLK_DIV5 = 0x8b

  # Modem status
  MODEM_CTS = (1 << 4)    # Clear to send
  MODEM_DSR = (1 << 5)    # Data set ready
  MODEM_RI = (1 << 6)     # Ring indicator
  MODEM_RLSD = (1 << 7)   # Carrier detect
  MODEM_DR = (1 << 8)     # Data ready
  MODEM_OE = (1 << 9)     # Overrun error
  MODEM_PE = (1 << 10)    # Parity error
  MODEM_FE = (1 << 11)    # Framing error
  MODEM_BI = (1 << 12)    # Break interrupt
  MODEM_THRE = (1 << 13)  # Transmitter holding register
  MODEM_TEMT = (1 << 14)  # Transmitter empty
  MODEM_RCVE = (1 << 15)  # Error in RCVR FIFO

  # FTDI MPSSE commands
  SET_BITS_LOW = 0x80     # Change LSB GPIO output
  SET_BITS_HIGH = 0x82    # Change MSB GPIO output
  GET_BITS_LOW = 0x81     # Get LSB GPIO output
  GET_BITS_HIGH = 0x83    # Get MSB GPIO output
  LOOPBACK_START = 0x84   # Enable loopback
  LOOPBACK_END = 0x85     # Disable loopback
  SET_TCK_DIVISOR = 0x86  # Set clock
  # -H series only
  ENABLE_CLK_3PHASE = 0x8c       # Enable 3-phase data clocking (I2C)
  DISABLE_CLK_3PHASE = 0x8d      # Disable 3-phase data clocking
  CLK_BITS_NO_DATA = 0x8e        # Allows JTAG clock to be output w/o data
  CLK_BYTES_NO_DATA = 0x8f       # Allows JTAG clock to be output w/o data
  CLK_WAIT_ON_HIGH = 0x94        # Clock until GPIOL1 is high
  CLK_WAIT_ON_LOW = 0x95         # Clock until GPIOL1 is low
  ENABLE_CLK_ADAPTIVE = 0x96     # Enable JTAG adaptive clock for ARM
  DISABLE_CLK_ADAPTIVE = 0x97    # Disable JTAG adaptive clock
  CLK_COUNT_WAIT_ON_HIGH = 0x9c  # Clock byte cycles until GPIOL1 is high
  CLK_COUNT_WAIT_ON_LOW = 0x9d   # Clock byte cycles until GPIOL1 is low
  # FT232H only
  DRIVE_ZERO = 0x9e       # Drive-zero mode

  # Latency
  LATENCY_MIN = 12
  LATENCY_MAX = 255
  LATENCY_EEPROM_FT232R = 77

  def self.ft232h_spi
    usb = LIBUSB::Context.new
    device = usb.devices(idVendor: 0x0403, idProduct: 0x6014).first || raise
    spi = SPIController.new nil, 1, true
    direction = spi.spi_dir | spi.gpio_dir
    initial = spi.cs_bits
    ftdi = new(usb, device)
    configure_mpsse(ftdi, frequency: 6e6, initial: initial, direction: direction, latency: 16)
    spi.ftdi = ftdi
    spi
  end

  def self.configure_mpsse ftdi, frequency:, initial:, direction:, latency:
    ftdi.set_bitmode 0, FTDI::BitModes::RESET
    ftdi.latency_timer = latency # ms

    # Disable event and error characters
    ftdi.set_event_char 0, false
    ftdi.set_error_char 0, false

    # Enable MPSSE mode
    ftdi.set_bitmode direction, BitModes::MPSSE

    # Configure clock
    frequency = ftdi.set_frequency frequency

    # Configure I/O
    cmd = [SET_BITS_LOW, initial & 0xFF, direction & 0xFF]
    if ftdi.wide_port?
      initial >>= 8
      direction >>= 8
      cmd.concat [SET_BITS_HIGH, initial & 0xFF, direction & 0xFF]
    end
    ftdi.write_data(cmd.pack("C*"))

    # Disable loopback
    ftdi.write_data([LOOPBACK_END].pack("C"))
    ftdi.validate_mpsse
  end

  module SPI
    SCK_BIT = 0x01
    DO_BIT = 0x02
    DI_BIT = 0x04
    CS_BIT = 0x08
    SPI_BITS = DI_BIT | DO_BIT | SCK_BIT
  end

  class SPIController
    include SPI

    attr_reader :spi_dir, :gpio_dir, :cs_bits, :gpio_mask, :spi_mask, :gpio_low, :turbo, :clock_phase
    attr_accessor :ftdi

    def initialize ftdi, cs_count, turbo
      @ftdi        = ftdi
      @gpio_port   = nil
      @gpio_dir    = 0
      @gpio_mask   = 0
      @gpio_low    = 0
      @wide_port   = false
      @cs_count    = cs_count
      @turbo       = turbo
      @clock_phase = false
      @cs_bits     = 0
      @spi_ports   = []
      @spi_dir     = 0
      @spi_mask    = SPI_BITS
      configure
    end

    def frequency
      @ftdi.frequency
    end

    def get_port cs, mode
      hold = 1 + (1e6 / frequency).to_i
      puts frequency
      puts hold
      SPIPort.new self, cs, frequency, hold, mode
    end

    def exchange frequency, out, readlen, cs_prolog, cs_epilog, cpol, cpha, duplex, droptail
      if duplex
        raise NotImplementedError, "duplex not implemented yet"
      end

      if cpha
        raise NotImplementedError, "what is this?"
      end

      if frequency != self.frequency
        raise NotImplementedError, "frequencies don't match %d %d" % [frequency, self.frequency]
      end

      logger.debug [frequency, out, readlen, cs_prolog.pack('C*'), cs_epilog.pack('C*'), cpol, cpha, duplex, droptail].inspect

      if duplex
      else
        exchange_half_duplex(frequency, out, readlen, cs_prolog, cs_epilog, cpol, cpha, droptail)
      end
      #cmd = cs_prolog.flat_map { |byte|
    end

    def exchange_half_duplex frequency, out, readlen, cs_prolog, cs_epilog, cpol, cpha, droptail
      direction = self.direction & 0xFF
      cmd = ''.b
      cs_prolog.each do |ctrl|
        ctrl = (ctrl & spi_mask) | gpio_low
        cmd << [FTDI::SET_BITS_LOW, ctrl, direction].pack("C3")
      end

      epilog = ''.b
      if cs_epilog
        cs_epilog.each do |ctrl|
          ctrl = (ctrl & spi_mask) | gpio_low
          epilog << [FTDI::SET_BITS_LOW, ctrl, direction].pack("C3")
        end
        # Restore idle state
        epilog << [FTDI::SET_BITS_LOW, cs_bits | gpio_low, direction].pack("C3")
        unless self.turbo
          epilog << FTDI::SEND_IMMEDIATE.chr
        end
      end
      writelen = out.bytesize

      if self.clock_phase != cpha
        raise NotImplementedError, "can't switch 3phase clock"
      end

      if !out.empty?
        if droptail.zero?
          wcmd = cpol ? FTDI::WRITE_BYTES_PVE_MSB : FTDI::WRITE_BYTES_NVE_MSB
          cmd << [wcmd, writelen - 1].pack("CS")
          cmd << out
        else
          raise NotImplementedError, "droptail isn't implemented yet"
        end
      end

      if readlen > 0
        raise NotImplementedError, "don't know how to read yet"
      else
        if turbo
          cmd << epilog
          ftdi.write_data cmd
        else
          raise NotImplementedError, "don't know how to non-turbo write yet"
        end
      end
    end

    def direction
      spi_dir | gpio_dir
    end

    private

    def logger
      ftdi.logger
    end

    def configure
      @cs_bits = ((CS_BIT << @cs_count) - 1) & ~(CS_BIT - 1)
      @spi_dir = @cs_bits | DO_BIT | SCK_BIT
      @spi_mask = @cs_bits | SPI_BITS
      set_gpio_direction(16, (~@spi_mask) & 0xFFFF, 0)
    end

    def set_gpio_direction width, pins, direction
      gpio_mask = ((1 << width) - 1) & ~@spi_mask

      if pins & gpio_mask != pins
        raise "No such GPIO pins"
      end

      @gpio_dir &= ~pins
      @gpio_dir |= (pins & direction)
      @gpio_mask = gpio_mask & pins
    end
  end

  attr_reader :logger, :frequency

  def initialize ctx, dev
    @ctx                   = ctx
    @usb_dev               = dev
    @logger                = Logger.new $stderr
    @usb_read_timeout      = 5000
    @usb_write_timeout     = 5000
    @baudrate              = -1
    @readbuffer            = []
    @readoffset            = 0
    @readbuffer_chunksize  = 4 << 10
    @writebuffer_chunksize = 4 << 10
    @max_packet_size       = 0
    @interface             = nil
    @index                 = nil
    @in_ep                 = nil
    @out_ep                = nil
    @bitmode               = BitModes::RESET
    @latency               = 0
    @latency_count         = 0
    @latency_min           = LATENCY_MIN
    @latency_max           = LATENCY_MAX
    @latency_threshold     = nil  # disable dynamic latency
    @lineprop              = 0
    @cbus_pins             = [0, 0]
    @cbus_out              = 0
    @tracer                = nil
    purge_buffers
    reset_device
  end

  def divcalc speed, frequency
    divisor = ((speed + frequency / 2) / frequency).to_i - 1
    divisor = [0, [0xFFFF, divisor].min].max
    actual_freq = speed / (divisor + 1)
    error = (actual_freq / frequency) - 1
    [divisor, actual_freq, error]
  end

  def set_frequency frequency
    if frequency > max_frequency
      raise "Unsupported frequency #{frequency}"
    end

    divcode = ENABLE_CLK_DIV5
    divisor, actual_freq, error = divcalc(BUS_CLOCK_BASE, frequency)

    if h_series?
      # high speed clock divider
      divisor_hs, actual_freq_hs, error_hs = divcalc(BUS_CLOCK_HIGH, frequency)

      if error_hs.abs < error.abs
        divcode = DISABLE_CLK_DIV5
        divisor = divisor_hs
        actual_freq = actual_freq_hs
        error = error_hs
      end
    end

    cmd = if h_series?
            [divcode]
          else
            raise NotImplementedError
          end
    cmd.concat [SET_TCK_DIVISOR, divisor & 0xFF, (divisor >> 8) & 0xFF]
    write_data cmd.pack("C*")
    validate_mpsse
    purge_rx_buffer

    if actual_freq > 1E6
      logger.debug('Clock frequency: %.6f MHz (error: %+.1f %%)' %
                     [(actual_freq/1E6), error*100])
    else
      logger.debug('Clock frequency: %.3f KHz (error: %+.1f %%)' %
                   [(actual_freq/1E3), error*100])
    end
    @frequency = actual_freq
    actual_freq
  end

  def validate_mpsse
    bytes = read_data 2

    if bytes.bytesize >= 2 && bytes[0] == "\xfa"
      raise "Invalid command %d" % bytes[1].ord
    end
  end

  def read_data size
    read
  end

  def write_data cmd
    offset     = 0
    size       = cmd.bytesize
    write_size = writebuffer_chunksize

    ep = @usb_dev.endpoints.last

    @usb_dev.open_interface(0) do |handle|
      while offset < size
        bytes = cmd.byteslice(offset, write_size)
        logger.debug "> #{bytes.inspect} "

        # FIXME: add timeout
        length = handle.bulk_transfer(endpoint: ep, dataOut: cmd.byteslice(offset, write_size))
        offset += length
      end
    end

    offset
  end

  def max_frequency
    if h_series?
      BUS_CLOCK_HIGH
    else
      BUS_CLOCK_BASE
    end
  end

  def h_series?
    [0x700, 0x800, 0x900].include? version
  end

  def set_error_char event_ch, enable
    value = (enable ? (1 << 8) : 0) | event_ch
    write_cmd SIO_REQ_SET_ERROR_CHAR, value
  end

  def set_event_char error_ch, enable
    value = (enable ? (1 << 8) : 0) | error_ch
    write_cmd SIO_REQ_SET_EVENT_CHAR, value
  end

  def purge_buffers
    purge_rx_buffer
    purge_tx_buffer
  end

  # Returns true if the gpio port is wide (IOW 16bit)
  def wide_port?
    port_width > 8
  end

  # return the width of the port in bits
  def port_width
    case version
    when 0x700, 0x900 then 16
    when 0x500        then 12
    else
      8
    end
  end

  def version
    @usb_dev.bcdDevice
  end

  def latency_timer= v
    cmd = SIO_REQ_SET_LATENCY_TIMER
    write_cmd cmd, v
  end

  def latency_timer
    req_type = LIBUSB::REQUEST_TYPE_VENDOR |
      LIBUSB::RECIPIENT_DEVICE |
      LIBUSB::ENDPOINT_IN
    cmd = SIO_REQ_GET_LATENCY_TIMER
    @usb_dev.open_interface(0) do |handle|
      handle.control_transfer(bmRequestType: req_type, bRequest: cmd, wValue: 0x00, wIndex: 0x0000, timeout: 0, dataIn: 1).unpack1('c')
    end
  end

  def writebuffer_chunksize
    @writebuffer_chunksize ||= FIFO_SIZES[version].first
  end

  def readbuffer_chunksize
    @readbuffer_chunksize ||= (FIFO_SIZES[version] + [max_packet_size]).min
  end

  def read
    @usb_dev.open_interface(0) do |handle|
      handle.bulk_transfer(endpoint: @usb_dev.endpoints.first, dataIn: readbuffer_chunksize)
    end
  end

  def max_packet_size
    endpoint.wMaxPacketSize
  end

  def set_bitmode bitmask, mode
    mask = FTDI::BitModes::MASK
    value = (bitmask & 0xFF) | ((mode & mask) << 8)
    cmd = SIO_REQ_SET_BITMODE
    write_cmd cmd, value
  end

  def reset_device
    cmd = SIO_REQ_RESET
    value = SIO_RESET_SIO
    write_cmd cmd, value
  end

  def purge_rx_buffer
    cmd = SIO_REQ_RESET
    value = SIO_RESET_PURGE_RX
    write_cmd cmd, value
  end

  def purge_tx_buffer
    cmd = SIO_REQ_RESET
    value = SIO_RESET_PURGE_TX
    write_cmd cmd, value
  end

  class SPIPort
    include SPI

    PAYLOAD_MAX_LENGTH = 0xFF00  # 16 bits max (- spare for control)

    attr_reader :controller, :cs_hold, :mode

    def initialize controller, cs, frequency, cs_hold, mode
      @controller = controller
      @cs         = cs
      @cs_hold    = cs_hold
      @mode       = mode
      set_mode mode
    end

    def frequency
      controller.frequency
    end

    def write bytes
      @controller.exchange frequency, bytes, 0, @cs_prolog, @cs_epilog, @cpol, @cpha, false, 0
    end

    private

    def set_mode mode
      raise "Invalid SPI mode" unless (0..3).include?(mode)

      @cpol       = !(mode & 0x2).zero?
      @cpha       = !(mode & 0x1).zero?

      cs_hold = 1

      cs_clock = 0xFF & ~(((@cpol ? 0 : 1) & SCK_BIT) | DO_BIT)
      cs_select = 0xFF & ~((CS_BIT << @cs) | ((@cpol ? 0 : 1) & SCK_BIT) | DO_BIT)
      @cs_prolog = [cs_clock, cs_select]
      @cs_epilog = ([cs_select] + [cs_clock] * cs_hold)
    end
  end

  private

  def write_cmd cmd, value
    req_type = LIBUSB::REQUEST_TYPE_VENDOR |
      LIBUSB::RECIPIENT_DEVICE |
      LIBUSB::ENDPOINT_OUT

    @usb_dev.open_interface(0) do |handle|
      handle.control_transfer(bmRequestType: req_type, bRequest: cmd, wValue: value, wIndex: 0x0000, timeout: 0, dataOut: ''.b)
    end
  end

  def endpoint
    @usb_dev.endpoints.first
  end
end

dev = FTDI.ft232h_spi

port = dev.get_port 0, 0

# Synchronous exchange with the remote SPI port
write_buf = "\x0C\x01\x09\xFF\x0A\x0F\x0B\x07\x0F\x01".b
port.write(write_buf)
sleep(1)
write_buf = "\x0F\x00".b
port.write(write_buf)
write_buf = "\x01\x0F\x02\x0F\x03\x0F\x04\x0F\x05\x0F\x06\x0F\x07\x0F\x08\x0F".b
port.write(write_buf)

def set_digits(dev, num)
  buf = ''.b
  8.times do |x|
    dig = num % 10
    dev.write([x + 1, dig].pack("CC"))
    num = num / 10
  end
end
set_digits(port, 22121)
