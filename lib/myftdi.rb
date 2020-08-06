require "logger"
require "forwardable"
require "myftdi/usb"
require "myftdi/spi"

class MyFTDI
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

  ERROR_BITS = [0x00, 0x8E]
  TX_EMPTY_BITS = 0x60

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

  # MyFTDI MPSSE commands
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

  def self.ft232h_device
    MyFTDI::USB.ft232h
  end

  def self.ft232h_i2c
    device = ft232h_device

    ftdi = new(device)
    controller = I2C::Controller.new ftdi, device
    direction, initial, frequency = controller.configure
    configure_mpsse(ftdi, frequency: frequency, initial: initial, direction: direction, latency: 16)
    controller.finish_config
    controller
  end

  def self.ft232h_spi
    device = ft232h_device

    config = SPIController::Config.new 1, true
    direction = config.direction
    initial = config.cs_bits

    ftdi = new(device)
    configure_mpsse(ftdi, frequency: 6e6, initial: initial, direction: direction, latency: 16)

    SPIController.new ftdi, config
  end

  def self.configure_mpsse ftdi, frequency:, initial:, direction:, latency:
    ftdi.set_bitmode 0, MyFTDI::BitModes::RESET
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

  class BufferedReader
    def initialize usb, controller
      @usb = usb
      @offset = 0
      @buffer = ''
      @controller   = controller
    end

    def read size
      packet_size = @controller.max_packet_size


      tmpbuf = @controller.read
      length = tmpbuf.bytesize

      want = size + 2 - length

      while want > 0
        b = @controller.read
        length = b.bytesize
        want -= length
        tmpbuf += b
        @controller.logger.debug "#{__method__} read_size #{length}"
        @controller.logger.debug "#{__method__} packet_size #{packet_size}"
      end

      length = tmpbuf.bytesize

      if length > 2
        chunks = (length + packet_size - 1) / packet_size
        count = packet_size - 2
        status = tmpbuf[0, 2].bytes
        if status[1] & ERROR_BITS[1] != 0
          raise "oh no!"
        end
        return tmpbuf[2..-1]
      else
        if length == 2
          status = tmpbuf[0, 2].bytes
          if status[1] & ERROR_BITS[1] != 0
            raise "oh no!"
          end
          return :empty
        else
          raise "argh!!"
        end
      end
    end

    private
    attr_reader :offset
  end

  attr_reader :logger, :frequency

  def initialize dev
    @usb_dev               = dev
    @logger                = Logger.new $stderr
    @logger.level          = Logger::INFO
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
    @buffered_reader       = BufferedReader.new @usb_dev, self
    purge_buffers
    reset_device
  end

  def mpsse_bit_delay
    0.5e-6
  end

  def enable_drivezero_mode v
    write_data([MyFTDI::DRIVE_ZERO, v & 0xff, (v >> 8) & 0xff].pack("CCC"))
  end

  def enable_adaptive_clock v
    if v
      write_data ENABLE_CLK_ADAPTIVE.chr
    else
      write_data DISABLE_CLK_ADAPTIVE.chr
    end
  end

  def enable_3phase_clock v
    if v
      write_data ENABLE_CLK_3PHASE.chr
    else
      write_data DISABLE_CLK_3PHASE.chr
    end
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

  def buffered_read size
    @buffered_reader.read size
  end

  def read_data size
    read
  end

  def write_data cmd
    offset     = 0
    size       = cmd.bytesize
    write_size = writebuffer_chunksize

    @usb_dev.bulk_transfer do |handle|
      while offset < size
        bytes = cmd.byteslice(offset, write_size)
        logger.debug "> #{bytes.inspect} "

        # FIXME: add timeout
        length = handle.call(cmd.byteslice(offset, write_size))
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

  def fifo_sizes
    FIFO_SIZES[version]
  end

  def writebuffer_chunksize
    @writebuffer_chunksize ||= FIFO_SIZES[version].first
  end

  def readbuffer_chunksize
    @readbuffer_chunksize ||= (FIFO_SIZES[version] + [max_packet_size]).min
  end

  def read
    @usb_dev.read readbuffer_chunksize
  end

  def max_packet_size
    @usb_dev.max_packet_size
  end

  def set_bitmode bitmask, mode
    mask = MyFTDI::BitModes::MASK
    value = (bitmask & 0xFF) | ((mode & mask) << 8)
    cmd = SIO_REQ_SET_BITMODE
    write_cmd cmd, value
  end

  def reset_device
    reset SIO_RESET_SIO
  end

  def purge_rx_buffer
    reset SIO_RESET_PURGE_RX
  end

  def purge_tx_buffer
    reset SIO_RESET_PURGE_TX
  end

  def reset value
    write_cmd SIO_REQ_RESET, value
  end

  private

  def divcalc speed, frequency
    divisor = ((speed + frequency / 2) / frequency).to_i - 1
    divisor = [0, [0xFFFF, divisor].min].max
    actual_freq = speed / (divisor + 1)
    error = (actual_freq / frequency) - 1
    [divisor, actual_freq, error]
  end

  def write_cmd cmd, value
    req_type = LIBUSB::REQUEST_TYPE_VENDOR |
      LIBUSB::RECIPIENT_DEVICE |
      LIBUSB::ENDPOINT_OUT

    @usb_dev.write_cmd req_type, cmd, value
  end

  def endpoint
    @usb_dev.endpoints.first
  end
end
