class MyFTDI
  module SPI
    SCK_BIT = 0x01
    DO_BIT = 0x02
    DI_BIT = 0x04
    CS_BIT = 0x08
    SPI_BITS = DI_BIT | DO_BIT | SCK_BIT
  end

  class SPIController
    include SPI
    extend Forwardable

    class Config
      include SPI

      attr_reader :spi_dir, :gpio_dir, :cs_bits, :gpio_mask, :spi_mask
      attr_reader :gpio_low, :turbo, :clock_phase

      def initialize cs_count, turbo
        @gpio_dir    = 0
        @gpio_mask   = 0
        @gpio_low    = 0
        @cs_count    = cs_count
        @turbo       = turbo
        @clock_phase = false
        @cs_bits     = 0
        @spi_dir     = 0
        @spi_mask    = SPI_BITS
        configure
      end

      def direction
        spi_dir | gpio_dir
      end

      private

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

    attr_reader :ftdi

    def_delegators :@config, :spi_dir, :gpio_dir, :cs_bits, :spi_mask, :gpio_low
    def_delegators :@config, :turbo, :clock_phase, :direction
    def_delegators :@ftdi, :frequency, :logger

    def initialize ftdi, config
      @ftdi        = ftdi
      @config      = config
    end

    def get_port cs, mode
      hold = 1 + (1e6 / frequency).to_i
      SPIPort.new self, cs, hold, mode
    end

    def exchange out, readlen, cs_prolog, cs_epilog, cpol, cpha, duplex, droptail
      if duplex
        raise NotImplementedError, "duplex not implemented yet"
      end

      if cpha
        raise NotImplementedError, "what is this?"
      end

      logger.debug [frequency, out, readlen, cs_prolog.pack('C*'), cs_epilog.pack('C*'), cpol, cpha, duplex, droptail].inspect

      if duplex
      else
        exchange_half_duplex(out, readlen, cs_prolog, cs_epilog, cpol, cpha, droptail)
      end
    end

    def exchange_half_duplex out, readlen, cs_prolog, cs_epilog, cpol, cpha, droptail
      direction = self.direction & 0xFF
      cmd = ''.b
      cs_prolog.each do |ctrl|
        ctrl = (ctrl & spi_mask) | gpio_low
        cmd << [MyFTDI::SET_BITS_LOW, ctrl, direction].pack("C3")
      end

      epilog = ''.b
      if cs_epilog
        cs_epilog.each do |ctrl|
          ctrl = (ctrl & spi_mask) | gpio_low
          epilog << [MyFTDI::SET_BITS_LOW, ctrl, direction].pack("C3")
        end
        # Restore idle state
        epilog << [MyFTDI::SET_BITS_LOW, cs_bits | gpio_low, direction].pack("C3")
        unless self.turbo
          epilog << MyFTDI::SEND_IMMEDIATE.chr
        end
      end
      writelen = out.bytesize

      if self.clock_phase != cpha
        raise NotImplementedError, "can't switch 3phase clock"
      end

      if !out.empty?
        if droptail.zero?
          wcmd = cpol ? MyFTDI::WRITE_BYTES_PVE_MSB : MyFTDI::WRITE_BYTES_NVE_MSB
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
  end

  class SPIPort
    include SPI

    PAYLOAD_MAX_LENGTH = 0xFF00  # 16 bits max (- spare for control)

    attr_reader :controller, :cs_hold, :mode

    def initialize controller, cs, cs_hold, mode
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
      @controller.exchange bytes, 0, @cs_prolog, @cs_epilog, @cpol, @cpha, false, 0
    end

    private

    def set_mode mode
      raise "Invalid SPI mode" unless (0..3).include?(mode)

      @cpol       = !(mode & 0x2).zero?
      @cpha       = !(mode & 0x1).zero?

      cs_clock = 0xFF & ~(((@cpol ? 0 : 1) & SCK_BIT) | DO_BIT)
      cs_select = 0xFF & ~((CS_BIT << @cs) | ((@cpol ? 0 : 1) & SCK_BIT) | DO_BIT)
      @cs_prolog = [cs_clock, cs_select]
      @cs_epilog = ([cs_select] + [cs_clock] * cs_hold)
    end
  end
end
