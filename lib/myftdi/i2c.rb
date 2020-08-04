class MyFTDI
  module I2C
    LOW = 0x00
    HIGH = 0xff
    BIT0 = 0x01
    IDLE = HIGH
    SCL_BIT = 0x01  #AD0
    SDA_O_BIT = 0x02  #AD1
    SDA_I_BIT = 0x04  #AD2
    SCL_FB_BIT = 0x80  #AD7
    PAYLOAD_MAX_LENGTH = 0xFF00  # 16 bits max (- spare for control)
    HIGHEST_I2C_ADDRESS = 0x78
    DEFAULT_BUS_FREQUENCY = 100000.0
    HIGH_BUS_FREQUENCY = 400000.0
    RETRY_COUNT = 3

    I2C_MASK = SCL_BIT | SDA_O_BIT | SDA_I_BIT
    I2C_MASK_CS = SCL_BIT | SDA_O_BIT | SDA_I_BIT | SCL_FB_BIT
    I2C_DIR = SCL_BIT | SDA_O_BIT

    Timings = Struct.new :t_hd_sta, :t_su_sta, :t_su_sto, :t_buf

    I2C_100K = Timings.new(4.0E-6, 4.7E-6, 4.0E-6, 4.7E-6)
    #I2C_400K = I2CTimings(0.6E-6, 0.6E-6, 0.6E-6, 1.3E-6)
    #I2C_1M = I2CTimings(0.26E-6, 0.26E-6, 0.26E-6, 0.5E-6)

    class Config
    end

    class Port
      def initialize controller, address
        @controller = controller
        @address    = address
      end

      def read_from from, length
        buffer = [from].pack("C")
        @controller.exchange(@address, buffer, length)
      end
    end

    class Controller
      def initialize ftdi, dev
        @ftdi = ftdi
        @dev = dev
        @gpio_port = nil
        @gpio_dir = 0
        @gpio_low = 0
        @gpio_mask = 0
        @i2c_mask = 0
        @wide_port = false
        @slaves = {}
        @retry_count = RETRY_COUNT
        @frequency = 0.0
        @immediate  = [MyFTDI::SEND_IMMEDIATE]
        @read_bit   = [MyFTDI::READ_BITS_PVE_MSB, 0]
        @read_byte  = [MyFTDI::READ_BYTES_PVE_MSB, 0, 0]
        @write_byte = [MyFTDI::WRITE_BYTES_NVE_MSB, 0, 0]
        @nack       = [MyFTDI::WRITE_BITS_NVE_MSB, 0, HIGH]
        @ack        = [MyFTDI::WRITE_BITS_NVE_MSB, 0, LOW]
        @ck_delay = 1
        @fake_tristate = false
        @tx_size = 1
        @rx_size = 1
        @ck_hd_sta = 0
        @ck_su_sto = 0
        @ck_idle = 0
        @read_optim = true
      end

      def configure
        frequency = DEFAULT_BUS_FREQUENCY
        timings = I2C_100K
        clkstrch = false
        io_dir = 0
        io_out = 0
        interface = 1
        @ck_hd_sta = compute_delay_cycles timings.t_hd_sta
        @ck_su_sto = compute_delay_cycles timings.t_su_sto

        ck_su_sta = compute_delay_cycles(timings.t_su_sta)
        ck_buf = compute_delay_cycles(timings.t_buf)
        @ck_idle = [ck_su_sta, ck_buf].max
        @ck_delay = ck_buf

        @i2c_mask = if clkstrch
                      I2C_MASK_CS
                    else
                      I2C_MASK
                    end
        set_gpio_direction 16, io_out, io_dir
        direction = I2C_DIR | @gpio_dir
        initial = IDLE | (io_out & @gpio_mask)
        frequency = (3.0 * frequency) / 2.0
        [direction, initial, frequency]
      end

      def finish_config
        @tx_size, @rx_size = @ftdi.fifo_sizes
        @ftdi.enable_adaptive_clock(false) # @ftdi.enable_adaptive_clock(clkstrch)
        @ftdi.enable_3phase_clock(true)
        @ftdi.enable_drivezero_mode(SCL_BIT | SDA_O_BIT | SDA_I_BIT)

        unless @ftdi.wide_port?
          raise NotImplementedError
        end
      end

      def get_port address
        Port.new self, address
      end

      def set_gpio_direction width, pins, direction
        if pins & @i2c_mask > 0
          raise "cannot access I2C pins as GPIO"
        end

        gpio_mask = (1 << width) - 1
        gpio_mask &= ~@i2c_mask

        if (pins & gpio_mask) != pins
          raise I2cIOError('No such GPIO pin(s)')
        end

        @gpio_dir &= ~pins
        @gpio_dir |= (pins & direction)
        @gpio_mask = gpio_mask & pins
      end

      def compute_delay_cycles value
        bit_delay = @ftdi.mpsse_bit_delay
        [1, (value + bit_delay) / bit_delay].max.to_i
      end

      def exchange address, out, readlen
        i2caddress = (address << 1) & HIGH
        do_prolog i2caddress
        do_write out
        do_prolog i2caddress | BIT0

        x = nil
        if readlen > 0
          x = do_read readlen
        end

        do_epilog
        x
      end

      private

      attr_reader :gpio_low, :gpio_dir, :ck_hd_sta, :write_byte, :read_bit
      attr_reader :immediate, :ftdi, :read_byte, :ack, :ck_delay, :nack, :rx_size
      attr_reader :tx_size, :read_optim, :ck_su_sto, :ck_idle

      def do_epilog
        @ftdi.write_data stop.pack("C*")
        ftdi.logger.debug "#{__method__}: #{@ftdi.buffered_read(1).dump}"
      end

      def do_prolog i2caddress
        ftdi.logger.debug "#{__method__} i2caddress: #{i2caddress}"
        cmd = idle + start + write_byte + [i2caddress]
        send_check_ack cmd.pack("C*")
      end

      def do_write out
        ftdi.logger.debug "#{__method__}"
        out.bytes.each do |byte|
          cmd = write_byte.dup
          cmd << byte
          send_check_ack(cmd.pack("C*"))
        end
      end

      def do_read readlen
        read_not_last = read_byte + ack + clk_lo_data_hi * ck_delay
        read_last     = read_byte + nack + clk_lo_data_hi * ck_delay
        chunk_size = rx_size - 2
        cmd_size = read_last.length
        tx_count = (tx_size - 1) / cmd_size
        chunk_size = [tx_count, chunk_size].min
        chunks = []
        rem = readlen

        if read_optim && rem > chunk_size
          raise NotImplementedError
        else
          while rem > 0
            if rem > chunk_size
              raise NotImplementedError
            else
              cmd = (read_not_last * (rem - 1)) + read_last + immediate
              @ftdi.write_data cmd.pack("C*")
              buf = @ftdi.buffered_read rem
              rem = 0
              return buf
            end
          end
        end
      end

      def send_check_ack buffer
        cmd = buffer + (clk_lo_data_hi + read_bit + immediate).pack("C*")
        ftdi.logger.debug "#{__method__} sending: #{cmd.dump}"
        ftdi.write_data cmd
        data = ftdi.buffered_read 1
        ftdi.logger.debug "#{__method__} read: #{data.dump}"
        data
      end

      def stop
        clk_lo_data_hi * ck_hd_sta + data_lo * ck_su_sto + idle * ck_idle
      end

      def clk_lo_data_hi
        [SET_BITS_LOW, SDA_O_BIT | gpio_low, I2C_DIR | (gpio_dir & 0xFF)]
      end

      def idle
        [SET_BITS_LOW, I2C_DIR | gpio_low, I2C_DIR | (gpio_dir & 0xFF)]
      end

      def start
        (data_lo * ck_hd_sta) + (clk_lo_data_lo * ck_hd_sta)
      end

      def clk_lo_data_lo
        [SET_BITS_LOW, gpio_low, I2C_DIR | (gpio_dir & 0xFF)]
      end

      def data_lo
        [SET_BITS_LOW, SCL_BIT | gpio_low, I2C_DIR | (gpio_dir & 0xFF)]
      end
    end
  end
end
