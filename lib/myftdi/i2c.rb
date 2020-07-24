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
        @immediate = [FTDI::SEND_IMMEDIATE]
        @read_bit = [FTDI::READ_BITS_PVE_MSB, 0]
        @read_byte = [FTDI::READ_BYTES_PVE_MSB, 0, 0]
        @write_byte = [FTDI::WRITE_BYTES_NVE_MSB, 0, 0]
        @nack = [FTDI::WRITE_BITS_NVE_MSB, 0, HIGH]
        @ack = [FTDI::WRITE_BITS_NVE_MSB, 0, LOW]
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
        @tx_size = @ftdi.writebuffer_chunksize
        @rx_size = @ftdi.readbuffer_chunksize
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
    end
  end
end
