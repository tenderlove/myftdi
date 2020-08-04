require "libusb"

class MyFTDI
  # A wrapper around the USB object so we can more easily swap / stub in test
  class USB # :nodoc:
    def self.ft232h
      new LIBUSB::Context.new.devices(idVendor: 0x0403, idProduct: 0x6014).first || raise
    end

    def initialize dev
      @usb_dev = dev
    end

    def bulk_transfer
      ep = @usb_dev.endpoints.last

      @usb_dev.open_interface(0) do |handle|
        writer = ->(chunk) {
          handle.bulk_transfer(endpoint: ep, dataOut: chunk)
        }
        yield writer
      end
    end

    def max_packet_size
      endpoint = @usb_dev.endpoints.first
      endpoint.wMaxPacketSize
    end

    def bcdDevice; @usb_dev.bcdDevice; end

    def write_cmd req_type, cmd, value
      @usb_dev.open_interface(0) do |handle|
        handle.control_transfer(bmRequestType: req_type, bRequest: cmd, wValue: value, wIndex: 0x0000, timeout: 0, dataOut: ''.b)
      end
    end

    def read size
      @usb_dev.open_interface(0) do |handle|
        handle.bulk_transfer(endpoint: @usb_dev.endpoints.first, dataIn: size)
      end
    end
  end
end
