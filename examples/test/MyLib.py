import pigpio

class MyLib:
    def __init__(self):
        self.pi = pigpio.pi()

    def i2c_acquire(self, bus, address):
        self.handle = self.pi.i2c_open(int(bus, 0), int(address, 0), 0)

    def i2c_release(self):
        self.pi.i2c_close(self.handle)

    def i2c_write_byte(self, data):
        self.pi.i2c_write_byte(self.handle, int(data, 0))

    def i2c_read_byte(self) -> int:
        return self.pi.i2c_read_byte(self.handle)

    def i2c_write_byte_data(self, reg, data):
        self.pi.i2c_write_byte_data(self.handle, int(reg, 0), int(data, 0))

    def i2c_read_byte_data(self, reg) -> int:
        return self.pi.i2c_read_byte_data(self.handle, int(reg, 0))

    def i2c_slave_init(self, address):
        s, b, d = self.pi.bsc_xfer(int(address, 0), [])
        if b:
            # if d[0] == ord('t'): # 116 send 'HH:MM:SS*'
            if d[0] == 1:
                # print("sent={} FR={} received={} [{}]".
                # format(s>>16, s&0xfff,b,d))

                s, b, d = self.pi.bsc_xfer(int(address, 0), [3])

            # elif d[0] == ord('d'): # 100 send 'Sun Oct 30*'
            elif d[0] == 2:
                print("sent={} FR={} received={} [{}]".
                format(s>>16, s&0xfff,b,d))

                # s, b, d = pi.bsc_i2c(address,
                # "{}*".format(time.asctime()[:10]))
                s, b, d = self.pi.bsc_xfer(int(address, 0), [4])

    

# b = pi.i2c_read_byte_data(handle, 0xD0)
# print(b)
# print("bubu")

# pi.i2c_close(handle)
