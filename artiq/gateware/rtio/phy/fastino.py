from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.io import DifferentialOutput, DifferentialInput, DDROutput
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine

from artiq.gateware.rtio import rtlink


class SerDes(Module):
    def transpose(self, i, n):
        # i is n,m c-contiguous
        # o is m,n c-contiguous
        m = len(i)//n
        assert n*m == len(i)

    def __init__(self, pins, pins_n):
        n_bits = 16
        n_channels = 32
        n_div = 7
        assert n_div == 7
        n_frame = 14
        n_lanes = len(pins.mosi)
        n_checksum = 12
        n_word = n_lanes*n_div
        n_body = n_word*n_frame - n_frame//2 - 1 - n_checksum

        self.dac = [Signal(n_bits, reset=i) for i in range(n_channels)]
        self.mask = Signal(n_channels, reset=(1 << n_channels) - 1)
        self.cfg = Signal(20, reset=0)
        adr = Signal(4)
        self.dat_r = Signal(n_frame//2*(1 << len(adr)))

        self.submodules.crc = LiteEthMACCRCEngine(
            data_width=4*n_lanes, width=n_checksum, polynom=0x180f)  # crc-12 telco
        checksum = Signal(n_checksum)

        body_ = Cat(self.cfg, adr, self.mask, self.dac)
        assert len(body_) == n_body
        body = Signal(n_body)
        self.comb += body.eq(body_)

        words_ = []
        j = 0
        for i in range(n_frame):
            if i == 0:
                k = n_word - n_checksum
            elif i == 1:
                words_.append(C(1))
                k = n_word - 1
            elif i < n_frame//2 + 2:
                words_.append(C(0))
                k = n_word - 1
            else:
                k = n_word
            words_.append(body[j:j + k])
            j += k
        words_ = Cat(words_)
        assert len(words_) == n_frame*n_word - n_checksum
        words = Signal(len(words_))
        self.comb += words.eq(words_)

        self.stb = Signal()
        clk = Signal(n_div, reset=0b1100011)
        clk_stb = Signal()
        i_frame = Signal(max=n_div*n_frame//2)  # DDR
        sr = [Signal(n_frame*n_div - n_checksum//n_lanes, reset_less=True)
                for i in range(n_lanes)]
        assert len(Cat(sr)) == len(words)
        ddr_data = Cat([sri[-2] for sri in sr], [sri[-1] for sri in sr])
        self.comb += [
            # assert one cycle ahead
            clk_stb.eq(~clk[0] & clk[-1]),
            # double period because of DDR
            self.stb.eq(i_frame == n_div*n_frame//2 - 1),

            self.crc.last.eq(checksum),
            self.crc.data.eq(ddr_data),
        ]
        miso = Signal()
        miso_sr = Signal(n_frame, reset_less=True)
        self.sync.rio_phy += [
            clk.eq(Cat(clk[-2:], clk)),
            If(clk[:2] == 0,  # TODO: tweak sampling
                miso_sr.eq(Cat(miso, miso_sr)),
            ),
            If(~self.stb,
                i_frame.eq(i_frame + 1),
            ),
            [sri[2:].eq(sri) for sri in sr],
            checksum.eq(self.crc.next),
            If(self.stb & clk_stb,
                i_frame.eq(0),
                # transpose
                Cat(sr).eq(Cat(words[mm::n_lanes] for mm in range(n_lanes))),
                checksum.eq(0),
                Array([self.dat_r[i*n_frame//2:(i + 1)*n_frame//2]
                    for i in range(1 << len(adr))])[adr].eq(miso_sr),
                adr.eq(adr + 1),
            ),
            If(i_frame == n_div*n_frame//2 - 2,
                ddr_data.eq(self.crc.next),
            ),
        ]

        clk_ddr = Signal()
        miso0 = Signal()
        self.specials += [
                DDROutput(clk[-1], clk[-2], clk_ddr, ClockSignal("rio_phy")),
                DifferentialOutput(clk_ddr, pins.clk, pins_n.clk),
                [(DDROutput(sri[-1], sri[-2], ddr, ClockSignal("rio_phy")),
                  DifferentialOutput(ddr, mp, mn))
                  for sri, ddr, mp, mn in zip(
                      sr, Signal(n_lanes), pins.mosi, pins_n.mosi)],
                DifferentialInput(pins.miso, pins_n.miso, miso0),
                MultiReg(miso0, miso, "rio_phy"),
        ]


class Fastino(Module):
    def __init__(self, pins, pins_n):
        self.rtlink = rtlink.Interface(
            rtlink.OInterface(data_width=16, address_width=6),
            rtlink.IInterface(data_width=32, timestamped=False))

        self.submodules.serializer = SerDes(pins, pins_n)

        self.sync.rtio += [
                If(self.rtlink.o.stb,
                    Array(self.serializer.dac + [
                        self.serializer.mask[:16],
                        self.serializer.mask[16:],
                        self.serializer.cfg[:16],
                        self.serializer.cfg[16:],
                        Signal()
                        ])[self.rtlink.o.address].eq(self.rtlink.o.data)
                ),
                self.rtlink.i.stb.eq(self.rtlink.o.stb &
                    (self.rtlink.o.address[4:] == 0b11)),
                self.rtlink.i.data.eq(Array([
                    self.serializer.dat_r[i*7:(i + 1)*7]
                    for i in range(1 << 4)])[
                        self.rtlink.o.address[:4]]),
        ]
