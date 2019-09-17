from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.io import DifferentialOutput, DifferentialInput, DDROutput
from misoc.cores.liteeth_mini.mac.crc import LiteEthMACCRCEngine

from artiq.gateware.rtio import rtlink


class SerDes(Module):
    def transpose(self, i, n):
        # i is n,m c-contiguous
        # o is m,n c-contiguous
        o = Signal.like(i)
        m = len(i)//n
        assert n*m == len(i)
        for nn in range(n):
            for mm in range(m):
                self.comb += o[mm*n + nn].eq(i[nn*m + mm])
        return o

    def __init__(self, pins, pins_n):
        n_bits = 16
        n_channels = 32
        n_div = 7
        assert n_div == 7
        n_frame = 14
        n_lanes = len(pins.mosi)

        self.dac = [Signal(n_bits, reset=i) for i in range(n_channels)]
        self.mask = Signal(n_channels, reset=(1 << n_channels) - 1)
        self.cfg = Signal(16, reset=0)
        adr = Signal(4)
        self.dat_r = Signal(n_frame*(1 << len(adr)))

        checksum = Signal(16)
        crc = LiteEthMACCRCEngine(
            data_width=2*n_lanes, width=len(checksum), polynom=0x1021)
        self.submodules += crc

        n_word = n_lanes*n_div
        body = Signal(n_frame*n_word - n_frame//2 - 1 - len(checksum))
        body_ = Cat(self.cfg, adr, self.dac, self.mask)
        assert len(body) == len(body_)
        self.comb += body.eq(body_)

        words = Signal(n_frame*n_word)
        words_ = []
        j = 0
        for i in range(n_frame):
            if i == 0:
                words_.append(C(1))
                words_.append(checksum)
            elif i <= n_frame//2:
                words_.append(C(0))
            k = n_word - (len(Cat(words_)) % n_word)
            words_.append(body[j:j + k])
            j += k
        words_ = Cat(words_)
        assert len(words_) == len(words)
        self.comb += words.eq(words_)

        self.stb = Signal()
        clk = Signal(n_div, reset=0b1100011)
        stb_clk = Signal()
        i_frame = Signal(max=n_frame//2)
        self.comb += [
            stb_clk.eq(~clk[0] & clk[-1]),
            self.stb.eq(stb_clk & (i_frame == n_frame//2 - 1)),
        ]
        sr = [Signal(n_frame*n_div, reset_less=True) for i in range(n_lanes)]
        miso = Signal()
        miso_sr = Signal(n_frame, reset_less=True)
        self.sync.rio_phy += [
            clk.eq(Cat(clk[-2:], clk)),
            [sri[2:].eq(sri) for sri in sr],
            If(~clk[0],
                miso_sr.eq(Cat(miso, miso_sr)),
            ),
            If(stb_clk,
                i_frame.eq(i_frame + 1),
            ),
            If(self.stb,
                i_frame.eq(0),
                Cat(sr).eq(self.transpose(words, n_frame*n_div)),
                adr.eq(adr + 1),
                Array([self.dat_r[i*n_frame:(i + 1)*n_frame]
                    for i in range(1 << len(adr))])[adr].eq(miso_sr),
            )
        ]

        clk_ddr = Signal()
        miso0 = Signal()
        self.specials += [
                DDROutput(clk[-1], clk[-2], clk_ddr, ClockSignal("rio_phy")),
                DifferentialOutput(clk_ddr, pins.clk, pins_n.clk),
                [(DDROutput(sri[-1], sri[-2], ddr, ClockSignal("rio_phy")),
                  DifferentialOutput(ddr, mp, mn))
                  for sri, ddr, mp, mn in zip(sr, Signal(n_lanes), pins.mosi, pins_n.mosi)],
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
                        self.serializer.cfg
                        ])[self.rtlink.o.address].eq(self.rtlink.o.data)
                )
                self.rtlink.i.stb.eq(self.rtlink.o.stb & (self.rtlink.o.address[2:5] == 0b111100)),
                self.rtlink.i.data.eq(Array([
                    self.serializer.dat_r[i*16:(i + 1)*16] for i in range(4)])[
                        self.rtlink.o.address[:2]]),
        ]
