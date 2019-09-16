from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.io import DifferentialOutput, DifferentialInput

from artiq.gateware.rtio import rtlink


class SerDes(Module):
    def __init__(self, pins, pins_n):
        n_bits = 16
        n_channels = 32
        n_div = 7
        assert n_div == 7
        n_frame = 14
        n_lanes = len(pins.mosi)

        self.dac = [Signal(n_bits) for i in range(n_channels)]
        self.mask = Signal(n_channels)
        self.cfg = Signal(16)
        adr = Signal(4)
        self.dat_r = Signal(n_frame*(1 << len(adr)))
        crc = Signal(16)

        self.stb = Signal()
        clk = Signal(n_div, reset=0b1100011)
        i_frame = Signal(max=n_frame)
        n_word = n_lanes*n_div
        body = Signal(n_frame*n_word - n_frame//2 - 1 - len(crc))
        body_ = Cat(self.cfg, adr, self.dac, self.mask)
        assert len(body) == len(body_)
        words = Signal(n_frame*n_word)
        words_ = []
        j = 0
        for i in range(n_frame):
            if i == 0:
                words_.append(C(1))
                words_.append(crc)
            elif i <= n_frame//2:
                words_.append(C(0))
            k = n_word - (len(Cat(words_)) % n_word)
            words_.append(body[j:j + k])
            j += k
        words_ = Cat(words_)
        assert len(words_) == len(words)
        stb_clk = Signal()
        self.comb += [
            stb_clk.eq(clk[1:3] == 0b10),
            self.stb.eq(stb_clk & (i_frame == n_frame - 1)),
            words.eq(words_),
            body.eq(body_),
        ]
        sr = [Signal(n_frame*n_div, reset_less=True) for i in range(n_lanes)]
        miso = Signal()
        miso_sr = Signal(n_div, reset_less=True)
        self.sync.rio_phy += [
            clk.eq(Cat(clk[-1], clk)),
            [sri[1:].eq(sri) for sri in sr],
            If(stb_clk,
                i_frame.eq(i_frame + 1),
                miso_sr.eq(Cat(miso, miso_sr)),
            ),
            If(self.stb,
                i_frame.eq(0),
                Cat(sr).eq(words),
                adr.eq(adr + 1),
                Array([self.dat_r[i*n_frame:(i + 1)*n_frame]
                    for i in range(1 << len(adr))])[adr].eq(miso_sr),
            )
        ]

        self.specials += [
                DifferentialOutput(clk[-1], pins.clk, pins_n.clk),
                [DifferentialOutput(sri[-1], mp, mn) for sri, mp, mn in zip(sr,
                    pins.mosi, pins_n.mosi)],
                DifferentialInput(pins.miso, pins_n.miso, miso),
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
                        self.serializer.mask[0],
                        self.serializer.mask[1],
                        self.serializer.cfg
                        ])[self.rtlink.o.address].eq(self.rtlink.o.data)
                )
        ]
        #self.specials += MultiReg(roi_boundary, target, "cl")
