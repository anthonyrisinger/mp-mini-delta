#!/usr/bin/env python3
from pprint import pprint as pp

import argparse
import glob
import json
import sys

import serial


def load_ipython_extension(ipython):
    """Run IPython shell -> %reload_ext mpmd"""
    mp = Printer(quiet=False)
    mp.info('Use `mp` to interact, eg', suffix=':')
    mp.info('>>> mp.home()', format=False)
    ipython.push({'mp': mp})


class Printer:
    """Monoprice Mini Delta controller."""

    def __init__(self, *, dryrun=False, quiet=False, pattern='/dev/cu.usbmodem*',
                 port=None, baudrate=115200, parity=serial.PARITY_NONE, **kwds):
        if not port and pattern:
            port = (*glob.glob(pattern), None)[0]
            if not port:
                raise TypeError(f"bad port pattern '{pattern}'")

        #TODO: mock dryrun connection.
        self.dryrun = dryrun
        self.quiet = quiet
        self.debug = False

        kwds.update(port=port, baudrate=baudrate, parity=parity)
        infos = ('Printer(', f'dryrun={dryrun}', *(f'{k}={v}' for k,v in kwds.items()), ')')
        self.info(' '.join(infos), format=False)

        self.kwds = kwds
        self.conn = serial.Serial(**self.kwds)
        if not self.conn.is_open:
            self.conn.open()
        self.conn.setRTS(False)
        self.buffer = self.conn.read_all()
        self._mesh = None

    def __enter__(self):
        """Refresh connection."""
        if self.conn is not None:
            self.conn.close()
        self.conn = serial.Serial(**self.kwds)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close connection."""
        self.conn.close()
        self.conn = None

    def __getattr__(self, name):
        """Forward simple gcodes [G1, M500, ...] to write()."""
        upper = name.upper()
        if upper[0] in ('G', 'M') and upper[1:].isdigit():
            def wrapper(**kwds):
                if self.conn is not None:
                    return self.write(upper, **kwds)
            return wrapper
        raise AttributeError(name)

    @property
    def mesh(self):
        """Fetch and cache bed offsets."""
        if self._mesh is None:
            lines = self.write('M421').strip().split('\n')[:-1]
            self._mesh = [[0.0]*7 for n in range(7)]
            for J, line in enumerate(lines):
                offsets = line.strip().split(' ')
                for I, offset in enumerate(offsets):
                    self._mesh[I][J] = float(offset)
        return self._mesh

    @property
    def xyz(self):
        """Fetch current coords."""
        line = self.write('M114').split('\n')[0]
        xyz = tuple(float(dim[2:]) for dim in line.split(' ', 3)[:3])
        return xyz

    @xyz.setter
    def xyz(self, xyz):
        """Move to XYZ coords."""
        self.move(xyz)

    def info(self, line,  prefix=' ## ', suffix='.', format=str.capitalize, **kwds):
        if not self.quiet:
            self.log(line, prefix=prefix, suffix=suffix, format=format, **kwds)

    def warn(self, line,  prefix=' ?? ', suffix='.', format=str.capitalize, **kwds):
        self.log(line, prefix=prefix, suffix=suffix, format=format, **kwds)

    def error(self, line, prefix=' !! ', suffix='!', format=str.capitalize, **kwds):
        self.log(line, prefix=prefix, suffix=suffix, format=format, **kwds)

    def log(self, line, end='\n', prefix='', suffix='', format=None):
        if format is None:
            print(f'{line}', end=end, file=sys.stderr)
        elif format is False:
            print(f'{prefix}{line}', end=end, file=sys.stderr)
        else:
            print(f'{prefix}{format(line)}{suffix}', end=end, file=sys.stderr)

    def write(self, op, **pvs):
        """Write gcode to connection."""
        gcode = self.gcode(op, **pvs)
        if not gcode:
            call = ', '.join((op, *(f"{p}={v}" for p,v in pvs.items())))
            self.error(f'gcode({call}) -> ()', format=False)
            return None

        if self.debug:
            self.log(f'>>> {b" ".join(gcode[:-1]).decode()}', format=False)

        if self.dryrun:
            return None

        for code in gcode:
            self.conn.write(code)

        read = self.read(caller=(op, pvs, gcode))
        if self.debug:
            self.log(read, end='', format=False)
        return read

    def read(self, lines=None, caller=None, until=None):
        """Read bytes from connection."""
        if caller:
            read = b''
            while read != b'ok\n' and b'\nok\n' not in read:
                read += self.conn.read_until(b'ok\n')
        elif until:
            read = self.conn.read_until(until)
        elif lines:
            read = b''.join(self.conn.readline() for line in range(lines))
        else:
            read = self.conn.read_all()
        return read.decode()

    def gconv(self, op, param, value):
        """Convert (param, value) pair to string."""
        if value is True:
            return f'{param}'
        if param in ('X', 'Y', 'Z', 'Q'):
            return f'{param}{format(value, ".3f")}'
        if param in ('I', 'J'):
            return f'{param}{format(value, "d")}'
        return f'{param}{value}'

    def gcode(self, op, **pvs):
        """Convert (op, params) to gcode IO list."""
        gcode = (op.encode(), *(self.gconv(op, *pv).encode() for pv in pvs.items()), b'\n')
        return gcode

    def home(self):
        """Home the printer."""
        self.move()

    def move(self, *args, **kwds):
        """Move to home or XYZ[EF] coords."""
        if not kwds and (not args or args[0] is None):
            self.G28()
            return self

        if len(args) == 1 and hasattr(args[0], '__len__'):
            args = tuple(args[0])
        kwds.update(zip(('X', 'Y', 'Z', 'F', 'E'), args))

        for (dim, dmin, dmax) in (
                ('X', -60.0,  60.0),
                ('Y', -60.0,  60.0),
                ('Z',  -0.5, 120.0)
                ):
            if dim in kwds:
                assert kwds[dim] >= dmin
                assert kwds[dim] <= dmax

        self.G1(**kwds)
        return self

    def bed(self, I=3, J=3, X=0, Y=0, Z=15):
        """Compute XYZ coords for IJ bed mesh index."""
        assert I >= 0 and I <= 6
        assert J >= 0 and J <= 6
        X = (I - 3) * 15.0 + X
        Y = (J - 3) * 15.0 + Y
        return (X, Y, Z)


def main():
    parser = argparse.ArgumentParser(description='Leveler.')
    parser.add_argument('--dryrun', help='dryrun', action='store_true')
    parser.add_argument('--quiet', help='quiet', action='store_true')
    parser.add_argument('--port', help='serial port')
    args = parser.parse_args()

    with Printer(dryrun=args.dryrun, quiet=args.quiet, port=args.port) as p:
        p.M421()


if __name__ == '__main__':
    main()
