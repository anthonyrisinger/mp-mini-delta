#!/usr/bin/env python3
from pprint import pprint as pp

import argparse
import collections
import glob
import json
import sys
import os

import serial


GCODES = {
    'G1': 'linear move',
    'G90': 'absolute positioning',
    'G21': 'millimeter units',
    'M114': 'get current position',
    'M421': 'set mesh value',
    'M500': 'save settings',
}


def load_ipython_extension(ipython, quiet=False, debug=False):
    """Run IPython shell -> %reload_ext mpmd"""
    for ns in (os.environ, ipython.user_ns):
        quiet = int(ns.get('MPMDQUIET', quiet))
        debug = int(ns.get('MPMDDEBUG', debug))
    mp = Printer(quiet=quiet, debug=debug)
    mp.info("Use 'mp' to interact, eg", suffix=':')
    mp.info('>>> mp.home()', format=False)
    ipython.push({'mp': mp})


class Printer:
    """Monoprice Mini Delta controller."""

    Coordinates = collections.namedtuple('Coordinates', 'X,Y,Z')

    def __init__(self, *, dryrun=False, quiet=False, debug=False, pattern='/dev/cu.usbmodem*',
                 port=None, baudrate=115200, parity=serial.PARITY_NONE, **kwds):
        if not port and pattern:
            port = (*glob.glob(pattern), None)[0]
            if not port:
                raise TypeError(f"bad port pattern '{pattern}'")

        kwds.update(port=port, baudrate=baudrate, parity=parity)
        self.kwds = kwds
        #TODO: mock dryrun connection.
        self.dryrun = dryrun
        self.quiet = int(quiet)
        self.debug = int(debug)
        #TODO: _connection method.
        self.conn = serial.Serial(**self.kwds)
        if not self.conn.is_open:
            self.conn.open()
        self.conn.setRTS(False)
        self._mesh = None
        self._homed = False
        self._purged = []
        self._purge_conn()
        self._init_gcode()

        info1 = ('Printer:', *(f'{k}={v}' for k,v in kwds.items()))
        info2 = f'         dryrun={self.dryrun} quiet={self.quiet} debug={self.debug}'
        self.info(' '.join(info1), format=False)
        self.info(info2, format=False)

    def _purge_conn(self):
        if self.conn.in_waiting:
            self._purged.append(self.conn.read_all())
            self.error(f'purged {len(self._purged[-1])} bytes')

    def _init_gcode(self):
        # Absolute positioning.
        self.write('G90')
        # Metric units.
        self.write('G21')
        # Default movement speed.
        self.write('G1', F=3000)
        # Detect if already homed.
        self._homed = sum(self.xyz) != 0

    def G28(self, **kwds):
        read = self.write('G28', **kwds)
        self._homed = True
        return read

    def M421(self, **kwds):
        read = self.write('M421', **kwds)
        if not kwds or kwds == {'E': True}:
            lines = read.strip().split('\n')[:-1]
            mesh = [[0.0]*7 for n in range(7)]
            for J, line in enumerate(lines):
                offsets = line.strip().split(' ')
                for I, offset in enumerate(offsets):
                    mesh[I][J] = float(offset)
            self._mesh = mesh
        return read

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
        if name[:1] in ('G', 'M') and name[1:].isdigit():
            def wrapper(**kwds):
                if self.conn is not None:
                    return self.write(name, **kwds)
            return wrapper
        raise AttributeError(name)

    @property
    def mesh(self):
        """Fetch and cache bed offsets."""
        if self._mesh is None:
            self.M421()
        return self._mesh

    @property
    def xyz(self):
        """Fetch current coords."""
        line = self.write('M114').split('\n')[0]
        xyz = self.Coordinates(*(float(dim[2:]) for dim in line.split(' ', 3)[:3]))
        return xyz

    @xyz.setter
    def xyz(self, xyz):
        """Move to XYZ coords."""
        self.move(xyz)

    def info(self, *lines, prefix=' ## ', suffix='.', format=True, **kwds):
        if not self.quiet:
            self.log(*lines, prefix=prefix, suffix=suffix, format=format, **kwds)

    def warn(self, *lines, prefix=' ?? ', suffix='.', format=True, **kwds):
        self.log(*lines, prefix=prefix, suffix=suffix, format=format, **kwds)

    def error(self, *lines, prefix=' !! ', suffix='!', format=True, **kwds):
        self.log(*lines, prefix=prefix, suffix=suffix, format=format, **kwds)

    def log(self, *lines, end='\n', prefix='', suffix='', format=None):
        for line in lines:
            if format is True:
                line = prefix + line[:1].upper() + line[1:] + suffix
            elif format is False:
                line = prefix + line
            elif format is not None:
                line = prefix + format(line) + suffix
            print(line, end=end, file=sys.stderr)

    def write(self, op, *, comment=None, **pvs):
        """Write gcode to connection."""
        if comment is None:
            comment = GCODES.get(op)
        gcode = self.gcode(op, comment=comment, **pvs)
        if not gcode:
            call = ', '.join((op, *(f"{p}={v}" for p,v in pvs.items())))
            self.error(f'gcode({call}) -> ()', format=False)
            return None

        if self.debug > 0:
            self.log(f'>>>> {b" ".join(gcode).decode().rstrip()}', format=None)

        if self.dryrun:
            return None

        self._purge_conn()
        for code in gcode:
            self.conn.write(code)

        read = self.read(caller=(op, pvs, gcode))
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

        read = read.decode()
        if self.debug > 1:
            self.log(read, end='', format=None)

        return read

    def gconv(self, op, param, value):
        """Convert (param, value) pair to string."""
        if value is True:
            return f'{param}'
        if param in ('X', 'Y', 'Z', 'Q'):
            return f'{param}{format(value, ".3f")}'
        if param in ('I', 'J'):
            return f'{param}{format(value, "d")}'
        return f'{param}{value}'

    def gcode(self, op, *, comment=None, **pvs):
        """Convert (op, params) to gcode IO list."""
        end = (b'\n',) if comment is None else (b';', comment.encode(), b'\n')
        gcode = (op.encode(), *(self.gconv(op, *pv).encode() for pv in pvs.items()), *end)
        return gcode

    def home(self):
        """Home the printer."""
        self.move()

    def move(self, *args, **kwds):
        """Move to home or XYZ[EF] coords."""
        if not kwds and (not args or args[0] is None):
            self.G28()
            return self

        if not self._homed:
            self.G28()

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

    def bed(self, I=3, J=3, X=0, Y=0, Z=10):
        """Compute XYZ coords for IJ bed mesh index."""
        assert I >= 0 and I <= 6
        assert J >= 0 and J <= 6
        X = (I - 3) * 15.0 + X
        Y = (J - 3) * 15.0 + Y
        return (X, Y, Z)

    def level(self, I=3, J=3, F=3000, gauge=0.1, steps=3):
        if I < 0 or I > 6 or J < 0 or J > 6:
            self.warn(f'mesh offset at ({I},{J}) out of bounds')
            return self

        if abs(3 - I) + abs(3 - J) > 3:
            self.warn(f'mesh offset at ({I},{J}) cannot be leveled')
            return self

        mesh = self.mesh
        if mesh[0][0] == 0:
            self.warn(f"missing expected mesh data, try 'G29 P2 V4'")
            return self

        reset = mesh[I][J]
        if reset < -0.5 or reset > 0.5:
            self.warn(f"unsafe mesh data '{reset}', try 'G29 P2 V4'")
            return self

        self.info(f'leveling mesh offset at ({I},{J})')
        self.move(self.bed(I=I, J=J), F=F)

        here = self.xyz
        back = here
        usteps = steps
        choice = None
        while choice != 'q':
            offset = round(here.Z - gauge, 3)
            step = round(min(max(abs((here.Z - gauge) / usteps), 0.01), 10.0), 3)
            down = round(here.Z - step, 3)
            up = round(here.Z + step, 3)

            # Set choice=h on first pass and reset, else prompt user; no choice == last choice.
            choice = 'h' if choice is None else input(
                f'<<< [h]elp [u]p [d]own [b]ack [s]et [r]eset [q]uit? {usteps}/{step}/{here.Z}mm '
            ) or choice

            if choice in ('h', '?', 'help'):
                self.info(f'nozzle at {here.Z}mm, steps is {usteps} @ {step}mm, gauge is {gauge}mm')
                self.info(f'  +/-    change steps')
                self.info(f'  up     up to {up}mm')
                self.info(f'  down   down to {down}mm')
                self.info(f'  back   back to {back.Z}mm')
                self.info(f'  set    set to {offset}mm')
                self.info(f"  redo   redo ({I},{J}) - reset {reset}mm")
                self.info(f'  quit   quit ({I},{J}) - keep {self.mesh[I][J]}mm')
                self.info(f'  help   show this message')
                continue

            if choice in ('+', '='):
                usteps = min(6, usteps + 1)
                self.info(f"steps set to {usteps}")
                continue

            if choice in ('-',):
                usteps = max(2, usteps - 1)
                self.info(f"steps set to {usteps}")
                continue

            if choice in ('u', 'up'):
                self.info(f"nozzle up to {up}mm")
                self.G1(Z=up)
                back, here = here, self.xyz
                continue

            if choice in ('d', 'down'):
                self.info(f"nozzle down to {down}mm")
                self.G1(Z=down)
                back, here = here, self.xyz
                continue

            if choice in ('b', 'back'):
                self.info(f"nozzle back to {back.Z}mm")
                self.G1(Z=back.Z)
                back, here = here, self.xyz
                continue

            if choice in ('s', 'set'):
                self.info(f"mesh set to {offset}mm")
                self.M421(I=I, J=J, Z=offset+0.001)
                self.M421(E=True)
                choice = None
                continue

            if choice in ('r', 'redo'):
                self.info(f"mesh reset to {reset}mm")
                self.M421(I=I, J=J, Z=reset+0.001)
                self.M421(E=True)
                self.xyz = self.bed(I=I, J=J)
                back, here = here, self.xyz
                usteps = steps
                choice = None
                continue

            if choice in ('x', 'xdebug', 'debug'):
                self.debug = int(choice == 'debug' or not self.debug)
                self.info(f"debug set to {self.debug}")
                continue

        self.xyz = self.bed(I=I, J=J)
        self.info(f'done leveling bed mesh offset ({I},{J})')


def main():
    parser = argparse.ArgumentParser(description='Monoprice Mini Delta Leveler.')
    parser.add_argument('--home', help='home print head', action='store_true')
    parser.add_argument('--level', help='level mesh offset', metavar='IJ', nargs='?', const='33')
    parser.add_argument('--gauge', help='gauge thickness', metavar='MM', default=0.1, type=float)
    parser.add_argument('--port', help='serial port name or path')
    parser.add_argument('--dryrun', '-n',  help='do not run gcode', action='store_true')
    parser.add_argument('--debug', '-d', help='increase verbosity', action='count', default=0)

    args = parser.parse_args()
    with Printer(dryrun=args.dryrun, debug=args.debug, port=args.port) as printer:
        if args.home:
            printer.home()
        if args.level:
            I, J = int(args.level[0]), int(args.level[-1])
            try:
                printer.level(I, J, gauge=args.gauge)
            except:
                raise
            else:
                printer.M500()


if __name__ == '__main__':
    main()
