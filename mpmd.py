#!/usr/bin/env python3
from pprint import pprint as pp

import argparse
import collections
import glob
import json
import math
import os
import sys
import traceback

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
        quiet = int(ns.get('MPMDQUIET') or quiet)
        debug = int(ns.get('MPMDDEBUG') or debug)
    mp = Printer(quiet=quiet, debug=debug)
    mp.info("Use 'mp' to interact, eg", suffix=':')
    mp.info('>>> mp.home()', format=False)
    ipython.push({'mp': mp, 'mpmd': sys.modules[__name__]})


class Printer:
    """Monoprice Mini Delta controller."""

    Coordinates = collections.namedtuple('Coordinates', 'X,Y,Z')

    # Set of all possible IJ bed mesh probe indices.
    probes = { (I, J) for J in range(7) for I in range(7) if math.sqrt((I-3)**2 + (J-3)**2) <= 3 }

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

        if self.debug:
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
        return self.Coordinates(X, Y, Z)

    def level(self, I=3, J=3, F=3000, gauge=0.1, steps=3, choice=None, next=None):
        IJ  = (I, J)
        if IJ not in self.probes:
            self.warn(f'mesh offset at {IJ} cannot be leveled')
            return None, None, None

        if self.mesh[0][0] == 0:
            self.warn(f"missing expected mesh data, try 'G29 P2 V4'")
            return None, None, None

        reset = self.mesh[I][J]
        if reset < -0.5 or reset > 0.5:
            self.warn(f"unsafe mesh data '{reset}', try 'G29 P2 V4'")
            return None, None, None

        self.info(f'leveling mesh offset at {IJ}')
        self.G1(F=F)

        here = self.xyz
        safe = self.bed(I=I, J=J)
        zero = self.bed(I=I, J=J, Z=gauge)
        if here.X != safe.X or here.Y != safe.Y or here.Z > safe.Z:
            self.xyz, here = safe, self.xyz
        back = here
        usteps = steps
        probed = None
        breaks = ('q', 'quit', 'n', 'next') if next else ('q', 'quit')
        while probed is None or choice not in breaks:
            probed = self.mesh[I][J]
            offset = round(here.Z - zero.Z, 2)
            step = round(min(max(abs((here.Z - zero.Z) / usteps), 0.03), 10.0), 2)
            down = round(here.Z - step, 2)
            up = round(here.Z + step, 2)

            # TODO: Fine... use decimals.
            # Set choice=h on first pass and reset, else prompt user; no choice == last choice.
            notice = '[%+.2f] ' % reset if probed != reset else ''
            prompt = f'<<< %d @ %.2fmm / %s%+.2f / %.2fmm %+.2f [-+23456sudbzr%sq]? [%s] ' % (
                usteps, step, notice, probed, here.Z, offset, 'n' if next else '', choice)
            choice = 'h' if choice is None else input(prompt) or choice

            if choice in ('h', '?', 'help'):
                self.info(f'nozzle at {here.Z}mm'
                          f' with {usteps} steps @ {step}mm'
                          f' to reach {zero.Z}mm gauge')
                self.info(f'  help   show this message')
                self.info(f'  -/+    change step size')
                self.info(f'  2-6    change step count')
                self.info(f'  set    adjust by {"%+.2f" % offset}mm')
                self.info(f'  up     up to {up}mm')
                self.info(f'  down   down to {down}mm')
                self.info(f'  back   back to {back.Z}mm')
                self.info(f"  zero   move to {zero.Z}mm")
                self.info(f"  redo   restore {reset}mm and try again")
                if next:
                    self.info(f'  next   keep leveled {IJ} and start on {next}')
                self.info(f'  quit   keep leveled {IJ} and exit')
                continue

            if choice in ('+', '='):
                usteps = max(2, usteps - 1)
                self.info(f"steps set to {usteps} (increases size)")
                continue

            if choice in ('-',):
                usteps = min(6, usteps + 1)
                self.info(f"steps set to {usteps} (decreases size)")
                continue

            if choice in ('2', '3', '4', '5', '6'):
                usteps = int(choice)
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

            if choice in ('z', 'zero'):
                self.info(f"nozzle zeroed to {zero.Z}mm")
                self.G1(Z=zero.Z)
                back, here = here, self.xyz
                continue

            if choice in ('s', 'set'):
                self.info(f"{IJ} adjusted by {'%+.2f' % offset}mm")
                self.M421(I=I, J=J, Q=offset+0.001)
                self.M421(E=True)
                self.G1(Z=zero.Z)
                back = back._replace(Z=round(back.Z - offset, 2))
                here = self.xyz
                choice = None
                continue

            if choice in ('r', 'redo'):
                self.info(f"{IJ} reset to {reset}mm")
                self.M421(I=I, J=J, Z=reset+0.001)
                self.M421(E=True)
                self.xyz = self.bed(I=I, J=J)
                back, here = here, self.xyz
                usteps = steps
                choice = None
                continue

            if choice in ('x', 'xdebug', 'debug'):
                self.debug = int(self.debug + 1 if self.debug < 2 else 0)
                self.info(f"debug set to {self.debug}")
                continue

        self.xyz = self.bed(I=I, J=J)
        self.info(f'done leveling mesh offset {IJ}')
        return choice, reset, probed


def main():
    parser = argparse.ArgumentParser(description='Monoprice Mini Delta Leveler.')
    parser.add_argument('--home', help='home print head', action='store_true')
    parser.add_argument('--level', help='level mesh offsets', metavar='IJ', nargs='?', const=True)
    parser.add_argument('--gauge', help='gauge thickness', metavar='MM', default=0.1, type=float)
    parser.add_argument('--port', help='serial port name or path')
    parser.add_argument('--dryrun', '-n',  help='do not run gcode', action='store_true')
    parser.add_argument('--debug', '-d', help='increase verbosity', action='count', default=0)

    args = parser.parse_args()
    with Printer(dryrun=args.dryrun, debug=args.debug, port=args.port) as printer:
        if args.home:
            printer.home()

        if args.level:
            if args.level is True:
                probes = [*reversed(sorted(Printer.probes))]
            else:
                probes = [(int(args.level[0]), int(args.level[-1]))]

            changes = 0
            errors = 0
            choice = None
            while choice not in ('q', 'quit') and probes:
                probe = probes.pop()
                more = probes[-1] if probes else None
                try:
                    choice, old, new = printer.level(*probe, gauge=args.gauge, choice=choice, next=more)
                    if old != new:
                        printer.info(f"changed {probe} from '{old}' to '{new}'")
                        changes += 1
                except Exception as e:
                    printer.error(f"level({probe}) raised '{e}'", suffix=':')
                    traceback.print_exc()
                    errors += 1

            if errors:
                printer.warn('not writing to SD Card because {errors} errors raised.')
            elif not changes:
                printer.info('not writing to SD Card because bed mesh is unchanged')
            else:
                printer.info(f'writing {changes} bed mesh changes to SD Card')
                printer.M500()

            if args.home:
                printer.home()


if __name__ == '__main__':
    main()
