#!/usr/bin/env python3
"""Read-only ownership checks for Gazebo/RViz windows on a shared X11 display.

Xlib runs only in the bounded child invoked by inspect_windows_bounded. No
screenshots, input injection, focus changes or window rearrangement are used.
Window titles alone never establish ownership: the unique container hostname
and a PID in the launched application's process group are both mandatory.
"""

import argparse
import ctypes
import ctypes.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys


MAX_WINDOWS = 4096
MAX_DEPTH = 8
MAX_TEXT_BYTES = 4096
APPLICATIONS = frozenset(('gazebo', 'rviz'))


# Layouts follow upstream include/X11/Xlib.h, including native XID/long width.
# https://github.com/mirror/libX11/blob/master/include/X11/Xlib.h
class XErrorEvent(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('display', ctypes.c_void_p),
                ('resourceid', ctypes.c_ulong), ('serial', ctypes.c_ulong),
                ('error_code', ctypes.c_ubyte), ('request_code', ctypes.c_ubyte),
                ('minor_code', ctypes.c_ubyte)]


XErrorHandler = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(XErrorEvent))


class WindowAttributes(ctypes.Structure):
    _fields_ = [
        ('x', ctypes.c_int), ('y', ctypes.c_int), ('width', ctypes.c_int),
        ('height', ctypes.c_int), ('border_width', ctypes.c_int), ('depth', ctypes.c_int),
        ('visual', ctypes.c_void_p), ('root', ctypes.c_ulong), ('class_', ctypes.c_int),
        ('bit_gravity', ctypes.c_int), ('win_gravity', ctypes.c_int), ('backing_store', ctypes.c_int),
        ('backing_planes', ctypes.c_ulong), ('backing_pixel', ctypes.c_ulong),
        ('save_under', ctypes.c_int), ('colormap', ctypes.c_ulong),
        ('map_installed', ctypes.c_int), ('map_state', ctypes.c_int),
        ('all_event_masks', ctypes.c_long), ('your_event_mask', ctypes.c_long),
        ('do_not_propagate_mask', ctypes.c_long), ('override_redirect', ctypes.c_int),
        ('screen', ctypes.c_void_p),
    ]


class X11Backend:
    """Small query-only Xlib wrapper. Call exclusively from a bounded process."""

    def __init__(self, display):
        library = ctypes.util.find_library('X11')
        if not library:
            raise RuntimeError('libX11 unavailable')
        self.x = ctypes.CDLL(library)
        self.display = None
        self._errors = []
        self._handler = None
        self._previous_handler = None
        pointer, window, atom = ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XOpenDisplay.restype = pointer
        self.x.XCloseDisplay.argtypes = [pointer]
        self.x.XCloseDisplay.restype = ctypes.c_int
        self.x.XDefaultRootWindow.argtypes = [pointer]
        self.x.XDefaultRootWindow.restype = window
        self.x.XInternAtom.argtypes = [pointer, ctypes.c_char_p, ctypes.c_int]
        self.x.XInternAtom.restype = atom
        self.x.XGetWindowProperty.argtypes = [
            pointer, window, atom, ctypes.c_long, ctypes.c_long, ctypes.c_int, atom,
            ctypes.POINTER(atom), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
        ]
        self.x.XGetWindowProperty.restype = ctypes.c_int
        self.x.XQueryTree.argtypes = [pointer, window, ctypes.POINTER(window),
                                    ctypes.POINTER(window), ctypes.POINTER(ctypes.POINTER(window)),
                                    ctypes.POINTER(ctypes.c_uint)]
        self.x.XQueryTree.restype = ctypes.c_int
        self.x.XGetWindowAttributes.argtypes = [pointer, window, ctypes.POINTER(WindowAttributes)]
        self.x.XGetWindowAttributes.restype = ctypes.c_int
        self.x.XFree.argtypes = [pointer]
        self.x.XFree.restype = ctypes.c_int
        self.x.XSetErrorHandler.argtypes = [ctypes.c_void_p]
        self.x.XSetErrorHandler.restype = ctypes.c_void_p
        self.x.XSync.argtypes = [pointer, ctypes.c_int]
        self.x.XSync.restype = ctypes.c_int
        self.display = self.x.XOpenDisplay(display.encode('ascii'))
        if not self.display:
            raise RuntimeError('Authorized desktop display unavailable')
        # Scoped to this short-lived query child. A host client can disappear
        # between enumeration and a query; BadWindow invalidates that candidate,
        # not the entire simulator. All other X protocol errors still fail.
        self._handler = XErrorHandler(self._capture_error)
        self._previous_handler = self.x.XSetErrorHandler(ctypes.cast(self._handler, ctypes.c_void_p))
        self.root = int(self.x.XDefaultRootWindow(self.display))

    def _capture_error(self, _display, event):
        # Xlib forbids protocol requests inside its error handler.
        # Never raise through a ctypes callback; propagate after the query.
        self._errors.append(int(event.contents.error_code))
        return 0

    def _query_had_bad_window(self):
        errors, self._errors = self._errors, []
        unexpected = [code for code in errors if code != 3]  # BadWindow = 3.
        if unexpected:
            raise RuntimeError('X11 query protocol error: ' + ','.join(map(str, unexpected)))
        return bool(errors)

    def close(self):
        if self.display:
            try:
                # Deliver any pending protocol errors before restoring Xlib's
                # previous handler. A lost connection remains a fatal child I/O error.
                self.x.XSync(self.display, 0)
                self._query_had_bad_window()
            finally:
                try:
                    self.x.XCloseDisplay(self.display)
                    self._query_had_bad_window()
                finally:
                    self.display = None
                    self.x.XSetErrorHandler(self._previous_handler)
                    self._handler = None

    def property(self, window, name, expected_format, expected_type, limit):
        atom = self.x.XInternAtom(self.display, name.encode('ascii'), 1)
        self._query_had_bad_window()
        if not atom:
            return None
        type_atom = self.x.XInternAtom(self.display, expected_type.encode('ascii'), 1)
        self._query_had_bad_window()
        if not type_atom:
            return None
        actual_type, actual_format = ctypes.c_ulong(), ctypes.c_int()
        count, remaining = ctypes.c_ulong(), ctypes.c_ulong()
        data = ctypes.POINTER(ctypes.c_ubyte)()
        # X protocol length is in 32-bit units, even on a 64-bit host.
        words = (limit + 3) // 4 if expected_format == 8 else limit
        status = self.x.XGetWindowProperty(
            self.display, window, atom, 0, words, 0, type_atom,
            ctypes.byref(actual_type), ctypes.byref(actual_format), ctypes.byref(count),
            ctypes.byref(remaining), ctypes.byref(data))
        try:
            bad_window = self._query_had_bad_window()
            if (bad_window or status != 0 or actual_type.value != type_atom
                    or actual_format.value != expected_format or remaining.value or count.value > limit):
                return None
            if not count.value:
                return b'' if expected_format == 8 else []
            if not data:
                return None
            if expected_format == 8:
                return ctypes.string_at(data, count.value)
            # Xlib expands 32-bit protocol values to native unsigned long slots.
            values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))
            return [int(values[i]) for i in range(count.value)]
        finally:
            if data:
                self.x.XFree(data)

    def client_list(self):
        return self.property(self.root, '_NET_CLIENT_LIST', 32, 'WINDOW', MAX_WINDOWS)

    def children(self, window):
        root, parent, count = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_uint()
        children = ctypes.POINTER(ctypes.c_ulong)()
        success = self.x.XQueryTree(self.display, window, ctypes.byref(root), ctypes.byref(parent),
                                   ctypes.byref(children), ctypes.byref(count))
        try:
            bad_window = self._query_had_bad_window()
            if bad_window or not success:
                return []
            if count.value > MAX_WINDOWS:
                raise RuntimeError('Desktop window tree exceeds observation bound')
            return [int(children[i]) for i in range(count.value)]
        finally:
            if children:
                self.x.XFree(children)

    def machine(self, window):
        value = self.property(window, 'WM_CLIENT_MACHINE', 8, 'STRING', MAX_TEXT_BYTES)
        return value.rstrip(b'\0').decode('ascii', errors='replace') if value is not None else None

    def pid(self, window):
        values = self.property(window, '_NET_WM_PID', 32, 'CARDINAL', 1)
        return values[0] if values and len(values) == 1 else None

    def details(self, window):
        title = self.property(window, '_NET_WM_NAME', 8, 'UTF8_STRING', MAX_TEXT_BYTES)
        if title is None:
            title = self.property(window, 'WM_NAME', 8, 'STRING', MAX_TEXT_BYTES)
        classes = self.property(window, 'WM_CLASS', 8, 'STRING', MAX_TEXT_BYTES)
        transient = self.property(window, 'WM_TRANSIENT_FOR', 32, 'WINDOW', 1)
        state = self.property(window, 'WM_STATE', 32, 'WM_STATE', 2)
        attributes = WindowAttributes()
        success = self.x.XGetWindowAttributes(self.display, window, ctypes.byref(attributes))
        if self._query_had_bad_window() or not success:
            return None
        return {
            'title': (title or b'').rstrip(b'\0').decode('utf-8', errors='replace'),
            'classes': [part.decode('utf-8', errors='replace') for part in (classes or b'').split(b'\0') if part],
            'mapped': attributes.map_state == 2,
            'iconic': bool(state and state[0] == 3),
            'managed': bool(state and state[0] in (1, 3)),  # NormalState / IconicState.
            'transient': bool(transient and transient[0]),
        }


def validate_request(display, expected_machine, process_groups):
    if not isinstance(display, str) or not re.fullmatch(r':[0-9]{1,5}(?:\.[0-9]{1,2})?', display):
        raise ValueError('Explicit local X11 display required')
    if (not isinstance(expected_machine, str)
            or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9-]{0,62}', expected_machine)):
        raise ValueError('Unique container hostname required')
    if (not isinstance(process_groups, dict) or set(process_groups) != APPLICATIONS
            or any(type(pid) is not int or pid <= 1 for pid in process_groups.values())
            or len(set(process_groups.values())) != 2):
        raise ValueError('Two distinct owned GUI process groups required')


def candidate_windows(backend):
    clients = backend.client_list()
    if clients is not None:
        if len(clients) > MAX_WINDOWS:
            raise RuntimeError('Desktop client list exceeds observation bound')
        return list(dict.fromkeys(clients))
    result, seen = [], {backend.root}
    queue = [(backend.root, 0)]
    for window, depth in queue:
        children = backend.children(window)
        for child in children:
            if child in seen:
                continue
            seen.add(child)
            if len(seen) > MAX_WINDOWS:
                raise RuntimeError('Desktop window tree exceeds observation bound')
            result.append(child)
            if depth + 1 < MAX_DEPTH:
                queue.append((child, depth + 1))
    return result


def inspect_windows(display, expected_machine, process_groups, *, backend_factory=X11Backend, getpgid=os.getpgid):
    """Return only owned main windows; minimize is allowed, ownership is mandatory."""
    validate_request(display, expected_machine, process_groups)
    backend = backend_factory(display)
    try:
        found = {}
        groups = {group: application for application, group in process_groups.items()}
        for window in candidate_windows(backend):
            if backend.machine(window) != expected_machine:
                continue
            pid = backend.pid(window)
            if type(pid) is not int or pid <= 1:
                continue
            try:
                application = groups.get(getpgid(pid))
            except (ProcessLookupError, PermissionError):
                continue
            if application is None:
                continue
            details = backend.details(window)
            if not details or details['transient'] or not (details['mapped'] or details['managed']):
                continue
            tokens = [details['title'].strip().lower()] + [name.lower() for name in details['classes']]
            name_matches = (any(name in ('gazebo', 'gzclient') for name in tokens) if application == 'gazebo'
                            else any('rviz' in name for name in tokens))
            if not name_matches:
                continue
            record = {'window': window, 'pid': pid, 'process_group': process_groups[application],
                      'title': details['title'], 'mapped': details['mapped'], 'iconic': details['iconic'],
                      'managed': details['managed'],
                      'machine': expected_machine}
            # A mapped main window is preferable to a minimized secondary window.
            if application not in found or (record['mapped'] and not found[application]['mapped']):
                found[application] = record
        return found
    finally:
        backend.close()


def inspect_windows_bounded(display, expected_machine, process_groups, *, runner=subprocess.run):
    validate_request(display, expected_machine, process_groups)
    result = runner([
        sys.executable, str(Path(__file__).resolve()), '--display', display, '--machine', expected_machine,
        '--gazebo-pgid', str(process_groups['gazebo']), '--rviz-pgid', str(process_groups['rviz']),
    ], timeout=2, check=True, capture_output=True, text=True)
    windows = json.loads(result.stdout)
    if not isinstance(windows, dict) or set(windows) - APPLICATIONS:
        raise ValueError('Invalid owned desktop window result')
    for application, record in windows.items():
        if (not isinstance(record, dict) or record.get('machine') != expected_machine
                or record.get('process_group') != process_groups[application]
                or type(record.get('window')) is not int or record['window'] <= 0
                or type(record.get('pid')) is not int or record['pid'] <= 1
                or type(record.get('mapped')) is not bool or type(record.get('iconic')) is not bool
                or type(record.get('managed')) is not bool
                or not (record['mapped'] or record['managed']) or not isinstance(record.get('title'), str)):
            raise ValueError('Invalid owned desktop window identity')
    return windows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--display', required=True)
    parser.add_argument('--machine', required=True)
    parser.add_argument('--gazebo-pgid', type=int, required=True)
    parser.add_argument('--rviz-pgid', type=int, required=True)
    args = parser.parse_args(argv)
    result = inspect_windows(args.display, args.machine, {'gazebo': args.gazebo_pgid, 'rviz': args.rviz_pgid})
    print(json.dumps(result, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
