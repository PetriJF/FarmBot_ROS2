"""Helper for mapping the high level commands to appropriate sequences/calls."""
from dataclasses import dataclass, field
from typing import Callable

from farmbot_controllers.sequences.calibration import calibrate_axes
from farmbot_controllers.sequences.find_home import find_home
from farmbot_controllers.sequences.single_call import single_call

# Peripheral pins live here, in the dispatch, and never reach the operator.
WATER_PUMP_PIN = 8
VACUUM_PIN = 9
LED_PIN = 7
PERIPHERAL_4_PIN = 10
PERIPHERAL_5_PIN = 12

# Sense pins read by the check commands.
TOOL_PIN = 63
SOIL_PIN = 59


def convert(words, types):
    """Cast each word with its converter."""
    if len(words) != len(types):
        raise ValueError(f'expected {len(types)} argument(s), got {len(words)}')
    return [cast(word) for cast, word in zip(types, words)]


def axes(words):
    """Read an optional axis letter X/Y/Z as a selection."""
    letter = words[0] if words else ''
    if letter not in ('', 'X', 'Y', 'Z'):
        raise ValueError(f"expected an axis X/Y/Z, got '{letter}'")
    return letter in ('', 'X'), letter in ('', 'Y'), letter in ('', 'Z')


def peripheral(pin, on):
    """Build the fixed keyword args for switching a peripheral pin on or off."""
    return {'pin': pin, 'value': int(on), 'pin_mode': False, 'pulse': False}


@dataclass
class Call:
    """A command that runs one client call: <module>.<method>(words, fixed, on_done)."""

    name: str                                    # task name shown in status
    module: str                                  # 'movement' / 'devices' / ...
    method: str                                  # e.g. 'move_gantry_abs'
    types: tuple = ()                            # cast the words positionally (the common case)
    fixed: dict = field(default_factory=dict)    # hidden constants (e.g. the pin)
    parse: Callable = None                       # custom words -> args, instead of `types`

    def build(self, words):
        """Turn the command's words into a one-step task around the client call."""
        args = self.parse(words) if self.parse else convert(words, self.types)

        def invoke(hardware, done=None):
            client = getattr(hardware, self.module)          # e.g. hardware.movement
            getattr(client, self.method)(*args, **self.fixed, on_done=done)

        return single_call(self.name, invoke)


@dataclass
class Sequence:
    """A command that expands into a multi-step sequence via a builder function."""

    builder: Callable                            # e.g. calibrate_axes
    parse: Callable = None                       # words -> the builder's arguments

    def build(self, words):
        """Parse the words to form the sequence."""
        args = self.parse(words) if self.parse else ()
        return self.builder(*args)


@dataclass
class ComplexSequence:
    """A command that needs async data before it can build its steps."""

    module: str                                  # 'movement' / 'devices' / ...
    method: str                                  # e.g. 'move_gantry_abs'
    types: tuple = ()                            # cast the words positionally (the common case)
    fixed: dict = field(default_factory=dict)    # hidden constants (e.g. the pin)
    parse: Callable = None                       # custom words -> args, instead of `types`

    def build(self, words):
        """Turn the command's words into a one-step task around the client call."""
        args = self.parse(words) if self.parse else convert(words, self.types)

        def invoke(hardware, done):
            client = getattr(hardware, self.module)          # e.g. hardware.movement
            return getattr(client, self.method)(*args, **self.fixed, on_done=done)

        return invoke


COMMANDS = {
    # Movement
    'M':     Call('move', 'movement', 'move_gantry_abs', types=(float, float, float)),
    'M_S':   Call('move', 'movement', 'move_gantry_s', types=(float, float, float, float)),
    'H_0':   Call('home', 'movement', 'go_home'),
    'H_1':   Sequence(find_home, parse=axes),   # no letter = all axes
    'C_0':   Sequence(calibrate_axes, parse=axes),

    # Peripherals
    'D_W_0': Call('water_pump', 'devices', 'set_pin_value',
                  fixed=peripheral(WATER_PUMP_PIN, False)),
    'D_W_1': Call('water_pump', 'devices', 'set_pin_value',
                  fixed=peripheral(WATER_PUMP_PIN, True)),
    'D_V_0': Call('vacuum', 'devices', 'set_pin_value',
                  fixed=peripheral(VACUUM_PIN, False)),
    'D_V_1': Call('vacuum', 'devices', 'set_pin_value',
                  fixed=peripheral(VACUUM_PIN, True)),
    'D_L_0': Call('led', 'devices', 'set_pin_value',
                  fixed=peripheral(LED_PIN, False)),
    'D_L_1': Call('led', 'devices', 'set_pin_value',
                  fixed=peripheral(LED_PIN, True)),
    'P4_0':  Call('peripheral_4', 'devices', 'set_pin_value',
                  fixed=peripheral(PERIPHERAL_4_PIN, False)),
    'P4_1':  Call('peripheral_4', 'devices', 'set_pin_value',
                  fixed=peripheral(PERIPHERAL_4_PIN, True)),
    'P5_0':  Call('peripheral_5', 'devices', 'set_pin_value',
                  fixed=peripheral(PERIPHERAL_5_PIN, False)),
    'P5_1':  Call('peripheral_5', 'devices', 'set_pin_value',
                  fixed=peripheral(PERIPHERAL_5_PIN, True)),

    # Devices
    'M_SV':  Call('servo', 'devices', 'move_servo', types=(int, float)),
    'D_C':   Call('check_tool', 'devices', 'read_pin', fixed={'pin': TOOL_PIN, 'pin_mode': False}),
    'D_S_C': Call('check_soil', 'devices', 'read_pin', fixed={'pin': SOIL_PIN, 'pin_mode': True}),

    # States
    'SW_VER': Call('sw_version', 'states', 'request_sw_version'),

    # Map sequences
    'P_3': ComplexSequence('map_sequences', 'seed_plants_command'),
    'P_4': ComplexSequence('map_sequences', 'water_plants_cmd', fixed={'rigid': True}),
    'P_5': ComplexSequence('map_sequences', 'water_plants_cmd', fixed={'rigid': False}),
    'P_9': ComplexSequence('map_sequences', 'check_moisture_cmd'),

    # Tool sequences
    'T_1_1': ComplexSequence('tool_sequences', 'mount_tool_command', fixed={'index': 1}),
    'T_1_2': ComplexSequence('tool_sequences', 'unmount_tool_command', fixed={'index': 1}),
    'T_2_1': ComplexSequence('tool_sequences', 'mount_tool_command', fixed={'index': 2}),
    'T_2_2': ComplexSequence('tool_sequences', 'unmount_tool_command', fixed={'index': 2}),
    'T_3_1': ComplexSequence('tool_sequences', 'mount_tool_command', fixed={'index': 3}),
    'T_3_2': ComplexSequence('tool_sequences', 'unmount_tool_command', fixed={'index': 3}),
    'T_4_1': ComplexSequence('tool_sequences', 'mount_tool_command', fixed={'index': 4}),
    'T_4_2': ComplexSequence('tool_sequences', 'unmount_tool_command', fixed={'index': 4}),
    'T_5_1': ComplexSequence('tool_sequences', 'mount_tool_command', fixed={'index': 5}),
    'T_5_2': ComplexSequence('tool_sequences', 'unmount_tool_command', fixed={'index': 5}),
    'T_6_1': ComplexSequence('tool_sequences', 'mount_tool_command', fixed={'index': 6}),
    'T_6_2': ComplexSequence('tool_sequences', 'unmount_tool_command', fixed={'index': 6}),

}


def to_task(command: str):
    """Parse an operator command into a queueable task."""
    code, *words = command.split(' ')
    spec = COMMANDS.get(code)
    if spec is None:
        raise ValueError('unknown command')
    return spec.build(words)


def unresolved(hardware) -> list:
    """Return command codes whose Call names a method missing on the hardware bundle."""
    return [code for code, spec in COMMANDS.items()
            if isinstance(spec, Call)
            and not hasattr(getattr(hardware, spec.module, None), spec.method)]
