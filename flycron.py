import logging
import struct


class Flycron:
    def __init__(self, config, name):
        self.printer = config.get_printer()
        self.name = name

        mcu_name = config.get("mcu")
        ppins = self.printer.lookup_object("pins")
        self.mcu = ppins.chips.get(mcu_name, None)
        if self.mcu is None:
            raise config.error(f"unknown MCU '{mcu_name}'")

        self.pid_p = config.getfloat("pid_p")
        self.pid_p_limit = config.getfloat("pid_p_limit", 1.0)
        self.pid_i = config.getfloat("pid_i", 0.0)
        self.pid_i_limit = config.getfloat("pid_i_limit", 0.0, 1.0)
        self.pid_d = config.getfloat("pid_d", 0.0)
        self.pid_d_limit = config.getfloat("pid_d_limit", 1.0)

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:mcu_identify", self._mcu_identify)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "FLYCRON_SETPID",
            "MCU",
            self.name,
            self.cmd_FLYCRON_SETPID,
            desc=self.cmd_FLYCRON_SETPID_help,
        )
        gcode.register_mux_command(
            "FLYCRON_SETPOINT",
            "MCU",
            self.name,
            self.cmd_FLYCRON_SETPOINT,
            desc=self.cmd_FLYCRON_SETPOINT_help,
        )

    def _mcu_identify(self):
        self.pid_update_cmd = self.mcu.lookup_command(
            "pid_set_gains p=%u p_max=%u i=%u i_max=%u d=%i d_max=%u"
        )
        self.pid_setpoint_cmd = self.mcu.lookup_command("pid_set_setpoint setpoint=%u")

    def _handle_connect(self):
        self._apply()

    def _apply(self):
        self.pid_update_cmd.send(
            [
                float_to_u32(self.pid_p),
                float_to_u32(self.pid_p_limit),
                float_to_u32(self.pid_i),
                float_to_u32(self.pid_i_limit),
                float_to_u32(self.pid_d),
                float_to_u32(self.pid_d_limit),
            ]
        )

    cmd_FLYCRON_SETPID_help = """
    Sets PID parameters for the controller
    """

    def cmd_FLYCRON_SETPID(self, gcmd):
        self.pid_p = gcmd.get_float("P", self.pid_p)
        self.pid_p_limit = gcmd.get_float("P_LIMIT", self.pid_p)
        self.pid_i = gcmd.get_float("I", self.pid_i)
        self.pid_i_limit = gcmd.get_float("I_LIMIT", self.pid_i)
        self.pid_d = gcmd.get_float("D", self.pid_d)
        self.pid_d_limit = gcmd.get_float("D_LIMIT", self.pid_d)
        self._apply()

    cmd_FLYCRON_SETPOINT_help = """
    Sets the setpoint of a Flycron controller
    """

    def cmd_FLYCRON_SETPOINT(self, gcmd):
        target = gcmd.get_float("TARGET")
        self.pid_setpoint_cmd.send([float_to_u32(target)])


def float_to_u32(v):
    x = struct.unpack("I", struct.pack("f", float(v)))[0]
    return x


def load_config_prefix(config):
    name = config.get_name()[len("flycron ") :]
    logging.info("name %s", name)
    return Flycron(config, name)
