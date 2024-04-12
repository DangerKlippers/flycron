import logging
import struct


class PidParams:
    def __init__(self, index, prefix, defaults):
        self.index = index
        self.prefix = prefix
        self.limit = defaults.get("limit", 0.0)
        self.p = defaults.get("p", 0.0)
        self.p_limit = defaults.get("p_limit", 1.0)
        self.i = defaults.get("i", 0.0)
        self.i_limit = defaults.get("i_limit", 1.0)
        self.d = defaults.get("d", 0.0)
        self.d_limit = defaults.get("d_limit", 1.0)

    def load(self, config):
        self.limit = config.getfloat(f"pid_{self.prefix}_limit", self.limit)
        self.p = config.getfloat(f"pid_{self.prefix}_p", self.p)
        self.p_limit = config.getfloat(f"pid_{self.prefix}_p_limit", self.p_limit)
        self.i = config.getfloat(f"pid_{self.prefix}_i", self.i)
        self.i_limit = config.getfloat(f"pid_{self.prefix}_i_limit", self.i_limit)
        self.d = config.getfloat(f"pid_{self.prefix}_d", self.d)
        self.d_limit = config.getfloat(f"pid_{self.prefix}_d_limit", self.d_limit)

    def apply(self, cmd):
        cmd.send(
            [
                self.index,
                float_to_u32(self.limit),
                float_to_u32(self.p),
                float_to_u32(self.p_limit),
                float_to_u32(self.i),
                float_to_u32(self.i_limit),
                float_to_u32(self.d),
                float_to_u32(self.d_limit),
            ]
        )

    def gcmd_setpid(self, gcmd):
        self.limit = gcmd.get_float("LIMIT", self.limit)
        self.p = gcmd.get_float("P", self.p)
        self.p_limit = gcmd.get_float("P_LIMIT", self.p_limit)
        self.i = gcmd.get_float("I", self.i)
        self.i_limit = gcmd.get_float("I_LIMIT", self.i_limit)
        self.d = gcmd.get_float("D", self.d)
        self.d_limit = gcmd.get_float("D_LIMIT", self.d_limit)


class Flycron:
    def __init__(self, config, name):
        self.printer = config.get_printer()
        self.name = name

        mcu_name = config.get("mcu")
        ppins = self.printer.lookup_object("pins")
        self.mcu = ppins.chips.get(mcu_name, None)
        if self.mcu is None:
            raise config.error(f"unknown MCU '{mcu_name}'")
        self._stepper = None

        self.mass = config.getfloat("mass", 400.0)

        self.pid_pos = PidParams(
            0,
            "pos",
            {
                "limit": 240000.0,
                "p": 10.0,
                "p_limit": 240000.0,
            },
        )
        self.pid_pos.load(config)
        self.pid_vel = PidParams(
            1,
            "vel",
            {
                "limit": 2400.0,
                "p": 450.0,
                "p_limit": 2400.0,
                "i": 450.0,
                "i_limit": 2400.0,
            },
        )
        self.pid_vel.load(config)

        self.obs_alpha = config.getfloat("obs_alpha", 0.8)
        self.obs_beta = config.getfloat("obs_beta", 0.07)

        self.slew_limit_rising = config.getfloat("slew_limit_rising", float(3.40282347E+37))
        self.slew_limit_falling = config.getfloat("slew_limit_falling", float(-3.40282347E+37))

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:mcu_identify", self._mcu_identify)
        self.printer.register_event_handler("stepper_enable:motor_off", self._motor_off)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "FLYCRON_SETPID",
            "MCU",
            self.name,
            self.cmd_FLYCRON_SETPID,
            desc=self.cmd_FLYCRON_SETPID_help,
        )
        gcode.register_mux_command(
            "FLYCRON_SETFILTER",
            "MCU",
            self.name,
            self.cmd_FLYCRON_SETFILTER,
            desc=self.cmd_FLYCRON_SETFILTER_help,
        )
        gcode.register_mux_command(
            "FLYCRON_SETSLEW",
            "MCU",
            self.name,
            self.cmd_FLYCRON_SETSLEW,
            desc=self.cmd_FLYCRON_SETSLEW_help,
        )
        gcode.register_mux_command(
            "FLYCRON_GET_POSITION",
            "MCU",
            self.name,
            self.cmd_FLYCRON_GET_POSITION,
            desc=self.cmd_FLYCRON_GET_POSITION_help,
        )
        gcode.register_mux_command(
            "FLYCRON_ENCODER_SET",
            "MCU",
            self.name,
            self.cmd_FLYCRON_ENCODER_SET,
            desc=self.cmd_FLYCRON_ENCODER_SET_help,
        )

    def _mcu_identify(self):
        self.pid_set_gains_cmd = self.mcu.lookup_command(
            "pid_set_gains target=%u limit=%u p=%u p_max=%u i=%u i_max=%u d=%i d_max=%u"
        )
        self.pid_set_coefs_cmd = self.mcu.lookup_command(
            "pid_set_coefs target=%u alpha=%u beta=%u"
        )
        self.pid_set_slew_limits = self.mcu.lookup_command(
            "pid_set_slew_limits target=%u limit_rising=%u limit_falling=%u",
        )
        self.pid_set_mass = self.mcu.lookup_command(
            "pid_set_mass mass_grams=%u",
        )
        self.pid_enable_cmd = self.mcu.lookup_command("pid_set_enable enable=%c")
        self.pid_dump_cmd = self.mcu.lookup_query_command(
            "pid_get_dump", "pid_dump throttle=%u"
        )
        self.encoder_set_position_cmd = self.mcu.lookup_command(
            "encoder_set_position value=%u"
        )

    def _handle_connect(self):
        self._apply()

    def _motor_off(self, print_time):
        stepper = self.get_stepper()
        if stepper is not None:
            stepper._query_mcu_position()

    def _apply(self):
        self.pid_pos.apply(self.pid_set_gains_cmd)
        self.pid_vel.apply(self.pid_set_gains_cmd)
        self.pid_set_coefs_cmd.send(
            [0, float_to_u32(self.obs_alpha), float_to_u32(self.obs_beta)]
        )
        self.pid_set_slew_limits.send(
            [
                2,
                float_to_u32(self.slew_limit_rising),
                float_to_u32(self.slew_limit_falling),
            ]
        )
        self.pid_set_mass.send([float_to_u32(self.mass)])

    def get_stepper(self):
        if self._stepper is None:
            steppers = self.printer.lookup_object("force_move").steppers
            for stepper in steppers.values():
                if stepper.get_mcu() == self.mcu:
                    self._stepper = stepper
                    break
        return self._stepper

    cmd_FLYCRON_SETPID_help = """
    Sets PID parameters for the controller
    """

    def cmd_FLYCRON_SETPID(self, gcmd):
        target = gcmd.get("TARGET")
        if target == "pos":
            self.pid_pos.gcmd_setpid(gcmd)
        elif target == "vel":
            self.pid_vel.gcmd_setpid(gcmd)
        else:
            gcmd.respond_error("Unknown PID target")
            return
        self._apply()

    cmd_FLYCRON_SETFILTER_help = """
    Set observer alpha/beta filter parameters
    """

    def cmd_FLYCRON_SETFILTER(self, gcmd):
        self.obs_alpha = gcmd.get_float("ALPHA", self.obs_alpha)
        self.obs_beta = gcmd.get_float("BETA", self.obs_beta)
        self._apply()

    cmd_FLYCRON_SETPOINT_help = """
    Sets the setpoint of a Flycron controller
    """

    cmd_FLYCRON_SETSLEW_help = """
    Sets the slew rate limits of the final throttle output
    """

    def cmd_FLYCRON_SETSLEW(self, gcmd):
        self.slew_limit_rising = gcmd.get_float("RISING", self.slew_limit_rising)
        self.slew_limit_falling = gcmd.get_float("FALLING", self.slew_limit_falling)
        self._apply()

    cmd_FLYCRON_GET_POSITION_help = """
    Gets the current commanded and actual positions from a Flycron controller
    """

    def cmd_FLYCRON_GET_POSITION(self, gcmd):
        stepper = self.get_stepper()
        if stepper is None:
            gcmd.respond_error("Could not find stepper for controller")
            return

        oid = stepper.get_oid()
        pos = stepper._get_position_cmd.send([oid])["pos"]
        commanded_pos = self.mcu.lookup_query_command(
            "stepper_get_commanded_position oid=%c",
            "stepper_commanded_position oid=%c pos=%i",
            oid=oid,
        ).send([oid])["pos"]
        throttle = u32_to_float(self.pid_dump_cmd.send([])["throttle"])

        gcmd.respond_info(
            f"Actual {pos}, commanded {commanded_pos}, throttle {throttle}"
        )

    cmd_FLYCRON_ENCODER_SET_help = """
    Gets the current commanded and actual positions from a Flycron controller
    """

    def cmd_FLYCRON_ENCODER_SET(self, gcmd):
        value = gcmd.get_int("POS")
        self.encoder_set_position_cmd.send([value])


def float_to_u32(v):
    x = struct.unpack("I", struct.pack("f", float(v)))[0]
    return x


def u32_to_float(v):
    x = struct.unpack("f", struct.pack("I", v))[0]
    return x


def load_config_prefix(config):
    name = config.get_name()[len("flycron ") :]
    logging.info("name %s", name)
    return Flycron(config, name)
