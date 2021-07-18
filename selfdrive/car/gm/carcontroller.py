from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl

VisualAlert = car.CarControl.HUDControl.VisualAlert

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations less than 0.02, don't change the accel command
  if accel > accel_steady + 0.02:
    accel_steady = accel - 0.02
  elif accel < accel_steady - 0.02:
    accel_steady = accel + 0.02
  accel = accel_steady

  return accel, accel_steady

class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.

    self.start_time = 0.
    self.apply_steer_last = 0
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    self.accel_steady = 0

    self.params = CarControllerParams()

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert, dragonconf):

    P = self.params

    # Send CAN commands.
    can_sends = []

    # STEER
    lkas_enabled = enabled and not CS.out.steerWarning and CS.out.vEgo > P.MIN_STEER_SPEED and CS.enable_lkas
    if (frame % P.STEER_STEP) == 0:
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      # dp
      blinker_on = CS.out.leftBlinker or CS.out.rightBlinker
      if not enabled:
        self.blinker_end_frame = 0
      if self.last_blinker_on and not blinker_on:
        self.blinker_end_frame = frame + dragonconf.dpSignalOffDelay
      apply_steer = common_controller_ctrl(enabled,
                                           dragonconf,
                                           blinker_on or frame < self.blinker_end_frame,
                                           apply_steer, CS.out.vEgo)
      self.last_blinker_on = blinker_on

      self.apply_steer_last = apply_steer
      idx = (frame // P.STEER_STEP) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    # Pedal/Regen
    if CS.CP.enableGasInterceptor and (frame % 2) == 0:

      if not enabled or not CS.adaptive_Cruise or CS.out.vEgo <= 16.6:
        final_pedal = 0
      elif CS.adaptive_Cruise and CS.out.vEgo > 16.6:
        accel = actuators.gas - actuators.brake
        accel, self.accel_steady = accel_hysteresis(accel, self.accel_steady)
        final_pedal = clip(accel, 0., 1.)

      idx = (frame // 2) % 4
      can_sends.append(create_gas_command(self.packer_pt, final_pedal, idx))

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = lkas_enabled == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert == VisualAlert.steerRequired
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    return can_sends
