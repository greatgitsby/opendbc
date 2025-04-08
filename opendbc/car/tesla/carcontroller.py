import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams


def torque_blended_angle(apply_angle, torsion_bar_torque):
  deadzone = CarControllerParams.TORQUE_TO_ANGLE_DEADZONE
  if abs(torsion_bar_torque) < deadzone:
    return apply_angle

  limit = CarControllerParams.TORQUE_TO_ANGLE_CLIP
  if apply_angle * torsion_bar_torque >= 0:
    # Manually steering in the same direction as OP
    strength = CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER_OUTER
  else:
    # User is opposing OP direction
    strength = CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER_INNER

  torque = torsion_bar_torque - deadzone if torsion_bar_torque > 0 else torsion_bar_torque + deadzone
  return apply_angle + np.clip(torque, -limit, limit) * strength

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.last_hands_nanos = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Disengage and allow for user override on high torque inputs
    # TODO: move this to a generic disengageRequested carState field and set CC.cruiseControl.cancel based on it
    hands_on_fault = (CS.hands_on_level >= 3
        # user is applying lots of force or...
        or (CS.steering_override and abs(CS.out.steeringAngleDeg - actuators.steeringAngleDeg) > CarControllerParams.CONTINUED_OVERRIDE_ANGLE) # already overriding and...
      and not CS.out.standstill)  # continued angular disagreement while moving.
    cruise_cancel = CC.cruiseControl.cancel or hands_on_fault
    lat_active = CC.latActive and not hands_on_fault

    if self.frame % 2 == 0:
      if CS.hands_on_level > 0:
        self.last_hands_nanos = now_nanos
      elif now_nanos - self.last_hands_nanos > 1e9:
        CS.steering_override = False

      # Reset override when disengaged to ensure a fresh activation always engages steering.
      if not CC.latActive:
        CS.steering_override = False

      if lat_active:
        torque_blended_angle_deg = torque_blended_angle(actuators.steeringAngleDeg, CS.out.steeringTorque)

        # Angular rate limit based on speed
        self.apply_angle_last = apply_std_steer_angle_limits(torque_blended_angle_deg, self.apply_angle_last, CS.out.vEgo,
                                                             CS.out.steeringAngleDeg, CC.lat_active, CarControllerParams.ANGLE_LIMITS)
      else:
        self.apply_angle_last = CS.out.steeringAngleDeg

      can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active, (self.frame // 2) % 16))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed((self.frame // 10) % 16))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if cruise_cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if cruise_cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
