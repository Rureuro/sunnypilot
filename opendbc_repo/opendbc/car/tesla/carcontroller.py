import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.tesla.coop_steering import CoopSteeringCarController


DRIVER_STEER_HANDS_ON_LEVEL = 2
DRIVER_STEER_QUIET_FRAMES = 10  # require a stable release before re-enabling angle control


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase, CoopSteeringCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    CoopSteeringCarController.__init__(self)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)
    self.driver_steer_quiet_frames = DRIVER_STEER_QUIET_FRAMES

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CC_SP, CS, now_nanos):
    CoopSteeringCarController.update(self, self.CP_SP)
    actuators = CC.actuators
    can_sends = []

    # Tesla angle control does not blend with driver torque like torque-based LKAS.
    # Keep MADS enabled, but only re-enable angle control after driver steering
    # has been stably released so momentary steeringPressed drops do not chatter.
    driver_steering = CS.out.steeringPressed or CS.hands_on_level >= DRIVER_STEER_HANDS_ON_LEVEL
    if driver_steering:
      self.driver_steer_quiet_frames = 0
    else:
      self.driver_steer_quiet_frames = min(self.driver_steer_quiet_frames + 1, DRIVER_STEER_QUIET_FRAMES)

    lat_active = CC.latActive and self.driver_steer_quiet_frames >= DRIVER_STEER_QUIET_FRAMES

    if self.frame % 2 == 0:
      if lat_active:
        # Angular rate limit based on speed
        self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                            True, CarControllerParams, self.VM)
      else:
        self.apply_angle_last = CS.out.steeringAngleDeg

      can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active, self.coop_steering.control_type))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
