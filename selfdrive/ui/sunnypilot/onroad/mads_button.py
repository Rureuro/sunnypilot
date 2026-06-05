import pyray as rl

from cereal import messaging
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget


class MadsButton(Widget):
  def __init__(self):
    super().__init__()
    self._pm = messaging.PubMaster(["madsEnableButton", "madsDisableButton"])
    self._click_delay = 0.12

  @staticmethod
  def available() -> bool:
    mads = ui_state.sm["selfdriveStateSP"].mads
    tesla = ui_state.CP is not None and ui_state.CP.brand == "tesla"
    return ui_state.started and tesla and mads.available

  def _update_state(self):
    self.set_visible(self.available())

  def toggle(self):
    service = "madsDisableButton" if ui_state.sm["selfdriveStateSP"].mads.enabled else "madsEnableButton"
    msg = messaging.new_message(service)
    msg.valid = True
    self._pm.send(service, msg)

  def _handle_mouse_release(self, mouse_pos):
    self.toggle()
    super()._handle_mouse_release(mouse_pos)

  def _render(self, rect):
    mads = ui_state.sm["selfdriveStateSP"].mads
    active = bool(mads.enabled)
    pressed = self.is_pressed

    bg = rl.Color(22, 127, 64, 235) if active else rl.Color(12, 18, 24, 225)
    if pressed:
      bg = rl.Color(max(bg.r - 20, 0), max(bg.g - 20, 0), max(bg.b - 20, 0), bg.a)

    border = rl.Color(255, 255, 255, 210 if active else 135)
    text = rl.WHITE if active else rl.Color(235, 241, 245, 235)
    subtext = rl.Color(225, 235, 230, 230) if active else rl.Color(185, 196, 204, 230)

    rl.draw_rectangle_rounded(rect, 0.28, 12, bg)
    rl.draw_rectangle_rounded_lines_ex(rect, 0.28, 12, 3, border)

    title = "MADS"
    subtitle = "LAT"
    title_size = 34
    subtitle_size = 22
    title_width = rl.measure_text(title, title_size)
    subtitle_width = rl.measure_text(subtitle, subtitle_size)
    cx = rect.x + rect.width / 2
    y = rect.y + rect.height / 2 - 31
    rl.draw_text(title, int(cx - title_width / 2), int(y), title_size, text)
    rl.draw_text(subtitle, int(cx - subtitle_width / 2), int(y + 42), subtitle_size, subtext)
