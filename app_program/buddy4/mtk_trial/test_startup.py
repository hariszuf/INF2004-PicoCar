"""Check the board startup register table and motor-pin ownership (no hardware)."""
from pathlib import Path
import re
s = (Path(__file__).parent / 'overlay/kernel/sysdepend/pico_rp2040/hw_setting.c').read_text()
table = s.split('LOCAL const T_SETUP_REG pinfnc_tbl[] = {',1)[1].split('\n};',1)[0]
assert '{GPIO_OUT_CLR, 0x00000F00UL}' in table
assert '{GPIO_OE_SET, 0x00000F00UL}' in table
assert table.index('GPIO_OUT_CLR') < table.index('GPIO_CTRL(8)') < table.index('GPIO_OE_SET')
assert 'BOARD_LED_PIN' not in table
assert not re.search(r'\{GPIO_OE\s*,', table)
assert 'GPIO_OUT_SET' not in table
for pin in (8,9,10,11):
    assert f'{{GPIO_CTRL({pin}), GPIO_CTRL_FUNCSEL_SIO}}' in table
    pad = re.search(r'\{GPIO\(' + str(pin) + r'\),([^}]+)\}',table).group(1)
    assert 'GPIO_PUE' not in pad and 'GPIO_PDE' not in pad
    assert not re.search(r'GPIO_CTRL\(' + str(pin) + r'\),\s*GPIO_CTRL_FUNCSEL_I2C', s)
for pin in (16,17):
    assert f'{{GPIO_CTRL({pin}), GPIO_CTRL_FUNCSEL_I2C}}' in table
assert '#if BOARD_DIAG_GPIO_SCAN\n#error' in s
driver = (Path(__file__).parent / 'overlay/device/i2c/sysdepend/rp2040/i2c_rp2040.c').read_text()
for pin in (8,9,10,11):
    assert not re.search(r'GPIO_CTRL\(' + str(pin) + r'\)', driver)
print('Startup pin checks passed: motor latches low before SIO/OE; no motor pull-ups/I2C; IMU GP16/17; no GP16 LED; diagnostic scans forbidden. Hardware unverified.')
