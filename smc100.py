#!/usr/bin/env python
import serial
import time

from math import floor

# never wait for more than this e.g. during wait_states
MAX_WAIT_TIME_SEC = 12

# time to wait after sending a command. This number has been arrived at by
# trial and error
COMMAND_WAIT_TIME_SEC = 0.06

# States from page 65 of the manual
STATE_NOT_REFERENCED_FROM_RESET = '0A'
STATE_NOT_REFERENCED_FROM_CONFIGURATION = '0C'
STATE_READY_FROM_HOMING = '32'
STATE_READY_FROM_MOVING = '33'

STATE_CONFIGURATION = '14'

STATE_DISABLE_FROM_READY = '3C'
STATE_DISABLE_FROM_MOVING = '3D'
STATE_DISABLE_FROM_JOGGING = '3E'

class SMC100ReadTimeOutException(Exception):
  def __init__(self):
    super(SMC100ReadTimeOutException, self).__init__('Read timed out')

class SMC100WaitTimedOutException(Exception):
  def __init__(self):
    super(SMC100WaitTimedOutException, self).__init__('Wait timed out')

class SMC100DisabledStateException(Exception):
  def __init__(self, state):
    super(SMC100DisabledStateException, self).__init__('Disabled state encountered: '+state)

class SMC100RS232CorruptionException(Exception):
  def __init__(self, c):
    super(SMC100RS232CorruptionException, self).__init__('RS232 corruption detected: %s'%(hex(ord(c))))

class SMC100InvalidResponseException(Exception):
  def __init__(self, cmd, resp):
    s = 'Invalid response to %s: %s'%(cmd, resp)
    super(SMC100InvalidResponseException, self).__init__(s)

class SMC100(object):
  """
  Class to interface with Newport's SMC100 controller.

  The SMC100 accepts commands in the form of:

    <ID><command><arguments><CR><LF>

  Reply, if any, will be in the form

    <ID><command><result><CR><LF>

  There is minimal support for manually setting stage parameter as Newport's
  ESP stages can supply the SMC100 with the correct configuration parameters.

  Some effort is made to take up backlash, but this should not be trusted too
  much.

  The move commands must be used with care, because they make assumptions
  about the units which is dependent on the STAGE. I only have TRB25CC, which
  has native units of mm. A more general implementation will move the move
  methods into a stage class.
  """

  _port = None
  _smcID = None

  _silent = True

  _sleepfunc = time.sleep

  def __init__(self, smcID, port, backlash_compensation=True, silent=True, sleepfunc=None):
    """
    If backlash_compensation is False, no backlash compensation will be done.

    If silent is False, then additional output will be emitted to aid in
    debugging.

    If sleepfunc is not None, then it will be used instead of time.sleep. It
    will be given the number of seconds (float) to sleep for, and is provided
    for ease integration with single threaded GUIs.

    Note that this method only connects to the controller, it otherwise makes
    no attempt to home or configure the controller for the attached stage. This
    delibrate to minimise realworld side effects.

    If the controller has previously been configured, it will suffice to simply
    call home() to take the controller out of not referenced mode. For a brand
    new controller, call reset_and_configure().
    """

    super(SMC100, self).__init__()

    assert smcID is not None
    assert port is not None

    if sleepfunc is not None:
      self._sleepfunc = sleepfunc

    self._silent = silent

    self._last_sendcmd_time = 0

    print 'Connecting to SMC100 on %s'%(port)

    self._port = serial.Serial(
        port = port,
        baudrate = 57600,
        bytesize = 8,
        stopbits = 1,
        parity = 'N',
        xonxoff = True,
        timeout = 0.050)

    self._smcID = str(smcID)

  def reset_and_configure(self):
    """
    Configures the controller by resetting it and then asking it to load
    stage parameters from an ESP compatible stage. This is then followed
    by a homing action.
    """
    self.sendcmd('RS')
    self.sendcmd('RS')

    self._sleepfunc(3)

    self.wait_states(STATE_NOT_REFERENCED_FROM_RESET, ignore_disabled_states=True)

    stage = self.sendcmd('ID', '?', True)
    print 'Found stage', stage

    # enter config mode
    self.sendcmd('PW', 1)

    self.wait_states(STATE_CONFIGURATION)

    # load stage parameters
    self.sendcmd('ZX', 1)

    # enable stage ID check
    self.sendcmd('ZX', 2)

    # exit configuration mode
    self.sendcmd('PW', 0)

    # wait for us to get back into NOT REFERENCED state
    self.wait_states(STATE_NOT_REFERENCED_FROM_CONFIGURATION)

  def home(self, waitStop=True):
    """
    Homes the controller. If waitStop is True, then this method returns when
    homing is complete.

    Note that because calling home when the stage is already homed has no
    effect, and homing is generally expected to place the stage at the
    origin, an absolute move to 0 um is executed after homing. This ensures
    that the stage is at origin after calling this method.

    Calling this method is necessary to take the controller out of not referenced
    state after a restart.
    """
    self.sendcmd('OR')
    if waitStop:
      # wait for the controller to be ready
      st = self.wait_states((STATE_READY_FROM_HOMING, STATE_READY_FROM_MOVING))
      if st == STATE_READY_FROM_MOVING:
        self.move_absolute_um(0, waitStop=True)
    else:
      self.move_absolute_um(0, waitStop=False)

  def stop(self):
    self.sendcmd('ST')

  def get_status(self, silent=False):
    """
    Executes TS? and returns the the error code as integer and state as string
    as specified on pages 64 - 65 of the manual.
    """

    resp = self.sendcmd('TS', '?', expect_response=True, retry=10)
    errors = int(resp[0:4], 16)
    state = resp[4:]

    assert len(state) == 2
    
    if not silent:
      print('status:')
      if state == '0A':
        print('  state: NOT REFERENCED from reset')
      elif state == '0B':
        print('  state: NOT REFERENCED from HOMING')
      elif state == '0C':
        print('  state: NOT REFERENCED from CONFIGURATION')
      elif state == '0D':
        print('  state: NOT REFERENCED from DISABLE')
      elif state == '0E':
        print('  state: NOT REFERENCED from READY')
      elif state == '0F':
        print('  state: NOT REFERENCED from MOVING')
      elif state == '10':
        print('  state: NOT REFERENCED ESP stage error')
      elif state == '11':
        print('  state: NOT REFERENCED from JOGGING')
      elif state == '14':
        print('  state: CONFIGURATION')
      elif state == '1E':
        print('  state: HOMING commanded from RS-232-C')
      elif state == '1F':
        print('  state: HOMING commanded by SMC-RC')
      elif state == '28':
        print('  state: MOVING')
      eif state == '32':
        print('  state: READY from HOMING')
      elif state == '33':
        print('  state: READY from MOVING')
      elif state == '34':
        print('  state: READY from DISABLE')
      elif state == '35':
        print('  state: READY from JOGGING')
      elif state == '3C':
        print('  state: DISABLE from READY')
      elif state == '3D':
        print('  state: DISABLE from MOVING')
      elif state == '3E':
        print('  state: DISABLE from JOGGING')
      elif state == '46':
        print('  state: JOGGING from READY')
      elif state == '47':
        print('  state: JOGGING from DISABLE')
    return errors, state

  def get_position_mm(self):
    dist_mm = float(self.sendcmd('TP', '?', expect_response=True, retry=10))
    return dist_mm

  def get_position_um(self):
    return int(self.get_position_mm()*1000)

  def move_relative_mm(self, dist_mm, waitStop=True):
    """
    Moves the stage relatively to the current position by the given distance given in mm

    If waitStop is True then this method returns when the move is completed.
    """
    self.sendcmd('PR', dist_mm)
    if waitStop:
      # If we were previously homed, then something like PR0 will have no
      # effect and we end up waiting forever for ready from moving because
      # we never left ready from homing. This is why STATE_READY_FROM_HOMING
      # is included.
      self.wait_states((STATE_READY_FROM_MOVING, STATE_READY_FROM_HOMING))


  def move_relative_um(self, dist_um, **kwargs):
    """
    Moves the stage relatively to the current position by the given distance given in um. The
    given distance is first converted to an integer.

    If waitStop is True then this method returns when the move is completed.
    """
    dist_mm = int(dist_um)/1000
    self.move_relative_mm(dist_mm, **kwargs)

  def move_absolute_mm(self, position_mm, waitStop=True):
    """
    Moves the stage to the given absolute position given in mm.

    If waitStop is True then this method returns when the move is completed.
    """
    self.sendcmd('PA', position_mm)
    if waitStop:
      # If we were previously homed, then something like PR0 will have no
      # effect and we end up waiting forever for ready from moving because
      # we never left ready from homing. This is why STATE_READY_FROM_HOMING
      # is included.
      self.wait_states((STATE_READY_FROM_MOVING, STATE_READY_FROM_HOMING))

  def move_absolute_um(self, position_um, **kwargs):
    """
    Moves the stage to the given absolute position given in um. Note that the
    position specified will be floor'd first before conversion to mm.

    If waitStop is True then this method returns when the move is completed.
    """
    pos_mm = floor(position_um)/1000
    return self.move_absolute_mm(pos_mm, **kwargs)

  def wait_states(self, targetstates, ignore_disabled_states=False):
    """
    Waits for the controller to enter one of the the specified target state.
    Controller state is determined via the TS command.

    If ignore_disabled_states is True, disable states are ignored. The normal
    behaviour when encountering a disabled state when not looking for one is
    for an exception to be raised.

    Note that this method will ignore read timeouts and keep trying until the
    controller responds.  Because of this it can be used to determine when the
    controller is ready again after a command like PW0 which can take up to 10
    seconds to execute.

    If any disable state is encountered, the method will raise an error,
    UNLESS you were waiting for that state. This is because if we wait for
    READY_FROM_MOVING, and the stage gets stuck we transition into
    DISABLE_FROM_MOVING and then STAY THERE FOREVER.

    The state encountered is returned.
    """
    starttime = time.time()
    done = False
    self._emit('waiting for states %s'%(str(targetstates)))
    while not done:
      waittime = time.time() - starttime
      if waittime > MAX_WAIT_TIME_SEC:
        raise SMC100WaitTimedOutException()

      try:
        state = self.get_status()[1]
        if state in targetstates:
          self._emit('in state %s'%(state))
          return state
        elif not ignore_disabled_states:
          disabledstates = [
              STATE_DISABLE_FROM_READY,
              STATE_DISABLE_FROM_JOGGING,
              STATE_DISABLE_FROM_MOVING]
          if state in disabledstates:
            raise SMC100DisabledStateException(state)

      except SMC100ReadTimeOutException:
        self._emit('Read timed out, retrying in 1 second')
        self._sleepfunc(1)
        continue

  def sendcmd(self, command, argument=None, expect_response=False, retry=False):
    """
    Send the specified command along with the argument, if any. The response
    is checked to ensure it has the correct prefix, and is returned WITHOUT
    the prefix.

    It is important that for GET commands, e.g. 1ID?, the ? is specified as an
    ARGUMENT, not as part of the command. Doing so will result in assertion
    failure.

    If expect_response is True, a response is expected from the controller
    which will be verified and returned without the prefix.

    If expect_response is True, and retry is True or an integer, then when the
    response does not pass verification, the command will be sent again for
    retry number of times, or until success if retry is True.

    The retry option MUST BE USED CAREFULLY. It should ONLY be used read-only
    commands, because otherwise REPEATED MOTION MIGHT RESULT. In fact some
    commands are EXPLICITLY REJECTED to prevent this, such as relative move.
    """
    assert command[-1] != '?'

    if self._port is None:
      return

    if argument is None:
      argument = ''

    prefix = self._smcID + command
    tosend = prefix + str(argument)

    # prevent certain commands from being retried automatically
    no_retry_commands = ['PR', 'OR']
    if command in no_retry_commands:
      retry = False

    while self._port is not None:
      if expect_response:
        self._port.flushInput()

      self._port.flushOutput()

      self._port.write(tosend)
      self._port.write('\r\n')

      self._port.flush()

      if not self._silent:
        self._emit('sent', tosend)

      if expect_response:
        try:
          response = self._readline()
          if response.startswith(prefix):
            return response[len(prefix):]
          else:
            raise SMC100InvalidResponseException(command, response)
        except Exception, ex:
          if not retry or retry <=0:
            raise ex
          else:
            if type(retry) == int:
              retry -= 1
            continue
      else:
        # we only need to delay when we are not waiting for a response
        now = time.time()
        dt = now - self._last_sendcmd_time
        dt = COMMAND_WAIT_TIME_SEC - dt
        if dt > 0:
          self._sleepfunc(dt)
        
        self._last_sendcmd_time = now
        return None

  def _readline(self):
    """
    Returns a line, that is reads until \r\n.

    OK, so you are probably wondering why I wrote this. Why not just use
    self._port.readline()?

    I am glad you asked.

    With python < 2.6, pySerial uses serial.FileLike, that provides a readline
    that accepts the max number of chars to read, and the end of line
    character.

    With python >= 2.6, pySerial uses io.RawIOBase, whose readline only
    accepts the max number of chars to read. io.RawIOBase does support the
    idea of a end of line character, but it is an attribute on the instance,
    which makes sense... except pySerial doesn't pass the newline= keyword
    argument along to the underlying class, and so you can't actually change
    it.
    """
    done = False
    line = str()
    #print 'reading line',
    while not done:
      c = self._port.read()
      # ignore \r since it is part of the line terminator
      if len(c) == 0:
        raise SMC100ReadTimeOutException()
      elif c == '\r':
        continue
      elif c == '\n':
        done = True
      elif ord(c) > 32 and ord(c) < 127:
        line += c
      else:
        raise SMC100RS232CorruptionException(c)

    self._emit('read', line)

    return line

  def _emit(self, *args):
    if len(args) == 1:
      prefix = ''
      message = args[0]
    else:
      prefix = ' ' + args[0]
      message = args[1]

    if not self._silent:
      print '[SMC100' + prefix + '] ' + message

  def close(self):
    if self._port:
      self._port.close()
      self._port = None

  def __del__(self):
    self.close()
# Tests #####################################################################
def test_configure():
  smc100 = SMC100(1, '/dev/ttyS5', silent=False)
  smc100.reset_and_configure()
  # make sure there are no errors
  assert smc100.get_status()[0] == 0
  del smc100

def test_general():
  smc100 = SMC100(1, '/dev/ttyS5', silent=False)
  print smc100.get_position_mm()

  smc100.home()

  # make sure there are no errors
  assert smc100.get_status()[0] == 0

  smc100.move_relative_um(5*1000)
  smc100.move_relative_mm(5)

  assert smc100.get_status()[0] == 0

  pos = smc100.get_position_mm()

  assert abs(pos-10)<0.001

  smc100.move_relative_mm(-pos)

  assert smc100.get_status()[0] == 0

  del smc100

