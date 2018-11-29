from datetime import datetime
# from .swarmalator import Swarmalator
from .utils import LogWriter
from .state import serialize_state_to_dict


class SystemState:
    def __init__(self, agent):
        self.agent = agent
        self.is_running = False

    def send_state(self, state):
        pass

    def send_vel(self, vel, ang_vel):
        pass

    def destroy(self):
        pass


class SystemStateManager:
    def __init__(self):
        self.agents = []
        self.is_running = False
        self.params = {}

    def get_system_state_class(self):
        return SystemState

    def get_system_state_args(self, ident):
        return []

    def get_system_state_kwargs(self, ident):
        return {}

    def create_agent(self, swarmalator_class, ident, params, orient_mode):
        self.params = params
        swarmalator = swarmalator_class(ident, params, orient_mode)
        SystemStateClass = self.get_system_state_class()
        args = self.get_system_state_args(ident)
        kwargs = self.get_system_state_kwargs(ident)

        s = SystemStateClass(
            agent=swarmalator,
            *args, **kwargs
        )
        self.agents.append(s)
        return s

    def change_params(self, params):
        self.params = params
        for agent in self.agents:
            agent.agent.params = params

    def reset_phase(self, phase=None):
        for agent in self.agents:
            agent.agent.state.reset_phase(phase)

    def set_frequency(self, f=None):
        for agent in self.agents:
            agent.agent.frequency = f

    def start(self):
        self.is_running = True
        for agent in self.agents:
            agent.is_running = True

    def stop(self):
        self.is_running = False
        for agent in self.agents:
            agent.is_running = False

    def destroy(self):
        self.stop()
        for agent in self.agents:
            agent.destroy()


class LoggableSystemStateMixin:
    def __init__(self, log_writer, *args, **kwargs):
        self._log_writer = log_writer
        super().__init__(*args, **kwargs)

    def send_state(self, state):
        if self._log_writer is not None:
            data = serialize_state_to_dict(state)
            data['id'] = self.agent.ident
            data['timestamp'] = datetime.now()
            self._log_writer.add_record(data)
        super().send_state(state)


class LoggableSystemStateManagerMixin:
    def __init__(self, log_file_path=None, *args, **kwargs):
        self._log_writer = LogWriter(log_file_path)
        super().__init__(*args, **kwargs)

    def get_system_state_kwargs(self, ident):
        kwargs = super().get_system_state_kwargs(ident)
        kwargs.update({
            'log_writer': self._log_writer,
        })
        return kwargs

    def start(self):
        self._log_writer.new_mission(self.params)
        super().start()

    def stop(self):
        self._log_writer.stop()
        super().stop()
