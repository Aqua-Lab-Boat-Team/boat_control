from typing import TypeVar

from .registry import STATE_REGISTRY

StateT = TypeVar("StateT")


class SubscribedState:

    def __init__(self, node, requested_states: list[type]):
        self._node = node

        # Local state instances owned by this node
        self._states: dict[type, object] = {}

        # Keep subscription objects alive
        self._subscriptions = []

        for state_type in requested_states:
            self._add_state(state_type)

    def _add_state(self, state_type: type):
        if state_type not in STATE_REGISTRY:
            raise KeyError(
                f"{state_type.__name__} is not registered in STATE_REGISTRY"
            )

        spec = STATE_REGISTRY[state_type]

        # Create a fresh local dataclass instance
        state = spec.state_type()

        self._states[state_type] = state

        # Capture state/spec explicitly so each callback refers
        # to the correct objects.
        def callback(msg, state=state, spec=spec):
            spec.update(state, msg)

        subscription = self._node.create_subscription(
            spec.msg_type,
            spec.topic,
            callback,
            spec.qos,
        )

        self._subscriptions.append(subscription)

    def get(self, state_type: type[StateT]) -> StateT:
        return self._states[state_type]