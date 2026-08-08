import os
import signal

import pytest

from ft_fb_leaderarm import collector_node, observer_node


@pytest.mark.parametrize(
    ("module", "node_class"),
    [
        (collector_node, "PhysicalFtCollector"),
        (observer_node, "PhysicalFtContactObserver"),
    ],
)
def test_repeated_sigint_does_not_interrupt_cleanup(monkeypatch, module, node_class):
    events = []

    class FakeNode:
        def destroy_node(self):
            os.kill(os.getpid(), signal.SIGINT)
            events.append("destroyed")

    def interrupt_spin(_node):
        raise KeyboardInterrupt

    previous_sigint = signal.getsignal(signal.SIGINT)
    monkeypatch.setattr(module, node_class, FakeNode)
    monkeypatch.setattr(module.rclpy, "init", lambda args=None: events.append("init"))
    monkeypatch.setattr(module.rclpy, "spin", interrupt_spin)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: events.append("shutdown"))

    module.main()

    assert events == ["init", "destroyed", "shutdown"]
    assert signal.getsignal(signal.SIGINT) is previous_sigint
