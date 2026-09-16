"""Exercise the installed executor, without monkey patches or application startup.

Run in a disposable --network none container with an external timeout.
This is a dependency regression probe, not Perception/Canvas acceptance.
"""
import threading
import time
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import InvalidHandle
from std_msgs.msg import String


def main():
    rclpy.init(domain_id=95)
    executor = MultiThreadedExecutor(num_threads=2)
    publisher_node = rclpy.create_node('sim_lifecycle_probe_publisher')
    publisher = publisher_node.create_publisher(String, '/acceptance/lifecycle_probe', 10)
    errors = []
    received = [0]

    # Check both sides of the narrow guard: destroyed entity is skipped, but
    # the exact same exception from user code must still propagate.
    sub = publisher_node.create_subscription(String, '/acceptance/guard_contract', lambda msg: None, 10)

    def destroyed(entity):
        raise InvalidHandle('test: destroyed before take')

    async def forbidden(entity, value):
        raise AssertionError('callback invoked after failed take')

    async def callback_error(entity, value):
        raise InvalidHandle('test: user callback error')

    skipped = executor._make_handler(sub, publisher_node, destroyed, forbidden)
    skipped()
    assert skipped.done() and skipped.exception() is None
    assert not sub._executor_event and sub.callback_group.can_execute(sub)
    failed = executor._make_handler(sub, publisher_node, lambda entity: None, callback_error)
    failed()
    assert failed.done() and isinstance(failed.exception(), InvalidHandle)
    assert str(failed.exception()) == 'test: user callback error'
    assert sub.callback_group.can_execute(sub)
    publisher_node.destroy_subscription(sub)
    print('HANDLER_GUARD_CONTRACT_PASS callback_errors_propagate=True', flush=True)

    def receive(msg):
        received[0] += 1

    def spin():
        try:
            executor.spin()
        except Exception as error:
            traceback.print_exc()
            errors.append(type(error).__name__ + ': ' + str(error))

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        for cycle in range(100):
            node = rclpy.create_node('sim_lifecycle_probe_subscriber_' + str(cycle))
            executor.add_node(node)
            subscription = node.create_subscription(String, '/acceptance/lifecycle_probe', receive, 10)
            for _ in range(10):
                publisher.publish(String(data='probe'))
                time.sleep(.005)
            node.destroy_subscription(subscription)
            executor.remove_node(node)
            node.destroy_node()
            if (cycle + 1) % 10 == 0:
                print({'cycle': cycle + 1, 'received': received[0]}, flush=True)
            if errors:
                break
        print({'cycles': cycle + 1, 'received': received[0],
               'spin_alive': thread.is_alive(), 'errors': errors}, flush=True)
        assert cycle == 99 and thread.is_alive() and not errors, errors
        assert received[0] > 0, 'No callback observed; inconclusive'
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        publisher_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
